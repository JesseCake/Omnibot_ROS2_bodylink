// omnibot_drive.ino
// Differential drive controller for Omnibot
// Pico (RP2040) + TB6612 + AS5600 magnetic encoders
// Ready for micro-ROS cmd_vel / odom integration
//
// Serial tuning commands:
//   p2.5     set KP to 2.5
//   i0.3     set KI to 0.3
//   d0.0     set KD to 0.0
//   t75      set target speed to 75 mm/s (both wheels forward)
//   w25      set PWM slew limit to 25
//   m30      set PWM minimum (dead zone) to 30
//   a0.7     set EWMA speed filter alpha to 0.7
//   x255     set PWM maximum to 255
//   s        stop
//   r        reset odometry
//   ?        print all current parameters

#include <Arduino.h>
#include <Wire.h>

// ─────────────────────────────────────────────
//  Hardware pins  (TB6612 + Pico GPIO)
// ─────────────────────────────────────────────
constexpr int PWMA  = 2;
constexpr int AIN1  = 5;
constexpr int AIN2  = 4;

constexpr int PWMB  = 3;
constexpr int BIN1  = 6;
constexpr int BIN2  = 7;

constexpr int STBY  = 8;

constexpr int I2C0_SDA = 20, I2C0_SCL = 21;  // Left encoder
constexpr int I2C1_SDA = 18, I2C1_SCL = 19;  // Right encoder

// Motor direction flip — set true if motor runs backwards for positive target
constexpr bool LEFT_FLIP  = false;
constexpr bool RIGHT_FLIP = false;

// Encoder direction flip — set true if encoder reads negative when wheel goes forward
constexpr bool LEFT_ENC_FLIP  = true;
constexpr bool RIGHT_ENC_FLIP = false;

// ─────────────────────────────────────────────
//  Robot geometry
// ─────────────────────────────────────────────
constexpr float WHEEL_DIAM_MM  = 75.0f;
constexpr float WHEEL_CIRC_MM  = 3.14159265f * WHEEL_DIAM_MM;
constexpr float COUNTS_PER_REV = 4096.0f;
constexpr float TRACK_MM       = 170.0f;

// ─────────────────────────────────────────────
//  Control tunables — ALL runtime-adjustable via serial
// ─────────────────────────────────────────────
float gKP          = 5.0f;
float gKI          = 1.5f;
float gKD          = 0.0f;
float gTarget_mm_s = 0.0f;

float gPwmMax      = 255.0f;
float gPwmMin      = 30.0f;
float gPwmSlew     = 15.0f;
float gSpeedAlpha  = 0.7f;

constexpr uint32_t CTRL_PERIOD_MS  = 20;
constexpr uint32_t PRINT_PERIOD_MS = 100;

// ─────────────────────────────────────────────
//  AS5600 helpers
// ─────────────────────────────────────────────
uint16_t readAS5600(TwoWire &bus) {
  bus.beginTransmission(0x36);
  bus.write(0x0E);
  if (bus.endTransmission(false) != 0) return 0xFFFF;
  if (bus.requestFrom(0x36, 2) != 2)   return 0xFFFF;
  uint16_t hi = bus.read();
  uint16_t lo = bus.read();
  return ((hi << 8) | lo) & 0x0FFF;
}

int16_t angleDiff12(uint16_t from, uint16_t to) {
  int16_t d = (int16_t)to - (int16_t)from;
  if (d >  2047) d -= 4096;
  if (d < -2048) d += 4096;
  return d;
}

// ─────────────────────────────────────────────
//  PID
// ─────────────────────────────────────────────
struct PID {
  float kp, ki, kd;
  float integral = 0.0f;
  float prevMeas = 0.0f;
  float lastErr  = 0.0f;

  void reset() { integral = 0.0f; prevMeas = 0.0f; lastErr = 0.0f; }

  void setGains(float p, float i, float d) {
    kp = p; ki = i; kd = d;
    reset();
  }

  float update(float setpoint, float measured, float dt, bool atUpper, bool atLower) {
    lastErr = setpoint - measured;
    bool integrate = !((atUpper && lastErr > 0) || (atLower && lastErr < 0));
    if (integrate) integral += ki * lastErr * dt;
    float deriv = (dt > 0) ? ((measured - prevMeas) / dt) : 0.0f;
    prevMeas = measured;
    return kp * lastErr + integral - kd * deriv;
  }
};

// ─────────────────────────────────────────────
//  Motor drive
// ─────────────────────────────────────────────
struct MotorPins { int ain1, ain2, pwm; };

void driveSigned(const MotorPins &m, int pwmSigned, bool flip) {
  if (flip) pwmSigned = -pwmSigned;
  bool fwd  = (pwmSigned >= 0);
  int  duty = constrain(abs(pwmSigned), 0, (int)gPwmMax);
  digitalWrite(m.ain1, fwd ? LOW  : HIGH);
  digitalWrite(m.ain2, fwd ? HIGH : LOW);
  analogWrite(m.pwm, duty);
}

int slewLimit(int prev, int target, int maxStep) {
  return constrain(target, prev - maxStep, prev + maxStep);
}

// ─────────────────────────────────────────────
//  Per-wheel state
// ─────────────────────────────────────────────
struct Wheel {
  TwoWire   *bus;
  MotorPins  pins;
  bool       motorFlip;
  bool       encFlip;

  bool     encReady  = false;
  uint16_t prevAngle = 0;
  float    dsMM      = 0.0f;

  float target_mm_s  = 0.0f;
  float speedFilt    = 0.0f;
  int   lastPwm      = 0;
  PID   pid;

  void sampleEncoder() {
    uint16_t a = readAS5600(*bus);
    if (a == 0xFFFF) { dsMM = 0.0f; return; }
    if (!encReady) { encReady = true; prevAngle = a; dsMM = 0.0f; return; }
    int16_t ticks = angleDiff12(prevAngle, a);
    prevAngle = a;
    dsMM = ((float)ticks / COUNTS_PER_REV) * WHEEL_CIRC_MM * (encFlip ? -1.0f : 1.0f);
  }

  void update(float dt) {
    float speedRaw = (dt > 0.0f) ? (dsMM / dt) : 0.0f;
    speedFilt = gSpeedAlpha * speedFilt + (1.0f - gSpeedAlpha) * speedRaw;

    bool atUpper = (lastPwm >= (int)gPwmMax);
    bool atLower = (lastPwm <= -(int)gPwmMax);
    float u = pid.update(target_mm_s, speedFilt, dt, atUpper, atLower);

    if (target_mm_s == 0.0f) {
      u = 0.0f;
      pid.reset();
    } else if (fabsf(u) > 0.5f && fabsf(u) < gPwmMin) {
      u = (u > 0) ? gPwmMin : -gPwmMin;
    }

    int pwm = slewLimit(lastPwm, (int)roundf(constrain(u, -gPwmMax, gPwmMax)), (int)gPwmSlew);
    lastPwm = pwm;
    driveSigned(pins, pwm, motorFlip);
  }

  void stop() {
    target_mm_s = 0.0f;
    lastPwm = 0;
    pid.reset();
    driveSigned(pins, 0, motorFlip);
  }
};

// ─────────────────────────────────────────────
//  Odometry
// ─────────────────────────────────────────────
struct Odom {
  float x = 0, y = 0, th = 0, vx = 0, wz = 0;

  void reset() { x = y = th = vx = wz = 0.0f; }

  void integrate(float dsL_mm, float dsR_mm, float dt) {
    float dsL = dsL_mm * 0.001f;
    float dsR = dsR_mm * 0.001f;
    float b   = TRACK_MM * 0.001f;
    float ds  = 0.5f * (dsL + dsR);
    float dth = (dsR - dsL) / b;
    float thMid = th + 0.5f * dth;
    x  += ds * cosf(thMid);
    y  += ds * sinf(thMid);
    th += dth;
    while (th >  M_PI) th -= 2.0f * M_PI;
    while (th < -M_PI) th += 2.0f * M_PI;
    vx = (dt > 0) ? ds  / dt : 0.0f;
    wz = (dt > 0) ? dth / dt : 0.0f;
  }
} odom;

// ─────────────────────────────────────────────
//  Wheel instances
// ─────────────────────────────────────────────
Wheel left  = { &Wire,  { AIN1, AIN2, PWMA }, LEFT_FLIP,  LEFT_ENC_FLIP,  .pid = { gKP, gKI, gKD } };
Wheel right = { &Wire1, { BIN1, BIN2, PWMB }, RIGHT_FLIP, RIGHT_ENC_FLIP, .pid = { gKP, gKI, gKD } };

void setTargetsFromTwist(float vx_m_s, float wz_rad_s) {
  float b = TRACK_MM * 0.001f;
  left.target_mm_s  = (vx_m_s - 0.5f * wz_rad_s * b) * 1000.0f;
  right.target_mm_s = (vx_m_s + 0.5f * wz_rad_s * b) * 1000.0f;
  gTarget_mm_s = vx_m_s * 1000.0f;
}

void applyGains() {
  left.pid.setGains(gKP, gKI, gKD);
  right.pid.setGains(gKP, gKI, gKD);
}

void printParams() {
  Serial.println("--- Current parameters ---");
  Serial.print("  KP=");       Serial.print(gKP, 3);
  Serial.print("  KI=");       Serial.print(gKI, 3);
  Serial.print("  KD=");       Serial.println(gKD, 3);
  Serial.print("  pwmMax=");   Serial.print(gPwmMax, 0);
  Serial.print("  pwmMin=");   Serial.print(gPwmMin, 0);
  Serial.print("  pwmSlew=");  Serial.print(gPwmSlew, 0);
  Serial.print("  alpha=");    Serial.println(gSpeedAlpha, 2);
  Serial.print("  target=");   Serial.print(gTarget_mm_s, 1);
  Serial.println(" mm/s");
  Serial.println("--------------------------");
}

// ─────────────────────────────────────────────
//  Serial command parser
// ─────────────────────────────────────────────
String serialBuf = "";

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      serialBuf.trim();
      if (serialBuf.length() == 0) { serialBuf = ""; return; }

      char cmd = tolower(serialBuf[0]);
      float val = serialBuf.substring(1).toFloat();

      switch (cmd) {
        case 'p':
          gKP = val; applyGains();
          Serial.print("KP -> "); Serial.println(gKP, 3);
          break;
        case 'i':
          gKI = val; applyGains();
          Serial.print("KI -> "); Serial.println(gKI, 3);
          break;
        case 'd':
          gKD = val; applyGains();
          Serial.print("KD -> "); Serial.println(gKD, 3);
          break;
        case 't':
          gTarget_mm_s = val;
          setTargetsFromTwist(val * 0.001f, 0.0f);
          Serial.print("target -> "); Serial.print(gTarget_mm_s, 1); Serial.println(" mm/s");
          break;
        case 'w':
          gPwmSlew = constrain(val, 1.0f, 255.0f);
          Serial.print("pwmSlew -> "); Serial.println(gPwmSlew, 0);
          break;
        case 'm':
          gPwmMin = constrain(val, 0.0f, 100.0f);
          Serial.print("pwmMin -> "); Serial.println(gPwmMin, 0);
          break;
        case 'a':
          gSpeedAlpha = constrain(val, 0.0f, 0.99f);
          Serial.print("alpha -> "); Serial.println(gSpeedAlpha, 2);
          break;
        case 'x':
          gPwmMax = constrain(val, 50.0f, 255.0f);
          Serial.print("pwmMax -> "); Serial.println(gPwmMax, 0);
          break;
        case 's':
          gTarget_mm_s = 0.0f;
          left.stop(); right.stop();
          Serial.println("stopped");
          break;
        case 'r':
          odom.reset();
          Serial.println("odom reset");
          break;
        case '?':
          printParams();
          break;
        default:
          Serial.print("unknown: "); Serial.println(serialBuf);
          break;
      }
      serialBuf = "";
    } else {
      serialBuf += c;
    }
  }
}

// ─────────────────────────────────────────────
//  micro-ROS stubs  (fill in next stage)
// ─────────────────────────────────────────────
// #include <micro_ros_arduino.h>
// #include <nav_msgs/msg/odometry.h>
// #include <geometry_msgs/msg/twist.h>
// rcl_publisher_t odom_pub; rcl_subscription_t cmd_vel_sub;
// rclc_executor_t executor; rclc_support_t support;
// rcl_allocator_t allocator; rcl_node_t node;
//
// void cmd_vel_callback(const void *msg) {
//   const geometry_msgs__msg__Twist *t = (const geometry_msgs__msg__Twist *)msg;
//   setTargetsFromTwist(t->linear.x, t->angular.z);
// }
// void publishOdom() {
//   nav_msgs__msg__Odometry msg = {0};
//   msg.pose.pose.position.x    = odom.x;
//   msg.pose.pose.position.y    = odom.y;
//   msg.pose.pose.orientation.z = sinf(odom.th * 0.5f);
//   msg.pose.pose.orientation.w = cosf(odom.th * 0.5f);
//   msg.twist.twist.linear.x    = odom.vx;
//   msg.twist.twist.angular.z   = odom.wz;
//   rcl_publish(&odom_pub, &msg, NULL);
// }

// ─────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) delay(10);

  for (int p : { AIN1, AIN2, PWMA, BIN1, BIN2, PWMB, STBY })
    pinMode(p, OUTPUT);
  digitalWrite(STBY, HIGH);
  analogWriteFreq(20000);
  analogWriteResolution(8);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  Wire.setSDA(I2C0_SDA);  Wire.setSCL(I2C0_SCL);  Wire.setClock(400000);  Wire.begin();
  Wire1.setSDA(I2C1_SDA); Wire1.setSCL(I2C1_SCL); Wire1.setClock(400000); Wire1.begin();

  Serial.println("Omnibot drive ready.");
  Serial.println("Commands: p i d t w m a x s r ?");
  Serial.println("  p=KP  i=KI  d=KD  t=target(mm/s)");
  Serial.println("  w=pwmSlew  m=pwmMin  a=alpha  x=pwmMax");
  Serial.println("  s=stop  r=reset odom  ?=all params");
  Serial.println();
  printParams();
  Serial.println();
  Serial.println("ms       | tgt    spdL   errL   intL   pwmL  | spdR   errR   intR   pwmR  | odom_x  odom_y  odom_th");
  Serial.println("---------+--------------------------------------+--------------------------------------+---------------------------");
}

// ─────────────────────────────────────────────
//  Loop
// ─────────────────────────────────────────────
void loop() {
  static uint32_t lastTick  = millis();
  static uint32_t lastPrint = millis();

  uint32_t now = millis();

  if (now - lastTick >= CTRL_PERIOD_MS) {
    float dt = (now - lastTick) * 0.001f;
    lastTick = now;

    handleSerial();

    left.sampleEncoder();
    right.sampleEncoder();
    left.update(dt);
    right.update(dt);
    odom.integrate(left.dsMM, right.dsMM, dt);

    // rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
    // publishOdom();
  }

  if (now - lastPrint >= PRINT_PERIOD_MS) {
    lastPrint = now;

    static int lineCount = 0;
    if (++lineCount % 30 == 0) {
      Serial.println();
      Serial.println("ms       | tgt    spdL   errL   intL   pwmL  | spdR   errR   intR   pwmR  | odom_x  odom_y  odom_th");
      Serial.println("---------+--------------------------------------+--------------------------------------+---------------------------");
    }

    char buf[120];
    snprintf(buf, sizeof(buf),
      "%8lu | %6.1f %6.1f %6.1f %6.2f %5d | %6.1f %6.1f %6.2f %5d | %7.3f  %7.3f  %7.3f",
      now,
      left.target_mm_s,
      left.speedFilt,    left.pid.lastErr,  left.pid.integral,  left.lastPwm,
      right.speedFilt,   right.pid.lastErr, right.pid.integral, right.lastPwm,
      odom.x, odom.y, odom.th
    );
    Serial.println(buf);
  }
}