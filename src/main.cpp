// omnibot_drive.ino
// Differential drive controller for Omnibot
// Pico (RP2040) + TB6612 + AS5600 magnetic encoders
// micro-ROS Jazzy over USB Serial
//
// ROS2 interface:
//   Subscribes: /cmd_vel (geometry_msgs/Twist)
//   Publishes:  /odom    (nav_msgs/Odometry)
//   Parameters: kp, ki, kd, pwm_max, pwm_min, pwm_slew, speed_alpha
//
// Serial tuning (only active when micro-ROS agent is NOT connected):
//   p i d t w m a x s r ?
//
// EEPROM: all tunable params saved to flash on change

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>

// ─────────────────────────────────────────────
//  micro-ROS includes
// ─────────────────────────────────────────────
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc_parameter/rclc_parameter.h>
#include <rmw_microros/rmw_microros.h>

#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>

// ─────────────────────────────────────────────
//  Atomic stub — required by micro-ROS on RP2040
// ─────────────────────────────────────────────
extern "C" bool __atomic_test_and_set(volatile void *ptr, int memorder) {
  volatile uint8_t *p = (volatile uint8_t *)ptr;
  bool old = *p;
  *p = 1;
  return old;
}

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

constexpr int I2C0_SDA = 20, I2C0_SCL = 21;
constexpr int I2C1_SDA = 18, I2C1_SCL = 19;

constexpr bool LEFT_FLIP      = false;
constexpr bool RIGHT_FLIP     = false;
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
//  EEPROM layout
//  Simple struct at address 0, prefixed with magic
//  to detect uninitialised flash
// ─────────────────────────────────────────────
constexpr uint32_t EEPROM_MAGIC = 0xDEAD0042;
constexpr int      EEPROM_SIZE  = 256;

struct SavedParams {
  uint32_t magic;
  float kp;
  float ki;
  float kd;
  float pwmMax;
  float pwmMin;
  float pwmSlew;
  float speedAlpha;
};

// ─────────────────────────────────────────────
//  Runtime tunable params
// ─────────────────────────────────────────────
float gKP         = 5.0f;
float gKI         = 1.5f;
float gKD         = 0.0f;
float gPwmMax     = 200.0f;
float gPwmMin     = 30.0f;
float gPwmSlew    = 30.0f;
float gSpeedAlpha = 0.7f;

float gTarget_mm_s = 0.0f;

constexpr uint32_t CTRL_PERIOD_MS  = 20;
constexpr uint32_t PRINT_PERIOD_MS = 100;

// ─────────────────────────────────────────────
//  EEPROM helpers
// ─────────────────────────────────────────────
void eepromLoad() {
  EEPROM.begin(EEPROM_SIZE);
  SavedParams p;
  EEPROM.get(0, p);
  if (p.magic == EEPROM_MAGIC) {
    gKP         = p.kp;
    gKI         = p.ki;
    gKD         = p.kd;
    gPwmMax     = p.pwmMax;
    gPwmMin     = p.pwmMin;
    gPwmSlew    = p.pwmSlew;
    gSpeedAlpha = p.speedAlpha;
    Serial.println("Params loaded from flash.");
  } else {
    Serial.println("No saved params, using defaults.");
  }
}

void eepromSave() {
  SavedParams p;
  p.magic      = EEPROM_MAGIC;
  p.kp         = gKP;
  p.ki         = gKI;
  p.kd         = gKD;
  p.pwmMax     = gPwmMax;
  p.pwmMin     = gPwmMin;
  p.pwmSlew    = gPwmSlew;
  p.speedAlpha = gSpeedAlpha;

  // Only write if changed — earlephilhower EEPROM.put does
  // byte-level comparison so won't burn flash unnecessarily
  SavedParams existing;
  EEPROM.get(0, existing);
  if (memcmp(&p, &existing, sizeof(p)) != 0) {
    EEPROM.put(0, p);
    EEPROM.commit();
    Serial.println("Params saved to flash.");
  }
}

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

void applyGains() {
  left.pid.setGains(gKP, gKI, gKD);
  right.pid.setGains(gKP, gKI, gKD);
}

void setTargetsFromTwist(float vx_m_s, float wz_rad_s) {
  float b = TRACK_MM * 0.001f;
  left.target_mm_s  = (vx_m_s - 0.5f * wz_rad_s * b) * 1000.0f;
  right.target_mm_s = (vx_m_s + 0.5f * wz_rad_s * b) * 1000.0f;
  gTarget_mm_s = vx_m_s * 1000.0f;
}

// ─────────────────────────────────────────────
//  micro-ROS state
// ─────────────────────────────────────────────
enum class AgentState { WAITING, CONNECTED, ERROR };
AgentState agentState = AgentState::WAITING;

rcl_node_t            node;
rclc_support_t        support;
rcl_allocator_t       allocator;
rclc_executor_t       executor;

rcl_publisher_t       odom_pub;
rcl_subscription_t    cmd_vel_sub;
rclc_parameter_server_t param_server;

nav_msgs__msg__Odometry        odom_msg;
geometry_msgs__msg__Twist      cmd_vel_msg;

// ─────────────────────────────────────────────
//  micro-ROS callbacks
// ─────────────────────────────────────────────
void cmdVelCallback(const void *msg) {
  const geometry_msgs__msg__Twist *twist =
    (const geometry_msgs__msg__Twist *)msg;
  setTargetsFromTwist(twist->linear.x, twist->angular.z);
}

bool paramCallback(const Parameter *old_param, const Parameter *new_param, void *context) {
  (void)old_param; (void)context;

  bool changed = false;

  if (strcmp(new_param->name.data, "kp") == 0) {
    gKP = new_param->value.double_value;
    changed = true;
  } else if (strcmp(new_param->name.data, "ki") == 0) {
    gKI = new_param->value.double_value;
    changed = true;
  } else if (strcmp(new_param->name.data, "kd") == 0) {
    gKD = new_param->value.double_value;
    changed = true;
  } else if (strcmp(new_param->name.data, "pwm_max") == 0) {
    gPwmMax = constrain((float)new_param->value.double_value, 50.0f, 255.0f);
    changed = true;
  } else if (strcmp(new_param->name.data, "pwm_min") == 0) {
    gPwmMin = constrain((float)new_param->value.double_value, 0.0f, 100.0f);
    changed = true;
  } else if (strcmp(new_param->name.data, "pwm_slew") == 0) {
    gPwmSlew = constrain((float)new_param->value.double_value, 1.0f, 255.0f);
    changed = true;
  } else if (strcmp(new_param->name.data, "speed_alpha") == 0) {
    gSpeedAlpha = constrain((float)new_param->value.double_value, 0.0f, 0.99f);
    changed = true;
  }

  if (changed) {
    applyGains();
    eepromSave();
  }

  return true;
}

// ─────────────────────────────────────────────
//  micro-ROS init / cleanup
// ─────────────────────────────────────────────
#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { return false; } }

bool microRosInit() {
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(&node, "omnibot_drive", "", &support));

  // Publisher: /odom
  RCCHECK(rclc_publisher_init_default(
    &odom_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom"));

  // Subscriber: /cmd_vel
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // Parameter server
  const rclc_parameter_options_t param_opts = {
    .notify_changed_over_dds        = true,
    .max_params                     = 7,
    .allow_undeclared_parameters    = false,
    .low_mem_mode                   = false
  };
  RCCHECK(rclc_parameter_server_init_with_option(&param_server, &node, &param_opts));

  // Executor: cmd_vel + param server handles
  // param server needs RCLC_PARAMETER_EXECUTOR_HANDLES_NUMBER handles
  RCCHECK(rclc_executor_init(&executor, &support.context,
    1 + RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES, &allocator));

  RCCHECK(rclc_executor_add_subscription(
    &executor, &cmd_vel_sub, &cmd_vel_msg, &cmdVelCallback, ON_NEW_DATA));

  RCCHECK(rclc_executor_add_parameter_server(
    &executor, &param_server, paramCallback));

  // Declare parameters with current values
  RCCHECK(rclc_add_parameter(&param_server, "kp",          RCLC_PARAMETER_DOUBLE));
  RCCHECK(rclc_add_parameter(&param_server, "ki",          RCLC_PARAMETER_DOUBLE));
  RCCHECK(rclc_add_parameter(&param_server, "kd",          RCLC_PARAMETER_DOUBLE));
  RCCHECK(rclc_add_parameter(&param_server, "pwm_max",     RCLC_PARAMETER_DOUBLE));
  RCCHECK(rclc_add_parameter(&param_server, "pwm_min",     RCLC_PARAMETER_DOUBLE));
  RCCHECK(rclc_add_parameter(&param_server, "pwm_slew",    RCLC_PARAMETER_DOUBLE));
  RCCHECK(rclc_add_parameter(&param_server, "speed_alpha", RCLC_PARAMETER_DOUBLE));

  RCCHECK(rclc_parameter_set_double(&param_server, "kp",          gKP));
  RCCHECK(rclc_parameter_set_double(&param_server, "ki",          gKI));
  RCCHECK(rclc_parameter_set_double(&param_server, "kd",          gKD));
  RCCHECK(rclc_parameter_set_double(&param_server, "pwm_max",     gPwmMax));
  RCCHECK(rclc_parameter_set_double(&param_server, "pwm_min",     gPwmMin));
  RCCHECK(rclc_parameter_set_double(&param_server, "pwm_slew",    gPwmSlew));
  RCCHECK(rclc_parameter_set_double(&param_server, "speed_alpha", gSpeedAlpha));

  // Sync Pico clock with ROS agent wall time
  rmw_uros_sync_session(1000);

  return true;
}

void microRosCleanup() {
  rcl_publisher_fini(&odom_pub, &node);
  rcl_subscription_fini(&cmd_vel_sub, &node);
  rclc_parameter_server_fini(&param_server, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void publishOdom() {
  // Timestamp from agent epoch (correct wall clock time)
  int64_t time_ns = rmw_uros_epoch_nanos();
  odom_msg.header.stamp.sec     = (int32_t)(time_ns / 1000000000LL);
  odom_msg.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000LL);

  // Frame IDs — set once in setup, no need to repeat
  odom_msg.pose.pose.position.x = odom.x;
  odom_msg.pose.pose.position.y = odom.y;
  odom_msg.pose.pose.position.z = 0.0;

  // Quaternion from yaw
  odom_msg.pose.pose.orientation.x = 0.0;
  odom_msg.pose.pose.orientation.y = 0.0;
  odom_msg.pose.pose.orientation.z = sinf(odom.th * 0.5f);
  odom_msg.pose.pose.orientation.w = cosf(odom.th * 0.5f);

  odom_msg.twist.twist.linear.x  = odom.vx;
  odom_msg.twist.twist.linear.y  = 0.0;
  odom_msg.twist.twist.angular.z = odom.wz;

  // Pose covariance — diagonal, units metres^2 and radians^2
  odom_msg.pose.covariance[0]  = 0.01;  // x
  odom_msg.pose.covariance[7]  = 0.01;  // y
  odom_msg.pose.covariance[35] = 0.05;  // yaw

  // Twist covariance
  odom_msg.twist.covariance[0]  = 0.01;  // vx
  odom_msg.twist.covariance[35] = 0.05;  // vyaw

  rcl_publish(&odom_pub, &odom_msg, NULL);
}

// ─────────────────────────────────────────────
//  Serial tuning (fallback when agent not connected)
// ─────────────────────────────────────────────
String serialBuf = "";

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

void handleSerial() {
  // Only process serial tuning when micro-ROS agent is not connected
  if (agentState == AgentState::CONNECTED) return;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      serialBuf.trim();
      if (serialBuf.length() == 0) { serialBuf = ""; return; }

      char cmd = tolower(serialBuf[0]);
      float val = serialBuf.substring(1).toFloat();
      bool changed = false;

      switch (cmd) {
        case 'p': gKP = val;                                            changed = true; Serial.print("KP -> ");        Serial.println(gKP, 3);        break;
        case 'i': gKI = val;                                            changed = true; Serial.print("KI -> ");        Serial.println(gKI, 3);        break;
        case 'd': gKD = val;                                            changed = true; Serial.print("KD -> ");        Serial.println(gKD, 3);        break;
        case 'w': gPwmSlew    = constrain(val, 1.0f,   255.0f);        changed = true; Serial.print("pwmSlew -> ");   Serial.println(gPwmSlew, 0);   break;
        case 'm': gPwmMin     = constrain(val, 0.0f,   100.0f);        changed = true; Serial.print("pwmMin -> ");    Serial.println(gPwmMin, 0);    break;
        case 'a': gSpeedAlpha = constrain(val, 0.0f,   0.99f);         changed = true; Serial.print("alpha -> ");     Serial.println(gSpeedAlpha, 2); break;
        case 'x': gPwmMax     = constrain(val, 50.0f,  255.0f);        changed = true; Serial.print("pwmMax -> ");    Serial.println(gPwmMax, 0);    break;
        case 't':
          gTarget_mm_s = val;
          setTargetsFromTwist(val * 0.001f, 0.0f);
          Serial.print("target -> "); Serial.print(gTarget_mm_s, 1); Serial.println(" mm/s");
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

      if (changed) {
        applyGains();
        eepromSave();
      }

      serialBuf = "";
    } else {
      serialBuf += c;
    }
  }
}

// ─────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) delay(10);

  // Load params from flash before anything else
  eepromLoad();

  for (int p : { AIN1, AIN2, PWMA, BIN1, BIN2, PWMB, STBY })
    pinMode(p, OUTPUT);
  digitalWrite(STBY, HIGH);
  analogWriteFreq(20000);
  analogWriteResolution(8);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  Wire.setSDA(I2C0_SDA);  Wire.setSCL(I2C0_SCL);  Wire.setClock(400000);  Wire.begin();
  Wire1.setSDA(I2C1_SDA); Wire1.setSCL(I2C1_SCL); Wire1.setClock(400000); Wire1.begin();

  applyGains();

  // Set up micro-ROS transport (USB serial)
  set_microros_serial_transports(Serial);

  // Initialise odom message header strings
  odom_msg.header.frame_id.data     = (char*)"odom";
  odom_msg.header.frame_id.size     = 4;
  odom_msg.header.frame_id.capacity = 5;
  odom_msg.child_frame_id.data      = (char*)"base_link";
  odom_msg.child_frame_id.size      = 9;
  odom_msg.child_frame_id.capacity  = 10;

  Serial.println("Omnibot drive ready.");
  Serial.println("Waiting for micro-ROS agent...");
  Serial.println();
  printParams();
}

// ─────────────────────────────────────────────
//  Loop
// ─────────────────────────────────────────────
void loop() {
  static uint32_t lastTick      = millis();
  static uint32_t lastAgentPing = millis();

  uint32_t now = millis();

  // ── micro-ROS agent state machine ──
  // Try to connect every 500ms while waiting
  // Detect disconnection and reconnect gracefully
  if (now - lastAgentPing >= 500) {
    lastAgentPing = now;

    switch (agentState) {
      case AgentState::WAITING:
        if (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) {
          if (microRosInit()) {
            agentState = AgentState::CONNECTED;
            Serial.println("[micro-ROS] Agent connected.");
          } else {
            agentState = AgentState::ERROR;
          }
        }
        break;

      case AgentState::CONNECTED:
        if (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) {
          microRosCleanup();
          agentState = AgentState::WAITING;
          left.stop(); right.stop();
          Serial.println("[micro-ROS] Agent disconnected. Waiting...");
        }
        else {
          //Periodically sync clock with agent:
          static uint32_t lastSync = 0;
          if (now - lastSync >= 30000) {
            lastSync = now;
            rmw_uros_sync_session(1000);
          }
        }
        break;

      case AgentState::ERROR:
        microRosCleanup();
        agentState = AgentState::WAITING;
        break;
    }
  }

  // ── Control tick ──
  if (now - lastTick >= CTRL_PERIOD_MS) {
    float dt = (now - lastTick) * 0.001f;
    lastTick = now;

    handleSerial();

    left.sampleEncoder();
    right.sampleEncoder();
    left.update(dt);
    right.update(dt);
    odom.integrate(left.dsMM, right.dsMM, dt);

    if (agentState == AgentState::CONNECTED) {
      publishOdom();
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
    }
  }

  
}