#include <EEPROM.h>
#include <Servo.h>
#include <Encoder.h>
#include <math.h>
#include <PID_v1.h>
#include <Wire.h>
#include <AS5600.h>


const bool DEBUG_COMMANDS = false;
// Keep this false while receiving trajectories. At 9600 baud the original
// output consumed a large fraction of the serial bandwidth; 115200 is now used.
const bool DEBUG_ODOM = false;
const unsigned long ODOM_PRINT_INTERVAL_MS = 250;


// --- Servo Mapping Setup ---
struct JointConfig {
  float angle_min, angle_max;
  float pwm_min, pwm_max;
};

const int ARM_PID_CAL_POINTS = 5;

struct ArmPIDConfig {
  uint8_t i2c_channel;
  float angle_min, angle_max;
  int pwm_min, pwm_max;
  int raw_cal[ARM_PID_CAL_POINTS];
  float angle_cal[ARM_PID_CAL_POINTS];
  double kp, ki, kd;

  // Independent final PWM motion limits for this joint
  // max_pwm_speed:    us/s
  // pwm_acceleration: us/s^2
  // pwm_deceleration: us/s^2
  float max_pwm_speed;
  float pwm_acceleration;
  float pwm_deceleration;

  int last_pwm;
  float current_pwm_float;
  float current_pwm_speed;
};

const JointConfig GRIP = { 0, 90, 1200, 1100 };
const JointConfig HAND = { -70, 80, 920, 1090 };
const JointConfig WRIST = { -90, 90, 1960, 1150 };

float servoAngleToPWM(float angle_min, float angle_max, float pwm_min, float pwm_max, float angle) {
  return pwm_min + (angle - angle_min) * (pwm_max - pwm_min) / (angle_max - angle_min);
}

unsigned long current_millis = 0;
unsigned long prev_millis = 0;
unsigned long command_start_millis = 0;
// MoveIt trajectories in this pipeline are sampled at 10 Hz. Intermediate
// samples are streaming setpoints, not individual stop-and-settle goals.
unsigned long activeCommandId = 0;
// Identifies this power-on/reset. The host compares it across serial reopens to tell a
// reconnect (same boot, odometry intact) from a reset (new boot, odometry re-zeroed).
unsigned long bootId = 0;
unsigned long trajectoryStart = 0;
unsigned long settledSince = 0;
unsigned long lastStateMillis = 0;
unsigned long lastHostMillis = 0;
const unsigned long SETTLE_DWELL_MS = 500;
const unsigned long STATE_INTERVAL_MS = 50;
float previousSettleX = 0, previousSettleY = 0, previousSettleTheta = 0;
float previousSettleElbow = 0, previousSettleShoulder = 0;
// Abort rather than silently advancing when the final goal cannot be reached.
unsigned long command_timeout = 5000;
unsigned long lastOdomPrintMillis = 0;

float vx, vy, omega;
float target_x, target_y, target_theta;
float x, y, theta;
float dx, dy, dtheta;

int32_t prev_nw = 0, prev_ne = 0, prev_sw = 0, prev_se = 0;
// About 0.20 cm with the current encoder geometry. A 10-count threshold
// rejected otherwise completed goals for residual errors as small as 1.3 mm.
int drive_threshold = 20;
int theta_threshold = 10;
const float THETA_TRANSLATING_OUTPUT_LIMIT = 0.45;
// Maximum per-axis lag while following a trajectory, not endpoint tolerance.
const float BASE_TRACKING_TOLERANCE_CM = 6.0;
// The drive motors do not move below this normalized command. Map every
// nonzero controller output into the effective motor range instead of turning
// small corrections into zero power.
const float MIN_DRIVE_COMMAND = 0.35;
// The original single threshold mixed PWM microseconds with degrees.
const float SERVO_PWM_THRESHOLD_US = 2.0;
const float ELBOW_ANGLE_THRESHOLD_DEG = 0.5;
const float SHOULDER_ANGLE_THRESHOLD_DEG = 0.5;
float la_threshold = 1.0;

// --- Arm joint PID control ---
#define PCA9548A_ADDR 0x70
#define ELBOW_I2C_CHANNEL 1
#define SHOULDER_I2C_CHANNEL 0

const int ARM_PID_NUM_SAMPLES = 1;
const int ARM_PID_CORRECTION_LIMIT = 180;
const float ARM_PID_ENABLE_WINDOW = 10.0;
const float ARM_TARGET_TOLERANCE = 0.5;
const unsigned long ARM_PID_CONTROL_INTERVAL = 20;

// --- PID GAINS ---
double Kp_x = 0.0024;
double Ki_x = 0.00001;
double Kd_x = 0.00002;

double Kp_y = 0.002;
double Ki_y = 0.0000;
double Kd_y = 0.00001;
double Kp_th = 0.0053, Ki_th = 0, Kd_th = 0.00004;

// --- PID variables ---
double xInput = 0, xOutput = 0, xSetpoint = 0;
double yInput = 0, yOutput = 0, ySetpoint = 0;
double thInput = 0, thOutput = 0, thSetpoint = 0;

PID pidX(&xInput, &xOutput, &xSetpoint, Kp_x, Ki_x, Kd_x, DIRECT);
PID pidY(&yInput, &yOutput, &ySetpoint, Kp_y, Ki_y, Kd_y, DIRECT);
PID pidTh(&thInput, &thOutput, &thSetpoint, Kp_th, Ki_th, Kd_th, DIRECT);

float wheel_diameter = 0.097;
float base_radius = 0.31;
float wheel_circumference = PI * wheel_diameter;
float resolution = 3100;

#define SERIAL_BUF_SIZE 2048
char serialBuffer[SERIAL_BUF_SIZE];
int serialBufIdx = 0;
bool serialBufferOverflow = false;
const int COMMAND_VALUE_COUNT = 11; // Last field: time from start in milliseconds.
const int MAX_COMMANDS = 45;

// Fixed-size numeric storage avoids heap fragmentation from an array of
// Arduino String objects. Forty-five waypoints fit every current motion stage
// while remaining within the Mega's SRAM budget alongside the 2048-byte RX
// buffer.
struct CommandWaypoint {
  float value[COMMAND_VALUE_COUNT];
};

CommandWaypoint commandQueue[MAX_COMMANDS];
bool streamActive = false;
bool streamFinal = true;
bool streamTransit = false;
unsigned long streamSequence = 0;
int commandCount = 0;
int currentCommandIndex = 0;

// === Servo Setup ===
Servo grip, hand, wrist, elbow, shoulder;
AS5600 armEncoder;
float grip_target, hand_target, wrist_target, elbow_target, shoulder_target;
float grip_pos, hand_pos, wrist_pos, elbow_pos, shoulder_pos;
float dgrip, dhand, dwrist, delbow, dshoulder;

ArmPIDConfig ELBOW_PID = {
  ELBOW_I2C_CHANNEL,
  -35,
  145,
  600,
  2365,
  {3698, 3412, 3068, 2733, 1776},
  {-35, 0, 30, 60, 145},
  10,
  10,
  0.1,
  400.0,
  400.0,
  600.0,
  1500,
  1500.0,
  0.0
};

ArmPIDConfig SHOULDER_PID = {
  SHOULDER_I2C_CHANNEL,
  -30,
  150,
  605,
  2360,
  {2595, 2240, 1921, 892, 547},
  {-30, 0, 30, 120, 150},
  2.0,
  10,
  0.1,
  300.0,
  300.0,
  600.0,
  1500,
  1500.0,
  0.0
};

double elbowInput = 0, elbowOutput = 0, elbowSetpoint = 0;
double shoulderInput = 0, shoulderOutput = 0, shoulderSetpoint = 0;
PID elbowPID(&elbowInput, &elbowOutput, &elbowSetpoint, ELBOW_PID.kp, ELBOW_PID.ki, ELBOW_PID.kd, DIRECT);
PID shoulderPID(&shoulderInput, &shoulderOutput, &shoulderSetpoint, SHOULDER_PID.kp, SHOULDER_PID.ki, SHOULDER_PID.kd, DIRECT);

int elbow_pwm = 1500;
int shoulder_pwm = 1500;
unsigned long lastArmPIDMillis = 0;

// === Drive Setup ===
#define NW_PWM 6
#define NW_CW 49
#define NW_CCW 48
Encoder NW_ENCODER(19, 23);

#define NE_PWM 5
#define NE_CW 50
#define NE_CCW 51
Encoder NE_ENCODER(18, 22);

#define SW_PWM 4
#define SW_CW 53
#define SW_CCW 52
Encoder SW_ENCODER(2, 15);

#define SE_PWM 7
#define SE_CW 46
#define SE_CCW 47
Encoder SE_ENCODER(3, 14);

// === Linear Actuator Setup ===
float la_target, frame_target;
float la_pos, frame_pos;
float dla, dframe;
float vla, vframe;

#define LA_DOWN 44
#define LA_UP 45
#define LA_PWM 41

#define FRAME_UP 42
#define FRAME_DOWN 43
#define FRAME_PWM 40

#define RELAY_PIN 8

#define RELAY_ON HIGH
#define RELAY_OFF LOW

void selectArmI2CChannel(uint8_t channel);
int readArmAverageRaw(uint8_t channel);
int armAngleToBasePWM(ArmPIDConfig& joint, float angle);
void applyArmPIDTunings();
void resetArmPID(PID& pid, double& output);
int limitArmPWMChange(ArmPIDConfig& joint, int desired_pwm, float dt_sec);
void updateArmPID(unsigned long now);
void parseMessage(const CommandWaypoint& command);
void parseCommandQueue(char* input);
void parseStreamPacket(char* input);
void printRobotState();
void stopAtCurrentState();
void protocolError(const char* reason);
void printBootId();
void zeroOdometry();

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF);

  // Must match arduino_baud in gesture_control.launch.py.
  Serial.begin(115200);

  Wire.begin();
  selectArmI2CChannel(ELBOW_I2C_CHANNEL);
  armEncoder.begin();

  float init_grip = servoAngleToPWM(GRIP.angle_min, GRIP.angle_max, GRIP.pwm_min, GRIP.pwm_max, 0);
  float init_hand = servoAngleToPWM(HAND.angle_min, HAND.angle_max, HAND.pwm_min, HAND.pwm_max, 0);
  float init_wrist = servoAngleToPWM(WRIST.angle_min, WRIST.angle_max, WRIST.pwm_min, WRIST.pwm_max, 0);
  float init_elbow = armAngleToBasePWM(ELBOW_PID, 0);
  float init_shoulder = armAngleToBasePWM(SHOULDER_PID, 0);

  grip_pos = init_grip;
  hand_pos = init_hand;
  wrist_pos = init_wrist;
  elbow_pos = 0;
  shoulder_pos = 0;

  grip_target = init_grip;
  hand_target = init_hand;
  wrist_target = init_wrist;
  elbow_target = 0;
  shoulder_target = 0;
  elbow_pwm = (int)init_elbow;
  shoulder_pwm = (int)init_shoulder;

  ELBOW_PID.last_pwm = elbow_pwm;
  ELBOW_PID.current_pwm_float = elbow_pwm;
  ELBOW_PID.current_pwm_speed = 0.0;

  SHOULDER_PID.last_pwm = shoulder_pwm;
  SHOULDER_PID.current_pwm_float = shoulder_pwm;
  SHOULDER_PID.current_pwm_speed = 0.0;

  grip.attach(9);
  grip.writeMicroseconds((int)init_grip);

  hand.attach(13);
  hand.writeMicroseconds((int)init_hand);

  wrist.attach(12);
  wrist.writeMicroseconds((int)init_wrist);

  elbow.attach(11);
  elbow.writeMicroseconds(elbow_pwm);

  shoulder.attach(10);
  shoulder.writeMicroseconds(shoulder_pwm);

  la_target = 0;
  la_pos = 0;
  frame_target = 0;
  frame_pos = 0;

  pidX.SetSampleTime(5);
  pidY.SetSampleTime(5);
  pidTh.SetSampleTime(5);

  pidX.SetOutputLimits(-1.0, 1.0);
  pidY.SetOutputLimits(-1.0, 1.0);
  pidTh.SetOutputLimits(-1.0, 1.0);

  pidX.SetMode(AUTOMATIC);
  pidY.SetMode(AUTOMATIC);
  pidTh.SetMode(AUTOMATIC);

  elbowPID.SetSampleTime(ARM_PID_CONTROL_INTERVAL);
  shoulderPID.SetSampleTime(ARM_PID_CONTROL_INTERVAL);
  elbowPID.SetOutputLimits(-ARM_PID_CORRECTION_LIMIT, ARM_PID_CORRECTION_LIMIT);
  shoulderPID.SetOutputLimits(-ARM_PID_CORRECTION_LIMIT, ARM_PID_CORRECTION_LIMIT);
  elbowPID.SetMode(AUTOMATIC);
  shoulderPID.SetMode(AUTOMATIC);
  applyArmPIDTunings();
  lastArmPIDMillis = millis();

  prev_nw = NW_ENCODER.read();
  prev_ne = NE_ENCODER.read();
  prev_sw = SW_ENCODER.read();
  prev_se = SE_ENCODER.read();

  delay(1000);
  digitalWrite(RELAY_PIN, RELAY_ON);

  // Floating analog input plus boot timing: distinct across resets, no EEPROM wear.
  bootId = (micros() ^ ((unsigned long)analogRead(A0) << 20) ^ ((unsigned long)analogRead(A1) << 10)) & 0xFFFFFFUL;
  if (bootId == 0) bootId = 1;

  Serial.println("READY4");
  printBootId();
}

void loop() {
  prev_millis = current_millis;
  current_millis = millis();

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (serialBufferOverflow) {
        stopAtCurrentState();
        protocolError("RX_OVERFLOW");
      } else {
        serialBuffer[serialBufIdx] = '\0';
        if (serialBufIdx > 0) {
          lastHostMillis = current_millis;
          if (strcmp(serialBuffer, "PING") == 0) {
            // Heartbeat never changes a target.
          } else if (strcmp(serialBuffer, "STOP") == 0) {
            stopAtCurrentState();
            protocolError("STOPPED");
          } else if (strcmp(serialBuffer, "ID") == 0) {
            printBootId();
          } else if (strcmp(serialBuffer, "ZERO") == 0) {
            zeroOdometry();
          } else if (strncmp(serialBuffer, "STR ", 4) == 0) {
            parseStreamPacket(serialBuffer);
          } else if (commandCount > 0) {
            // Do not replace or acknowledge the active command with a new one.
            Serial.println("BUSY2");
          } else {
            parseCommandQueue(serialBuffer);
          }
        }
      }
      serialBufIdx = 0;
      serialBufferOverflow = false;
    } else if (c != '\r') {
      if (serialBufIdx < SERIAL_BUF_SIZE - 1) {
        serialBuffer[serialBufIdx++] = c;
        // Credit only bytes already removed from the AVR hardware RX ring.
        // Host sends at most 32 bytes, then waits for this acknowledgement.
        if (serialBufIdx % 32 == 0 && serialBuffer[0] == 'S' &&
            serialBuffer[1] == 'T' && serialBuffer[2] == 'R') {
          Serial.print("RX4 "); Serial.println(serialBufIdx);
        }
      } else {
        serialBufferOverflow = true;
      }
    }
  }

  update_pos();
  updateArmPID(current_millis);

  if (commandCount > 0 && current_millis - lastHostMillis > 1500) {
    stopAtCurrentState(); protocolError("HOST_LOST");
  }
  bool queueActive = commandCount > 0;

  unsigned long elapsed = current_millis - trajectoryStart;
  // Coarse positioning accepts lag during the move and verifies the settled
  // endpoint below. Contact/precision motion still enforces path tracking.
  const bool checkMotionTracking = !(streamActive && streamTransit);
  if (queueActive && checkMotionTracking && (fabs(linearCountsToCm(dx)) > BASE_TRACKING_TOLERANCE_CM ||
      fabs(linearCountsToCm(dy)) > BASE_TRACKING_TOLERANCE_CM ||
      fabs(thetaCountsToDeg(dtheta)) > 2.0 || fabs(delbow) > 3.0 || fabs(dshoulder) > 3.0)) {
    // Capture the actual tracking errors before stop replaces the targets.
    stopAtCurrentState();
    printRobotState();
    Serial.print("ERROR "); Serial.print(activeCommandId);
    Serial.print(" TRACKING_ERROR dx_cm="); Serial.print(linearCountsToCm(dx), 3);
    Serial.print(" dy_cm="); Serial.print(linearCountsToCm(dy), 3);
    Serial.print(" yaw_deg="); Serial.print(thetaCountsToDeg(dtheta), 3);
    Serial.print(" elbow_deg="); Serial.print(delbow, 3);
    Serial.print(" shoulder_deg="); Serial.println(dshoulder, 3);
    queueActive = false;
  }
  if (queueActive) {
    // Interpolate the time-parameterized trajectory instead of applying every
    // point at a fixed cadence. Intermediate samples are not stop goals.
    while (currentCommandIndex < commandCount &&
           elapsed >= (unsigned long)commandQueue[currentCommandIndex].value[10]) {
      currentCommandIndex++;
    }
    if (currentCommandIndex < commandCount) {
      const CommandWaypoint& a = commandQueue[currentCommandIndex - 1];
      const CommandWaypoint& b = commandQueue[currentCommandIndex];
      float u = (elapsed - a.value[10]) / (b.value[10] - a.value[10]);
      CommandWaypoint interpolated;
      for (int j = 0; j < 10; ++j)
        interpolated.value[j] = a.value[j] + u * (b.value[j] - a.value[j]);
      parseMessage(interpolated);
      settledSince = 0;
    } else if (streamActive && !streamFinal) {
      stopAtCurrentState();
      protocolError("BUFFER_UNDERRUN");
    } else {
      parseMessage(commandQueue[commandCount - 1]);
      updateBaseError();
      // Require both small error and no significant measured movement over a
      // dwell. Servo wrist/hand/gripper values remain estimates, not encoders.
      bool quiet = fabs(linearCountsToCm(x - previousSettleX)) < 0.1 &&
                   fabs(linearCountsToCm(y - previousSettleY)) < 0.1 &&
                   fabs(thetaCountsToDeg(theta - previousSettleTheta)) < 0.1 &&
                   fabs(elbow_pos - previousSettleElbow) < 0.15 &&
                   fabs(shoulder_pos - previousSettleShoulder) < 0.15;
      if (!status() || !quiet || settledSince == 0) {
        settledSince = (status() && quiet) ? current_millis : 0;
        previousSettleX = x; previousSettleY = y; previousSettleTheta = theta;
        previousSettleElbow = elbow_pos; previousSettleShoulder = shoulder_pos;
      }
      if (settledSince && current_millis - settledSince >= SETTLE_DWELL_MS) {
        printRobotState();
        Serial.print("DONE "); Serial.println(activeCommandId);
        // Coarse stages accept this measured configuration; stop hunting the
        // nominal endpoint. The next stage replans from fresh feedback.
        if (streamActive && streamTransit) stopAtCurrentState();
        commandCount = 0; currentCommandIndex = 0; streamActive = false;
      } else if (elapsed > (unsigned long)commandQueue[commandCount - 1].value[10] + command_timeout) {
        stopAtCurrentState();
        printRobotState();
        protocolError("SETTLE_TIMEOUT");
      }
    }
  }
  if (current_millis - lastStateMillis >= STATE_INTERVAL_MS) {
    printRobotState(); lastStateMillis = current_millis;
  }

  updateBaseError();
  printBaseStatus();

  xInput = (double)x;
  yInput = (double)y;
  thInput = (double)theta;

  xSetpoint = (double)target_x;
  ySetpoint = (double)target_y;
  thSetpoint = (double)target_theta;

  pidX.Compute();
  pidY.Compute();
  pidTh.Compute();

  if (abs(dx) < drive_threshold) xOutput = 0.0;
  if (abs(dy) < drive_threshold) yOutput = 0.0;
  if (abs(dtheta) < theta_threshold) thOutput = 0.0;

  if (abs(dx) > drive_threshold || abs(dy) > drive_threshold) {
    thOutput = constrain(thOutput, -THETA_TRANSLATING_OUTPUT_LIMIT, THETA_TRANSLATING_OUTPUT_LIMIT);
  }

  vx = (float)xOutput;
  vy = (float)yOutput;
  omega = (float)thOutput;

  float heading_rad = (theta / resolution) * wheel_circumference / base_radius;
  float vx_body = cos(heading_rad) * vx + sin(heading_rad) * vy;
  float vy_body = -sin(heading_rad) * vx + cos(heading_rad) * vy;

  drive(vx_body, vy_body, omega);

  grip.writeMicroseconds((int)grip_pos);
  hand.writeMicroseconds((int)hand_pos);
  wrist.writeMicroseconds((int)wrist_pos);

  controlLA(vla);
  controlFrame(vframe);
}

bool status() {
  if (streamActive && streamTransit) {
    return fabs(linearCountsToCm(dx)) < 0.5 && fabs(linearCountsToCm(dy)) < 0.5 &&
      fabs(thetaCountsToDeg(dtheta)) < 0.5 && fabs(delbow) < 1.0 && fabs(dshoulder) < 1.0 &&
      fabs(dgrip) <= SERVO_PWM_THRESHOLD_US && fabs(dhand) <= SERVO_PWM_THRESHOLD_US &&
      fabs(dwrist) <= SERVO_PWM_THRESHOLD_US && fabs(dla) < la_threshold && fabs(dframe) < la_threshold;
  }
  return (
    abs(dx) < drive_threshold &&
    abs(dy) < drive_threshold &&
    abs(dtheta) < theta_threshold &&
    abs(dgrip) <= SERVO_PWM_THRESHOLD_US &&
    abs(dhand) <= SERVO_PWM_THRESHOLD_US &&
    abs(dwrist) <= SERVO_PWM_THRESHOLD_US &&
    abs(delbow) < ELBOW_ANGLE_THRESHOLD_DEG &&
    abs(dshoulder) < SHOULDER_ANGLE_THRESHOLD_DEG &&
    abs(dla) < la_threshold &&
    abs(dframe) < la_threshold
  );
}

float linearCountsToCm(float counts) {
  return (counts / resolution) * wheel_circumference * 100.0;
}

float thetaCountsToDeg(float counts) {
  return ((counts / resolution) * wheel_circumference / base_radius) * (180.0 / PI);
}

void updateBaseError() {
  dx = target_x - x;
  dy = target_y - y;
  dtheta = target_theta - theta;
}

void printBaseStatus() {
  if (!DEBUG_ODOM) return;
  if (current_millis - lastOdomPrintMillis < ODOM_PRINT_INTERVAL_MS) return;
  lastOdomPrintMillis = current_millis;

  Serial.print("POS x_cm=");
  Serial.print(linearCountsToCm(x), 2);
  Serial.print(" y_cm=");
  Serial.print(linearCountsToCm(y), 2);
  Serial.print(" th_deg=");
  Serial.print(thetaCountsToDeg(theta), 2);

  Serial.print(" || TGT x_cm=");
  Serial.print(linearCountsToCm(target_x), 2);
  Serial.print(" y_cm=");
  Serial.print(linearCountsToCm(target_y), 2);
  Serial.print(" th_deg=");
  Serial.print(thetaCountsToDeg(target_theta), 2);

  Serial.print(" || ERR dx_cm=");
  Serial.print(linearCountsToCm(dx), 2);
  Serial.print(" dy_cm=");
  Serial.print(linearCountsToCm(dy), 2);
  Serial.print(" dth_deg=");
  Serial.println(thetaCountsToDeg(dtheta), 2);
}

void selectArmI2CChannel(uint8_t channel) {
  Wire.beginTransmission(PCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delayMicroseconds(200);
}

int readArmAverageRaw(uint8_t channel) {
  long sum = 0;
  selectArmI2CChannel(channel);

  for (int i = 0; i < ARM_PID_NUM_SAMPLES; i++) {
    sum += armEncoder.readAngle();
    delayMicroseconds(500);
  }

  return sum / ARM_PID_NUM_SAMPLES;
}

float armRawToAngle(ArmPIDConfig& joint, int raw) {
  bool increasing = joint.raw_cal[ARM_PID_CAL_POINTS - 1] > joint.raw_cal[0];

  if (increasing) {
    if (raw <= joint.raw_cal[0]) return joint.angle_cal[0];
    if (raw >= joint.raw_cal[ARM_PID_CAL_POINTS - 1]) return joint.angle_cal[ARM_PID_CAL_POINTS - 1];

    for (int i = 0; i < ARM_PID_CAL_POINTS - 1; i++) {
      if (raw >= joint.raw_cal[i] && raw <= joint.raw_cal[i + 1]) {
        return joint.angle_cal[i] +
               (raw - joint.raw_cal[i]) *
               (joint.angle_cal[i + 1] - joint.angle_cal[i]) /
               (joint.raw_cal[i + 1] - joint.raw_cal[i]);
      }
    }
  } else {
    if (raw >= joint.raw_cal[0]) return joint.angle_cal[0];
    if (raw <= joint.raw_cal[ARM_PID_CAL_POINTS - 1]) return joint.angle_cal[ARM_PID_CAL_POINTS - 1];

    for (int i = 0; i < ARM_PID_CAL_POINTS - 1; i++) {
      if (raw <= joint.raw_cal[i] && raw >= joint.raw_cal[i + 1]) {
        return joint.angle_cal[i] +
               (raw - joint.raw_cal[i]) *
               (joint.angle_cal[i + 1] - joint.angle_cal[i]) /
               (joint.raw_cal[i + 1] - joint.raw_cal[i]);
      }
    }
  }

  return joint.angle_cal[0];
}

int armAngleToBasePWM(ArmPIDConfig& joint, float angle) {
  angle = constrain(angle, joint.angle_min, joint.angle_max);
  float pwm = joint.pwm_min +
              (angle - joint.angle_min) *
              (joint.pwm_max - joint.pwm_min) /
              (joint.angle_max - joint.angle_min);

  return constrain((int)pwm, joint.pwm_min, joint.pwm_max);
}

void resetArmPID(PID& pid, double& output) {
  output = 0;
  pid.SetMode(MANUAL);
  pid.SetMode(AUTOMATIC);
}

void applyArmPIDTunings() {
  elbowPID.SetTunings(ELBOW_PID.kp, ELBOW_PID.ki, ELBOW_PID.kd);
  shoulderPID.SetTunings(SHOULDER_PID.kp, SHOULDER_PID.ki, SHOULDER_PID.kd);
}

void writeArmPWM(ArmPIDConfig& joint, Servo& servo, int pwm) {
  pwm = constrain(pwm, joint.pwm_min, joint.pwm_max);
  joint.last_pwm = pwm;
  servo.writeMicroseconds(pwm);
}

int limitArmPWMChange(ArmPIDConfig& joint, int desired_pwm, float dt_sec) {
  desired_pwm = constrain(desired_pwm, joint.pwm_min, joint.pwm_max);

  float pwm_error = desired_pwm - joint.current_pwm_float;
  float distance = abs(pwm_error);

  if (distance < 0.5) {
    joint.current_pwm_float = desired_pwm;
    joint.current_pwm_speed = 0.0;
    return desired_pwm;
  }

  float direction = (pwm_error > 0) ? 1.0 : -1.0;
  float allowed_speed = sqrt(2.0 * joint.pwm_deceleration * distance);
  allowed_speed = min(allowed_speed, joint.max_pwm_speed);

  if (joint.current_pwm_speed * direction < 0.0) {
    float new_magnitude =
      abs(joint.current_pwm_speed) - joint.pwm_deceleration * dt_sec;

    if (new_magnitude <= 0.0) {
      joint.current_pwm_speed = 0.0;
    } else {
      joint.current_pwm_speed = -direction * new_magnitude;
    }
  } else {
    float speed_magnitude = abs(joint.current_pwm_speed);

    if (speed_magnitude < allowed_speed) {
      speed_magnitude += joint.pwm_acceleration * dt_sec;
      speed_magnitude = min(speed_magnitude, allowed_speed);
    } else if (speed_magnitude > allowed_speed) {
      speed_magnitude -= joint.pwm_deceleration * dt_sec;
      speed_magnitude = max(speed_magnitude, allowed_speed);
    }

    speed_magnitude = min(speed_magnitude, joint.max_pwm_speed);
    joint.current_pwm_speed = direction * speed_magnitude;
  }

  float next_pwm =
    joint.current_pwm_float + joint.current_pwm_speed * dt_sec;

  if ((pwm_error > 0 && next_pwm >= desired_pwm) ||
      (pwm_error < 0 && next_pwm <= desired_pwm)) {
    next_pwm = desired_pwm;
    joint.current_pwm_speed = 0.0;
  }

  next_pwm = constrain(next_pwm,
                       (float)joint.pwm_min,
                       (float)joint.pwm_max);

  if (next_pwm <= joint.pwm_min || next_pwm >= joint.pwm_max) {
    joint.current_pwm_speed = 0.0;
  }

  joint.current_pwm_float = next_pwm;
  return (int)round(joint.current_pwm_float);
}

void updateArmJointPID(ArmPIDConfig& joint,
                       Servo& servo,
                       PID& pid,
                       double& input,
                       double& output,
                       double& setpoint,
                       double command_angle,
                       float& measured_angle,
                       float& joint_error,
                       int& current_pwm,
                       float dt_sec) {
  int raw = readArmAverageRaw(joint.i2c_channel);
  measured_angle = armRawToAngle(joint, raw);

  input = measured_angle;
  setpoint = command_angle;
  joint_error = command_angle - measured_angle;

  int base_pwm = armAngleToBasePWM(joint, command_angle);
  int requested_pwm = current_pwm;

  if (abs(joint_error) <= ARM_TARGET_TOLERANCE) {
    // Keep the correction that established this hold; zeroing it causes a
    // PWM discontinuity on the first update outside the deadband.
    joint.current_pwm_float = current_pwm;
    joint.current_pwm_speed = 0.0;
    writeArmPWM(joint, servo, current_pwm);
    return;
  }

  if (abs(joint_error) > ARM_PID_ENABLE_WINDOW) {
    output = 0;
    requested_pwm = base_pwm;

    current_pwm =
      limitArmPWMChange(joint, requested_pwm, dt_sec);

    writeArmPWM(joint, servo, current_pwm);
    resetArmPID(pid, output);
    return;
  }

  pid.Compute();

  requested_pwm =
    constrain((int)(base_pwm + output),
              joint.pwm_min,
              joint.pwm_max);

  current_pwm =
    limitArmPWMChange(joint, requested_pwm, dt_sec);

  writeArmPWM(joint, servo, current_pwm);
}

void updateArmPID(unsigned long now) {
  if (lastArmPIDMillis == 0) {
    lastArmPIDMillis = now;
    return;
  }

  if (now - lastArmPIDMillis < ARM_PID_CONTROL_INTERVAL) return;

  float dt_sec = (now - lastArmPIDMillis) / 1000.0;
  lastArmPIDMillis = now;

  updateArmJointPID(ELBOW_PID, elbow, elbowPID, elbowInput, elbowOutput, elbowSetpoint,
                    elbow_target, elbow_pos, delbow, elbow_pwm, dt_sec);
  updateArmJointPID(SHOULDER_PID, shoulder, shoulderPID, shoulderInput, shoulderOutput, shoulderSetpoint,
                    shoulder_target, shoulder_pos, dshoulder, shoulder_pwm, dt_sec);
}

void update_pos() {
  float dt = current_millis - prev_millis;

  int32_t nw = NW_ENCODER.read();
  int32_t ne = NE_ENCODER.read();
  int32_t sw = SW_ENCODER.read();
  int32_t se = SE_ENCODER.read();

  int32_t d_nw = nw - prev_nw;
  int32_t d_ne = ne - prev_ne;
  int32_t d_sw = sw - prev_sw;
  int32_t d_se = se - prev_se;

  prev_nw = nw;
  prev_ne = ne;
  prev_sw = sw;
  prev_se = se;

  float dtheta_counts = -((+d_nw + d_ne + d_sw + d_se) / 4.0);
  float dx_body = ((+d_nw - d_ne + d_sw - d_se) / 4.0) * sqrt(2);
  float dy_body = ((-d_nw - d_ne + d_sw + d_se) / 4.0) * sqrt(2);

  float theta_mid_counts = theta + 0.5 * dtheta_counts;
  float theta_mid_rad = (theta_mid_counts / resolution) * wheel_circumference / base_radius;

  x += cos(theta_mid_rad) * dx_body - sin(theta_mid_rad) * dy_body;
  y += sin(theta_mid_rad) * dx_body + cos(theta_mid_rad) * dy_body;
  theta += dtheta_counts;

  updateBaseError();

  dgrip = grip_target - grip_pos;
  dhand = hand_target - hand_pos;
  dwrist = wrist_target - wrist_pos;
  if (abs(dgrip) > SERVO_PWM_THRESHOLD_US) {
    grip_pos = moveToward(grip_pos, grip_target, dt * 0.2);
  }
  if (abs(dhand) > SERVO_PWM_THRESHOLD_US) {
    hand_pos = moveToward(hand_pos, hand_target, dt * 0.2);
  }
  if (abs(dwrist) > SERVO_PWM_THRESHOLD_US) {
    wrist_pos = moveToward(wrist_pos, wrist_target, dt * 0.2);
  }

  dla = la_target - la_pos;
  dframe = frame_target - frame_pos;

  vla = (abs(dla) > la_threshold) ? sgn(dla) : 0;
  vframe = (abs(dframe) > la_threshold) ? sgn(dframe) : 0;

  if (abs(dla) > la_threshold) {
    la_pos += (sgn(dla) * dt * 0.00871);
  }
  if (abs(dframe) > la_threshold) {
    frame_pos += (sgn(dframe) * dt * 0.0091455);
  }
}

void parseMessage(const CommandWaypoint& command) {
  float x_ = 0.01 * command.value[0];
  float y_ = 0.01 * command.value[1];
  float theta_ = command.value[2];
  float grip_ = command.value[3];
  float hand_ = command.value[4];
  float wrist_ = command.value[5];
  float elbow_ = command.value[6];
  float shoulder_ = command.value[7];
  float la_ = command.value[8];
  float frame_ = command.value[9];

  target_x = (x_ / wheel_circumference) * resolution;
  target_y = (y_ / wheel_circumference) * resolution;
  target_theta = (((PI / 180) * base_radius * theta_) / wheel_circumference) * resolution;

  grip_target = servoAngleToPWM(GRIP.angle_min, GRIP.angle_max, GRIP.pwm_min, GRIP.pwm_max, grip_);
  hand_target = servoAngleToPWM(HAND.angle_min, HAND.angle_max, HAND.pwm_min, HAND.pwm_max, hand_);
  wrist_target = servoAngleToPWM(WRIST.angle_min, WRIST.angle_max, WRIST.pwm_min, WRIST.pwm_max, wrist_);
  elbow_target = constrain(elbow_, ELBOW_PID.angle_min, ELBOW_PID.angle_max);
  shoulder_target = constrain(shoulder_, SHOULDER_PID.angle_min, SHOULDER_PID.angle_max);
  applyArmPIDTunings();
  la_target = constrain(la_, 0, 435);
  frame_target = constrain(frame_, 0, 150);
}

void protocolError(const char* reason) {
  Serial.print("ERROR "); Serial.print(activeCommandId);
  Serial.print(" "); Serial.println(reason);
}

// "BOOT <id> ZERO": boot identity plus the capabilities this firmware supports.
void printBootId() {
  Serial.print("BOOT "); Serial.print(bootId); Serial.println(" ZERO");
}

// Re-zero without a reset: odometry and the held base target become the current pose, so the
// base PID holds still instead of driving back, and the arm is untouched (no re-homing).
void zeroOdometry() {
  if (commandCount > 0 || streamActive) {
    Serial.println("ERROR 0 ZERO_BUSY");
    return;
  }
  x = 0; y = 0; theta = 0;
  target_x = 0; target_y = 0; target_theta = 0;
  dx = 0; dy = 0; dtheta = 0;
  previousSettleX = 0; previousSettleY = 0; previousSettleTheta = 0;
  settledSince = 0;
  // PID_v1 keeps lastInput and the integral; re-initialize so the jump in measured
  // position is not seen as motion (derivative kick) and no stale integral remains.
  xInput = 0; yInput = 0; thInput = 0;
  xSetpoint = 0; ySetpoint = 0; thSetpoint = 0;
  xOutput = 0; yOutput = 0; thOutput = 0;
  pidX.SetMode(MANUAL); pidY.SetMode(MANUAL); pidTh.SetMode(MANUAL);
  pidX.SetMode(AUTOMATIC); pidY.SetMode(AUTOMATIC); pidTh.SetMode(AUTOMATIC);
  Serial.print("ZEROED "); Serial.println(bootId);
}

void stopAtCurrentState() {
  target_x = x; target_y = y; target_theta = theta;
  grip_target = grip_pos; hand_target = hand_pos; wrist_target = wrist_pos;
  elbow_target = elbow_pos; shoulder_target = shoulder_pos;
  la_target = la_pos; frame_target = frame_pos;
  commandCount = 0; currentCommandIndex = 0; settledSince = 0; streamActive = false;
}

float pwmToAngle(const JointConfig& joint, float pwm) {
  return joint.angle_min + (pwm - joint.pwm_min) *
      (joint.angle_max - joint.angle_min) / (joint.pwm_max - joint.pwm_min);
}

void printRobotState() {
  Serial.print("STATE4 "); Serial.print(activeCommandId); Serial.print(" ");
  Serial.print(linearCountsToCm(x), 3); Serial.print(" ");
  Serial.print(linearCountsToCm(y), 3); Serial.print(" ");
  Serial.print(thetaCountsToDeg(theta), 3); Serial.print(" ");
  Serial.print(pwmToAngle(GRIP, grip_pos), 3); Serial.print(" ");
  Serial.print(pwmToAngle(HAND, hand_pos), 3); Serial.print(" ");
  Serial.print(pwmToAngle(WRIST, wrist_pos), 3); Serial.print(" ");
  Serial.print(elbow_pos, 3); Serial.print(" "); Serial.println(shoulder_pos, 3);
  if (streamActive) {
    // Consumed points except the interpolation anchor can be reclaimed.
    int retained = commandCount - max(0, currentCommandIndex - 1);
    Serial.print("BUF3 "); Serial.print(activeCommandId); Serial.print(" ");
    Serial.print(streamSequence); Serial.print(" "); Serial.println(MAX_COMMANDS - retained);
  }
}

// STR id crc sequence final timed-points. CRC covers sequence/final as well
// as positions. Appends retain absolute trajectory time and PID state.
void parseStreamPacket(char* input) {
  char* end;
  unsigned long id = strtoul(input + 4, &end, 10);
  if (end == input + 4 || *end != ' ' || id == 0) {
    stopAtCurrentState(); protocolError("BAD_STREAM_ID"); return;
  }
  if (commandCount > 0 && (!streamActive || id != activeCommandId)) {
    Serial.println("BUSY2"); return;
  }
  bool first = commandCount == 0;
  if (first) activeCommandId = id;
  char* crcStart = end + 1;
  unsigned long expected = strtoul(crcStart, &end, 16);
  if (end == crcStart || *end != ' ' || expected > 65535) {
    stopAtCurrentState(); protocolError("BAD_CRC_HEADER"); return;
  }
  char* body = end + 1;
  uint16_t crc = 0xffff;
  for (const char* c = body; *c; ++c) {
    crc ^= (uint16_t)(uint8_t)*c << 8;
    for (int bit=0; bit<8; ++bit)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
  }
  if (crc != expected) { stopAtCurrentState(); protocolError("CRC_MISMATCH"); return; }
  unsigned long seq = strtoul(body, &end, 10);
  if (end == body || *end != ' ' || seq != (first ? 0 : streamSequence) || (!first && streamFinal)) {
    stopAtCurrentState(); protocolError("STREAM_SEQUENCE"); return;
  }
  char* flag = end + 1;
  if ((flag[0] < '0' || flag[0] > '3') || flag[1] != ' ') {
    stopAtCurrentState(); protocolError("STREAM_FINAL"); return;
  }
  bool final = ((flag[0] - '0') & 1) != 0;
  bool transit = ((flag[0] - '0') & 2) != 0;
  if (!first && transit != streamTransit) {
    stopAtCurrentState(); protocolError("STREAM_PROFILE"); return;
  }
  int discard = first ? 0 : max(0, currentCommandIndex - 1);
  if (first) { commandCount = 0; currentCommandIndex = 0; }
  for (int i=discard; i<commandCount; ++i) commandQueue[i-discard] = commandQueue[i];
  commandCount -= discard; currentCommandIndex -= discard;
  int count = commandCount;
  char* context = NULL;
  char* point = strtok_r(flag + 2, ",", &context);
  bool any = false;
  while (point != NULL) {
    if (count >= MAX_COMMANDS) { stopAtCurrentState(); protocolError("BUFFER_FULL"); return; }
    char* valuesContext = NULL;
    char* token = strtok_r(point, " \t", &valuesContext);
    int n = 0;
    while (token != NULL && n < COMMAND_VALUE_COUNT) {
      char* endValue;
      float v = strtod(token, &endValue);
      if (*endValue || !isfinite(v)) { stopAtCurrentState(); protocolError("BAD_NUMBER"); return; }
      commandQueue[count].value[n++] = v;
      token = strtok_r(NULL, " \t", &valuesContext);
    }
    if (n != COMMAND_VALUE_COUNT || token != NULL) {
      stopAtCurrentState(); protocolError("INCORRECT_INPUT_COUNT"); return;
    }
    float* v = commandQueue[count].value;
    if (fabs(v[0]) > 1000 || fabs(v[1]) > 1000 || v[3] < 0 || v[3] > 90 ||
        v[4] < -70 || v[4] > 80 || v[5] < -90 || v[5] > 90 ||
        v[6] < -35 || v[6] > 140 || v[7] < -30 || v[7] > 150 || v[8] != 0 || v[9] != 0) {
      stopAtCurrentState(); protocolError("JOINT_LIMIT"); return;
    }
    if (v[10] < 0 || v[10] > 120000 || v[10] != floor(v[10]) ||
        (count == 0 && v[10] != 0) || (count > 0 && v[10] <= commandQueue[count-1].value[10])) {
      stopAtCurrentState(); protocolError("BAD_TIMING"); return;
    }
    count++; any = true;
    point = strtok_r(NULL, ",", &context);
  }
  if (!any || (first && !final && count < 2)) {
    stopAtCurrentState(); protocolError("EMPTY_STREAM"); return;
  }
  commandCount = count; streamActive = true; streamFinal = final; streamTransit = transit; streamSequence = seq + 1;
  lastHostMillis = current_millis;
  if (first) {
    parseMessage(commandQueue[0]); currentCommandIndex = 1;
    trajectoryStart = current_millis; settledSince = 0;
  }
  Serial.print("ACK "); Serial.print(activeCommandId); Serial.println(" BUFFERED");
}

void parseCommandQueue(char* input) {
  streamActive = false;
  commandCount = 0; currentCommandIndex = 0; settledSince = 0;
  if (strncmp(input, "CMD ", 4) != 0) { protocolError("PROTOCOL_V2_REQUIRED"); return; }
  char* endId;
  unsigned long id = strtoul(input + 4, &endId, 10);
  if (endId == input + 4 || *endId != ' ' || id == 0) { protocolError("BAD_ID"); return; }
  activeCommandId = id;
  char* endCrc;
  unsigned long expectedCrc = strtoul(endId + 1, &endCrc, 16);
  if (endCrc == endId + 1 || *endCrc != ' ' || expectedCrc > 65535) {
    protocolError("BAD_CRC_HEADER"); return;
  }
  uint16_t crc = 0xffff;
  for (const char* c = endCrc + 1; *c; ++c) {
    crc ^= (uint16_t)(uint8_t)*c << 8;
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
  }
  if (crc != expectedCrc) { protocolError("CRC_MISMATCH"); return; }
  char* queueContext = NULL;
  char* commandText = strtok_r(endCrc + 1, ",", &queueContext);
  while (commandText != NULL) {
    if (commandCount >= MAX_COMMANDS) {
      commandCount = 0; protocolError("QUEUE_TOO_LONG"); return;
    }
    char* tokenContext = NULL;
    char* token = strtok_r(commandText, " \t", &tokenContext);
    int tokenCount = 0;
    while (token != NULL && tokenCount < COMMAND_VALUE_COUNT) {
      char* endValue;
      float value = strtod(token, &endValue);
      if (*endValue != '\0' || !isfinite(value)) {
        commandCount = 0; protocolError("BAD_NUMBER"); return;
      }
      commandQueue[commandCount].value[tokenCount++] = value;
      token = strtok_r(NULL, " \t", &tokenContext);
    }
    if (tokenCount != COMMAND_VALUE_COUNT || token != NULL) {
      commandCount = 0; protocolError("INCORRECT_INPUT_COUNT"); return;
    }
    float* v = commandQueue[commandCount].value;
    if (fabs(v[0]) > 1000 || fabs(v[1]) > 1000 || v[3] < 0 || v[3] > 90 ||
        v[4] < -70 || v[4] > 80 || v[5] < -90 || v[5] > 90 ||
        v[6] < -35 || v[6] > 140 || v[7] < -30 || v[7] > 150 || v[8] != 0 || v[9] != 0) {
      commandCount = 0; protocolError("JOINT_LIMIT"); return;
    }
    float t = commandQueue[commandCount].value[10];
    if (t < 0 || t > 120000 || (commandCount == 0 && t != 0) ||
        (commandCount > 0 && t <= commandQueue[commandCount - 1].value[10])) {
      commandCount = 0; protocolError("BAD_TIMING"); return;
    }
    commandCount++;
    commandText = strtok_r(NULL, ",", &queueContext);
  }
  if (commandCount > 0) {
    Serial.print("ACK "); Serial.print(activeCommandId); Serial.println(" QUEUED");
    // Preserve learned holding correction across settled trajectory chunks.
    parseMessage(commandQueue[0]); currentCommandIndex = 1;
    trajectoryStart = current_millis; command_start_millis = current_millis;
  } else protocolError("EMPTY_QUEUE");
}

float sgn(int32_t value) {
  return (value > 0) - (value < 0);
}

float moveToward(float current, float target, float maxStep) {
  float delta = target - current;
  if (fabs(delta) <= maxStep) return target;
  return current + ((delta > 0) ? maxStep : -maxStep);
}

void drive(float xcmd, float ycmd, float thetacmd) {
  float nw = (ycmd - xcmd + thetacmd);
  float ne = (ycmd + xcmd + thetacmd);
  float sw = (ycmd + xcmd - thetacmd);
  float se = (ycmd - xcmd - thetacmd);

  float maximum = max(abs(nw), max(abs(ne), max(abs(sw), abs(se))));
  if (maximum > 1.0) {
    nw /= maximum;
    ne /= maximum;
    sw /= maximum;
    se /= maximum;
  }

  setMotorPower(NW_PWM, NW_CW, NW_CCW, nw);
  setMotorPower(NE_PWM, NE_CW, NE_CCW, ne);
  setMotorPower(SW_PWM, SW_CCW, SW_CW, sw);
  setMotorPower(SE_PWM, SE_CCW, SE_CW, se);
}

void setMotorPower(int pwm, int cw, int ccw, float percent) {
  bool dir = (percent > 0);
  digitalWrite(cw, dir);
  digitalWrite(ccw, !dir);

  float output = constrain(fabs(percent), 0.0, 1.0);
  if (output <= 0.0001) {
    analogWrite(pwm, 0);
    return;
  }

  float compensatedOutput =
    MIN_DRIVE_COMMAND + (1.0 - MIN_DRIVE_COMMAND) * output;
  analogWrite(pwm, (int)(200 * compensatedOutput));
}

void controlLA(int direction) {
  bool dir = (direction > 0);
  digitalWrite(LA_UP, dir);
  digitalWrite(LA_DOWN, !dir);
  bool enabled = (direction != 0);
  digitalWrite(LA_PWM, enabled);
}

void controlFrame(int direction) {
  bool dir = (direction > 0);
  digitalWrite(FRAME_UP, dir);
  digitalWrite(FRAME_DOWN, !dir);
  bool enabled = (direction != 0);
  digitalWrite(FRAME_PWM, enabled);
}
