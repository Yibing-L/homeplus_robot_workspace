#include <EEPROM.h>
#include <Servo.h>
#include <Encoder.h>
#include <math.h>
#include <PID_v1.h>

// --- Servo Mapping Setup ---
struct JointConfig {
  float angle_min, angle_max;
  float pwm_min, pwm_max;
};

const JointConfig GRIP = { 0, 90, 750, 1900 };
const JointConfig HAND = { -70, 80, 920, 1090 };
const JointConfig WRIST = { -90, 90, 1950, 1150 };
const JointConfig ELBOW = { -35, 145, 600, 2375 };
const JointConfig SHOULDER = { -30, 150, 595, 2360 };

float servoAngleToPWM(const JointConfig& j, float angle) {
  return j.pwm_min + (angle - j.angle_min) * (j.pwm_max - j.pwm_min) / (j.angle_max - j.angle_min);
}


unsigned long current_millis = 0;
unsigned long prev_millis = 0;
unsigned long command_start_millis = 0;
unsigned long command_timeout = 3000;  // 3000ms max wait per waypoint

float vx, vy, omega;
float target_x, target_y, target_theta;
float x, y, theta;
float dx, dy, dtheta;

int32_t prev_nw = 0, prev_ne = 0, prev_sw = 0, prev_se = 0;
int drive_threshold = 10;
float servo_threshold = 5.0;
float la_threshold = 1.0;

// --- PID GAINS ---
// X
// double Kp_x = 0.0015;  // was 0.0011
// double Ki_x = 0.000; // was 0.00010
// double Kd_x = 0.000;  // was 0.0002

double Kp_x = 0.0015;   // was 0.0011
double Ki_x = 0.00001;  // was 0.00010
double Kd_x = 0.0001;   // was 0.0002

double Kp_y = 0.0015;
double Ki_y = 0.00001;
double Kd_y = 0.0001;
double Kp_th = 0.0045, Ki_th = 0.00001, Kd_th = 0.0005;

// --- PID variables ---
double xInput = 0, xOutput = 0, xSetpoint = 0;
double yInput = 0, yOutput = 0, ySetpoint = 0;
double thInput = 0, thOutput = 0, thSetpoint = 0;

PID pidX(&xInput, &xOutput, &xSetpoint, Kp_x, Ki_x, Kd_x, DIRECT);
PID pidY(&yInput, &yOutput, &ySetpoint, Kp_y, Ki_y, Kd_y, DIRECT);
PID pidTh(&thInput, &thOutput, &thSetpoint, Kp_th, Ki_th, Kd_th, DIRECT);

float wheel_diameter = 0.092;
float base_radius = 0.34;
float wheel_circumference = PI * wheel_diameter;
float resolution = 3100;

#define SERIAL_BUF_SIZE 2048
char serialBuffer[SERIAL_BUF_SIZE];
int serialBufIdx = 0;
const int MAX_COMMANDS = 100;
String commandQueue[MAX_COMMANDS];
int commandCount = 0;
int currentCommandIndex = 0;

// === Servo Setup ===
Servo grip, hand, wrist, elbow, shoulder;
float grip_target, hand_target, wrist_target, elbow_target, shoulder_target;
float grip_pos, hand_pos, wrist_pos, elbow_pos, shoulder_pos;
float dgrip, dhand, dwrist, delbow, dshoulder;

// === Drive Setup ===
#define NW_PWM 6
#define NW_CW 49
#define NW_CCW 48
Encoder NW_ENCODER(21, 23);

#define NE_PWM 5
#define NE_CW 50
#define NE_CCW 51
Encoder NE_ENCODER(20, 22);

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
#define LA_PWM 41  // 12mm/s, max 435

#define FRAME_UP 42
#define FRAME_DOWN 43
#define FRAME_PWM 40  // 12mm/s

#define RELAY_PIN 8

// 如果你的 relay 是高电平触发，用这个
#define RELAY_ON HIGH
#define RELAY_OFF LOW

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF);  // 一开机先保持关闭

  Serial.begin(9600);

  float init_grip = servoAngleToPWM(GRIP, 0);
  float init_hand = servoAngleToPWM(HAND, 0);
  float init_wrist = servoAngleToPWM(WRIST, 0);
  float init_elbow = servoAngleToPWM(ELBOW, 0);
  float init_shoulder = servoAngleToPWM(SHOULDER, 0);


  grip_pos = init_grip;
  hand_pos = init_hand;
  wrist_pos = init_wrist;
  elbow_pos = init_elbow;
  shoulder_pos = init_shoulder;

  grip_target = init_grip;
  hand_target = init_hand;
  wrist_target = init_wrist;
  elbow_target = init_elbow;
  shoulder_target = init_shoulder;

  grip.attach(9);
  grip.writeMicroseconds((int)init_grip);

  hand.attach(13);
  hand.writeMicroseconds((int)init_hand);

  wrist.attach(12);
  wrist.writeMicroseconds((int)init_wrist);

  elbow.attach(11);
  elbow.writeMicroseconds((int)init_elbow);

  shoulder.attach(10);
  shoulder.writeMicroseconds((int)init_shoulder);

  la_target = 0;
  la_pos = 0;
  frame_target = 0;
  frame_pos = 0;

  // --- Configure PID controllers ---
  pidX.SetSampleTime(5);
  pidY.SetSampleTime(5);
  pidTh.SetSampleTime(5);

  // Output is normalized velocity command in [-1, 1]
  pidX.SetOutputLimits(-1.0, 1.0);
  pidY.SetOutputLimits(-1.0, 1.0);
  pidTh.SetOutputLimits(-1.0, 1.0);

  pidX.SetMode(AUTOMATIC);
  pidY.SetMode(AUTOMATIC);
  pidTh.SetMode(AUTOMATIC);

  prev_nw = NW_ENCODER.read();
  prev_ne = NE_ENCODER.read();
  prev_sw = SW_ENCODER.read();
  prev_se = SE_ENCODER.read();

  delay(1000);                        // 等1秒，给系统初始化时间
  digitalWrite(RELAY_PIN, RELAY_ON);  // 最后再打开relay

  Serial.println("USB Serial Ready");
}

void loop() {
  prev_millis = current_millis;
  current_millis = millis();

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      serialBuffer[serialBufIdx] = '\0';
      if (serialBufIdx > 0) {
        parseCommandQueue(String(serialBuffer));
      }
      serialBufIdx = 0;
    } else if (c != '\r') {
      if (serialBufIdx < SERIAL_BUF_SIZE - 1) {
        serialBuffer[serialBufIdx++] = c;
      }
    }
  }

  update_pos();  // updates x,y,theta and dx,dy,dtheta

  bool timed_out = (current_millis - command_start_millis) > command_timeout;
  if (status() || timed_out) {
    if (currentCommandIndex < commandCount) {
      if (timed_out) Serial.println("timeout - advancing");
      else Serial.println("proceed");
      parseMessage(commandQueue[currentCommandIndex]);
      currentCommandIndex++;
      command_start_millis = current_millis;
    } else if (commandCount > 0 && currentCommandIndex >= commandCount) {
      // All commands in queue finished
      Serial.println("DONE");
      commandCount = 0;
      currentCommandIndex = 0;
    }
  }

  // --- Feed PID with measurements and setpoints ---
  xInput = (double)x;
  yInput = (double)y;
  thInput = (double)theta;

  xSetpoint = (double)target_x;
  ySetpoint = (double)target_y;
  thSetpoint = (double)target_theta;

  // Compute PID outputs
  pidX.Compute();
  pidY.Compute();
  pidTh.Compute();

  // Optional deadband to stop dithering near goal
  if (abs(dx) < drive_threshold) xOutput = 0.0;
  if (abs(dy) < drive_threshold) yOutput = 0.0;
  if (abs(dtheta) < drive_threshold) thOutput = 0.0;

  // PID outputs are world-frame commands, but drive() wants body-frame commands
  vx = (float)xOutput;
  vy = (float)yOutput;
  omega = (float)thOutput;

  float heading_rad = (theta / resolution) * wheel_circumference / base_radius;
  float vx_body = cos(heading_rad) * vx + sin(heading_rad) * vy;
  float vy_body = -sin(heading_rad) * vx + cos(heading_rad) * vy;

  drive(vx_body, vy_body, omega);

  grip.writeMicroseconds((int)grip_target);
  hand.writeMicroseconds((int)hand_target);
  wrist.writeMicroseconds((int)wrist_target);
  elbow.writeMicroseconds((int)elbow_target);
  shoulder.writeMicroseconds((int)shoulder_target);

  controlLA(vla);
  controlFrame(vframe);
}

bool status() {
  return ((abs(dx) < drive_threshold) && (abs(dy) < drive_threshold) && (abs(dtheta) < drive_threshold) && (abs(dgrip) < servo_threshold) && (abs(dhand) < servo_threshold) && (abs(dwrist) < servo_threshold) && (abs(delbow) < servo_threshold) && (abs(dshoulder) < servo_threshold) && (abs(dla) < la_threshold) && (abs(dframe) < la_threshold));
}

void update_pos() {
  float dt = current_millis - prev_millis;

  // === Drive encoders ===
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

  dx = target_x - x;
  dy = target_y - y;
  dtheta = target_theta - theta;

  // === Servos (rate-limited open-loop) ===
  dgrip = grip_target - grip_pos;
  dhand = hand_target - hand_pos;
  dwrist = wrist_target - wrist_pos;
  delbow = elbow_target - elbow_pos;
  dshoulder = shoulder_target - shoulder_pos;

  if (abs(dgrip) > servo_threshold) { grip_pos += (sgn(dgrip) * dt * 0.2); }
  if (abs(dhand) > servo_threshold) { hand_pos += (sgn(dhand) * dt * 0.2); }
  if (abs(dwrist) > servo_threshold) { wrist_pos += (sgn(dwrist) * dt * 0.2); }
  if (abs(delbow) > servo_threshold) { elbow_pos += (sgn(delbow) * dt * 0.2); }
  if (abs(dshoulder) > servo_threshold) { shoulder_pos += (sgn(dshoulder) * dt * 0.7 * 0.2); }

  // === Linear Actuators (bang-bang) ===
  dla = la_target - la_pos;
  dframe = frame_target - frame_pos;

  vla = (abs(dla) > la_threshold) ? sgn(dla) : 0;
  vframe = (abs(dframe) > la_threshold) ? sgn(dframe) : 0;

  if (abs(dla) > la_threshold) { la_pos += (sgn(dla) * dt * 0.00871); }
  if (abs(dframe) > la_threshold) { frame_pos += (sgn(dframe) * dt * 0.0091455); }
}

int splitString(String input, char delimiter, String* tokens, int maxTokens) {
  int tokenCount = 0;
  int start = 0;
  int end = input.indexOf(delimiter);

  while (end != -1 && tokenCount < maxTokens) {
    tokens[tokenCount++] = input.substring(start, end);
    start = end + 1;
    end = input.indexOf(delimiter, start);
  }
  if (tokenCount < maxTokens) {
    tokens[tokenCount++] = input.substring(start);
  }
  return tokenCount;
}

void parseMessage(String input) {
  Serial.println(input);

  const int maxTokens = 10;
  String tokens[maxTokens];
  int tokenCount = splitString(input, ' ', tokens, maxTokens);

  if (tokenCount != 10) {
    Serial.println("Error: Incorrect number of inputs.");
    return;
  }

  float x_ = 0.01 * tokens[0].toFloat();
  float y_ = 0.01 * tokens[1].toFloat();
  float theta_ = tokens[2].toFloat();
  float grip_ = tokens[3].toFloat();
  float hand_ = tokens[4].toFloat();
  float wrist_ = tokens[5].toFloat();
  float elbow_ = tokens[6].toFloat();
  float shoulder_ = tokens[7].toFloat();
  float la_ = tokens[8].toFloat();
  float frame_ = tokens[9].toFloat();

  // convert meters / degrees to encoder pulses
  target_x = (x_ / wheel_circumference) * resolution;
  target_y = (y_ / wheel_circumference) * resolution;
  target_theta = (((PI / 180) * base_radius * theta_) / wheel_circumference) * resolution;

  grip_target = servoAngleToPWM(GRIP, grip_);
  hand_target = servoAngleToPWM(HAND, hand_);
  wrist_target = servoAngleToPWM(WRIST, wrist_);
  elbow_target = servoAngleToPWM(ELBOW, elbow_);
  shoulder_target = servoAngleToPWM(SHOULDER, shoulder_);

  la_target = constrain(la_, 0, 435);
  frame_target = constrain(frame_, 0, 150);
}

void parseCommandQueue(String input) {
  Serial.println(input);
  commandCount = 0;
  currentCommandIndex = 0;
  int startIdx = 0;

  while (startIdx < input.length() && commandCount < MAX_COMMANDS) {
    int endIdx = input.indexOf(',', startIdx);
    if (endIdx == -1) endIdx = input.length();

    String command = input.substring(startIdx, endIdx);
    command.trim();

    if (command.length() > 0) {
      commandQueue[commandCount++] = command;
    }
    startIdx = endIdx + 1;
  }

  if (commandCount > 0) {
    parseMessage(commandQueue[currentCommandIndex]);
    currentCommandIndex++;
    command_start_millis = millis();
  }
}

float sgn(int32_t x) {
  return (x > 0) - (x < 0);  // 1 for positive, -1 for negative, and 0 for 0
}

void drive(float xcmd, float ycmd, float thetacmd) {
  float nw = (ycmd - xcmd + thetacmd);
  float ne = (ycmd + xcmd + thetacmd);
  float sw = (ycmd + xcmd - thetacmd);
  float se = (ycmd - xcmd - thetacmd);

  float m = max(abs(nw), max(abs(ne), max(abs(sw), abs(se))));
  if (m > 1.0) {
    nw /= m;
    ne /= m;
    sw /= m;
    se /= m;
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

  float u = fabs(percent);
  if (u < 0.35) {  // <15% power won’t move the motor
    analogWrite(pwm, 0);
    return;
  }
  analogWrite(pwm, (int)(200 * constrain(u, 0, 1)));
}


void controlLA(int direction) {
  bool dir = (direction > 0);
  digitalWrite(LA_UP, dir);
  digitalWrite(LA_DOWN, !dir);
  bool en = (direction != 0);
  digitalWrite(LA_PWM, en);
}

void controlFrame(int direction) {
  bool dir = (direction > 0);
  digitalWrite(FRAME_UP, dir);
  digitalWrite(FRAME_DOWN, !dir);
  bool en = (direction != 0);
  digitalWrite(FRAME_PWM, en);
}
