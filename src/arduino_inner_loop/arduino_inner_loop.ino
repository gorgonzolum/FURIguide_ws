// Arduino Inner Loop Control Code
// Based on Enhanced Thunder Tumbler 2 Code
// from "Modeling and Control of a Longitudinal Platoon" (Zhichao Li)
// enhanced with ROS integration
// used as an interim to accelerate hardware prototyping

// ROS header must be included first
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <Adafruit_BNO055.h>
#include <Adafruit_MotorShield.h>
#include <Encoder.h>
#include <math.h>

Adafruit_BNO055 imu_bno055 = Adafruit_BNO055();
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *motorL = AFMS.getMotor(3);
Adafruit_DCMotor *motorR = AFMS.getMotor(2);

#define DEBUG
// Control
int PWM_D2A_FACTOR = 49;
int CTRL_LOOP_PERIOD = 100;

// Physical Constraints
float axle_length = 0.15;
float wheel_radius = 0.05;

// Setup and Macros
#ifdef DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
#else
  #define DEBUG_PRINT(x)
#endif

void setup() {
  Serial.begin(9600);

  setupIMU();
  Serial.print("IMU Setup Complete");
  setupMotors();
  Serial.print("Motor Setup Complete");
  //setupROS();
  //Serial.print("ROS Initialized");

  delay(1000);
}

void setupMotors() {
  AFMS.begin();

  motorL->setSpeed(0);
  motorR->setSpeed(0);
}

// --- Encoder ---
#define ENC_LEFT_PIN_A 2
#define ENC_LEFT_PIN_B 2 
#define ENC_RIGHT_PIN_A 3
#define ENC_RIGHT_PIN_B 3 
#define ENCODER_CPT_GEARED 80

Encoder EncL(ENC_LEFT_PIN_A, ENC_LEFT_PIN_B);
Encoder EncR(ENC_RIGHT_PIN_A, ENC_RIGHT_PIN_B);

long encoder_left_read() {
  return EncL.read();
}

long encoder_right_read() {
  return EncR.read();
}

void encoder_left_write(long val) {
  return EncL.write(val);
}
void encoder_right_write(long val ) {
  return EncR.write(val);
}
void encoder_reset() {
  encoder_left_write(0);
  encoder_right_write(0);
}

float encoder_calculate_angular_speed(long delta_tick, long delta_time) {
  return 1.0 * (delta_tick) / (delta_time / 1000.0) * 2 * M_PI / ENCODER_CPT_GEARED;
}

// --- IMU ---
imu::Vector<3> euler_init;

imu::Vector<3> euler;
imu::Vector<3> acc;
imu::Vector<3> gyro;

void setupIMU() {
  imu_bno055.begin();
}

void imu_read() {
  euler = imu_bno055.getVector(Adafruit_BNO055::VECTOR_EULER);
  acc = imu_bno055.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  gyro = imu_bno055.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
}

float imu_get_theta() {
  return -(euler.x() - euler_init.x()) * M_PI / 180.0;
}

float imu_get_accy() {
  return -acc.y();
}

float imu_get_omega() {
  return gyro.z();
}

// --- Control Code ---
// Control Loop Period
volatile long ctrl_loop_period = CTRL_LOOP_PERIOD;

// PID Control Paramters
float prefilter_co = 0.167;

float kp_left = 0.25;
float ki_left = 0.1;
float kd_left = 0.0;

float kp_right = 0.25;
float ki_right = 0.1;
float kd_right = 0.0;

float roll_off_co = 0.8; // 40 / (s+40)

// Robot State
float imu_theta = 0;
float imu_accy = 0;
float imu_omega = 0;

float wl = 0;
float wr = 0;

float wl_p = 0;
float wr_p = 0;

float wl_dsr = 0;
float wr_dsr = 0;

float err_wl = 0;
float err_wr = 0;
float err_wl_p = 0;
float err_wr_p = 0;
float err_wl_pp = 0;
float err_wr_pp = 0;

float wl_dsr_filtered = 0;
float wr_dsr_filtered = 0;
float wl_dsr_filtered_p = 0;
float wr_dsr_filtered_p = 0;

int pwml = 0;
int pwmr = 0;
int pwml_out = 0;
int pwmr_out = 0;
int pwml_out_p = 0;
int pwmr_out_p = 0;

float pwml_up;
float pwml_ui;
float pwml_ud;
float pwmr_up;
float pwmr_ui;
float pwmr_ud;

void ctrl_get_theta_accx_omega() {
  imu_read();
  imu_theta = imu_get_theta();
  imu_accy = imu_get_accy();
  imu_omega = imu_get_omega();

  //DEBUG_PRINT("IMU Theta: "); DEBUG_PRINT(imu_theta); DEBUG_PRINT("\r\n");
}

void ctrl_get_current_wl_wr() {
  long left = encoder_left_read();
  long right = encoder_right_read();
  
  wl = encoder_calculate_angular_speed(left, ctrl_loop_period);
  wr = encoder_calculate_angular_speed(right, ctrl_loop_period);

  // Average filter
  wl = (wl + wl_p) / 2;
  wr = (wr + wr_p) / 2;

  wl_p = wl;
  wr_p = wr;
  
  DEBUG_PRINT("wl, wr: "); DEBUG_PRINT(wl); DEBUG_PRINT(", "); DEBUG_PRINT(wr);
  DEBUG_PRINT("\r\n");

  // Reset to record delta
  encoder_reset();
}

float ctrl_pid(float err, float err_sum, float err_p,
               float kp, float ki, float kd,
               float *up_out, float *ui_out, float *ud_out,
               float ts) {
  float u = 0;
  float up = kp * err;
  float ui = ki * ts * err_sum;
  float ud = kd * (err - err_p) / ts;

  u = up + ui + ud;

  if (up_out != NULL)
    *up_out = up;
  if (ui_out != NULL)
    *ui_out = up;
  if (ud_out != NULL)
    *ud_out = up;

  return u;
}

float ctrl_pid_inc(int u_p, 
                    float err, float err_p, float err_pp,
                    float kp, float ki, float kd,
                    float *up_out, float *ui_out, float *ud_out,
                    float ts) {
  float up = kp * (err - err_p);
  float ui = ki * ts * err;
  float ud = kd * ((err -err_p) - (err_p - err_pp)) / ts;
  float ud_temp = ud;
  
  float delta_u = (up + ui + ud) * PWM_D2A_FACTOR;
  int u = u_p + (int)delta_u;
  
  if (up_out != NULL)
    *up_out = up;
  if (ui_out != NULL)
    *ui_out = up;
  if (ud_out != NULL)
    *ud_out = up;

  return u;
}

// FIXME: do this on ROS message
void ctrl_update_wd(double vd, double wd) {
  wl_dsr = (2*vd + axle_length*wd) / (2*wheel_radius);
  wr_dsr = (2*vd - axle_length*wd) / (2*wheel_radius);

  DEBUG_PRINT("wldsr, wrdsr: "); DEBUG_PRINT(wl_dsr); DEBUG_PRINT(", "); DEBUG_PRINT(wr_dsr);
  DEBUG_PRINT("\r\n");
}

float ctrl_output_rolloff (int u_rf_p, int u_in) {
  float u_rf = (1 - roll_off_co) * u_rf_p + roll_off_co * u_in;
  return u_rf;
}

void ctrl_inner_loop() {
  int stop_action_level = 0;

  if (wl_dsr <= 0.02 || wr_dsr <= 0.02) {
    if (wl_dsr < 0 || wr_dsr < 0) {
      stop_action_level = 2;
    } else {
      stop_action_level = 1;
    }

    wl_dsr = 0;
    wr_dsr = 0;
  }

  wl_dsr_filtered = (1 - prefilter_co) * wl_dsr_filtered_p + prefilter_co * wl_dsr;
  wr_dsr_filtered = (1 - prefilter_co) * wr_dsr_filtered_p + prefilter_co * wr_dsr;

  // Error between measured and desired omega
  err_wl = wl_dsr_filtered - wl;
  err_wr = wr_dsr_filtered - wr;

  DEBUG_PRINT("err_wl, err_wr: "); DEBUG_PRINT(err_wl); DEBUG_PRINT(", "); DEBUG_PRINT(err_wr);
  DEBUG_PRINT("\r\n");

  // PID Inner Loop Controllre
  pwml_out = ctrl_pid_inc(pwml_out_p, 
                          err_wl, err_wl_p, err_wl_pp,
                          kp_left, ki_left, kd_left,
                          &pwml_up, &pwml_ui, &pwml_ud,
                          (float)ctrl_loop_period / 1000.0); 

  pwmr_out = ctrl_pid_inc(pwmr_out_p, 
                          err_wr, err_wr_p, err_wr_pp,
                          kp_right, ki_right, kd_right,
                          &pwmr_up, &pwmr_ui, &pwmr_ud,
                          (float)ctrl_loop_period / 1000.0);

  //pwml_out = pwml_out_p + (int)(err_wl * PWM_D2A_FACTOR * 0.2);
  //pwmr_out = pwmr_out_p + (int)(err_wr * PWM_D2A_FACTOR * 0.2);
  DEBUG_PRINT("pwml_out, pwmr_out: "); DEBUG_PRINT(pwml_out); DEBUG_PRINT(", "); DEBUG_PRINT(pwmr_out);
  DEBUG_PRINT("\r\n");

  pwml = (int) ctrl_output_rolloff(pwml_out_p, pwml_out);
  pwml = constrain(pwml, -30, 225);
  pwmr = (int) ctrl_output_rolloff(pwmr_out_p, pwmr_out);
  pwmr = constrain(pwmr, -30, 225);

  if (stop_action_level > 0) {
    if (stop_action_level <= 1) {
      pwml = 0;
      pwmr = 0;
    } else {
      pwml = 0;
      pwmr = 0;
    }
  }
}

void ctrl_inner_loop_update() {
  // Iteration
  pwml_out_p = pwml_out;
  pwmr_out_p = pwmr_out;

  err_wl_p = err_wl;
  err_wr_p = err_wr;
  err_wl_pp = err_wl_p;
  err_wr_pp = err_wr_p;

  wl_dsr_filtered_p = wl_dsr_filtered;
  wr_dsr_filtered_p = wr_dsr_filtered;
}

void ctrl_loop_reset () {
  pwml_out_p = 0;
  pwmr_out_p = 0;

  err_wl_p = 0;
  err_wr_p = 0;
  err_wl_pp = 0;
  err_wr_pp = 0;

  wl_dsr_filtered_p = 0;
  wr_dsr_filtered_p = 0;
}

void update_motors() {
  DEBUG_PRINT("PWML: "); DEBUG_PRINT(pwml);
  DEBUG_PRINT(" PWMR: "); DEBUG_PRINT(pwmr);
  DEBUG_PRINT("\r\n");

  motorL->setSpeed(abs(pwml));
  if (pwml > 0)
    motorL->run(FORWARD);
  else
    motorL->run(BACKWARD);

  motorR->setSpeed(abs(pwmr));
  if (pwmr > 0)
    motorR->run(FORWARD);
  else
    motorR->run(BACKWARD);
}

// --- ROS ---
ros::NodeHandle arduino_nh;

// Pub/Sub Declarations
void cmd_velMsgRecv_cb(geometry_msgs::Twist &msg);
ros::Subscriber<geometry_msgs::Twist> velSub("cmd_vel", cmd_velMsgRecv_cb);

nav_msgs::Odometry odom_msg;
ros::Publisher odomPub("odom", &odom_msg);

// Setup
void setupROS() {
  arduino_nh.initNode();
  arduino_nh.subscribe(velSub);
  arduino_nh.advertise(odomPub);
}

// Pub/Sub Callbacks
void cmd_velMsgRecv_cb(geometry_msgs::Twist& msg) {
  ctrl_update_wd(msg.linear.x, msg.angular.z);
}

void publishOdom() {
}


unsigned long time = 0;
unsigned long time_p = 0;
unsigned long sample_time = ctrl_loop_period; // T = 100 msec
void loop() {
  // 10Hz Update Rate
  time = millis(); 
  if (time - time_p > sample_time) {
    time_p = time;
    publishOdom();
    arduino_nh.spinOnce();

    ctrl_get_current_wl_wr();
    ctrl_get_theta_accx_omega();

    if (time > 3000)
      ctrl_update_wd(0.5, 1.5);

    ctrl_inner_loop();
    update_motors();

    ctrl_inner_loop_update();
  }
}
