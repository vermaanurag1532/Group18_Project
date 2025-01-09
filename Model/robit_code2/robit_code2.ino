/*Motor controller using micro_ros serial set_microros_transports*/
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include <odometry.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <sensor_msgs/msg/imu.h>

sensor_msgs__msg__Imu imu_msg;

Adafruit_MPU6050 mpu;

//pin declaration
//Left wheel
int8_t R_FORW = 26;
int8_t R_BACK = 27;
int8_t R_enablePin = 25;
int8_t R_encoderPin1 = 12;  //Encoder Output of pin1 must connected with intreput pin of Esp32.
int8_t R_encoderPin2 = 13;
//right wheel
int8_t L_FORW = 33;
int8_t L_BACK = 32;
int8_t L_enablePin = 5;
int8_t L_encoderPin1 = 18;  //Encoder Output of pin1 must connected with intreput pin of Esp32.
int8_t L_encoderPin2 = 21;

//parameters of the robot
float wheels_y_distance_ = 0.55;
float wheel_radius = 0.0525;
float wheel_circumference_ = 2 * 3.14 * wheel_radius;
//encoder value per revolution of left wheel and right wheel
int tickPerRevolution_LW = 630;
int tickPerRevolution_RW = 630;
int threshold = 0;
//pid constants of left wheel
float kp_l = 1.35;
float ki_l = 0.0;
float kd_l = 0.15;
//pid constants of right wheel
float kp_r = 1.35;
float ki_r = 0.0;
float kd_r = 0.15;

//pwm parameters setup
const int freq = 30000;
const int pwmChannelL = 0;
const int pwmChannelR = 1;
const int resolution = 8;

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
std_msgs__msg__Int32 encodervalue_l;
std_msgs__msg__Int32 encodervalue_r;
nav_msgs__msg__Odometry odom_msg;
rcl_timer_t timer;
rcl_timer_t ControlTimer;
rcl_timer_t imuTimer;
unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
Odometry odometry;

//creating a class for motor control
class MotorController {
public:
  int8_t Forward;
  int8_t Backward;
  int8_t Enable;
  int8_t EncoderPinA;
  int8_t EncoderPinB;
  std_msgs__msg__Int32 EncoderCount;
  volatile long CurrentPosition;
  volatile long PreviousPosition;
  volatile long CurrentTime;
  volatile long PreviousTime;
  volatile long CurrentTimeforError;
  volatile long PreviousTimeForError;
  float rpmFilt;
  float eintegral;
  float ederivative;
  float rpmPrev;
  float kp;
  float ki;
  float kd;
  float error;
  float previousError = 0;
  int tick;

  MotorController(int8_t ForwardPin, int8_t BackwardPin, int8_t EnablePin, int8_t EncoderA, int8_t EncoderB, int tickPerRevolution) {
    this->Forward = ForwardPin;
    this->Backward = BackwardPin;
    this->Enable = EnablePin;
    this->EncoderPinA = EncoderA;
    this->EncoderPinB = EncoderB;
    this->tick = tickPerRevolution;
    pinMode(Forward, OUTPUT);
    pinMode(Backward, OUTPUT);
    pinMode(EnablePin, OUTPUT);
    pinMode(EncoderPinA, INPUT);
    pinMode(EncoderPinB, INPUT);
  }

  //initializing the parameters of PID controller
  void initPID(float proportionalGain, float integralGain, float derivativeGain) {
    kp = proportionalGain;
    ki = integralGain;
    kd = derivativeGain;
  }

  //function return rpm of the motor using the encoder tick values
  float getRpm() {
    CurrentPosition = EncoderCount.data;
    CurrentTime = millis();
    float delta1 = ((float)CurrentTime - PreviousTime) / 1.0e3;
    float velocity = ((float)CurrentPosition - PreviousPosition) / delta1;
    float rpm = (velocity / tick) * 60;
    rpmFilt = 0.854 * rpmFilt + 0.0728 * rpm + 0.0728 * rpmPrev;
    float rpmPrev = rpm;
    PreviousPosition = CurrentPosition;
    PreviousTime = CurrentTime;
    // Serial.println(rpmFilt);
    return rpmFilt;
  }

  //pid controller
  float pid(float setpoint, float feedback) {
    CurrentTimeforError = millis();
    float delta2 = ((float)CurrentTimeforError - PreviousTimeForError) / 1.0e3;
    error = setpoint - feedback;
    eintegral = eintegral + (error * delta2);
    ederivative = (error - previousError) / delta2;
    float control_signal = (kp * error) + (ki * eintegral) + (kd * ederivative);

    previousError = error;
    PreviousTimeForError = CurrentTimeforError;
    return control_signal;
  }
  //move the robot wheels based the control signal generated by the pid controller
  void moveBase(float ActuatingSignal, int threshold, int pwmChannel) {
    if (ActuatingSignal > 0) {
      digitalWrite(Forward, HIGH);
      digitalWrite(Backward, LOW);
    } else {
      digitalWrite(Forward, LOW);
      digitalWrite(Backward, HIGH);
    }
    int pwm = threshold + (int)fabs(ActuatingSignal);
    if (pwm > 255)
      pwm = 255;
    ledcWrite(pwmChannel, pwm);
  }
  void stop() {
    digitalWrite(Forward, LOW);
    digitalWrite(Backward, LOW);
  }

  // void plot(float Value1, float Value2){
  //     Serial.print("Value1:");
  //     Serial.print(Value1);
  //     Serial.print(",");
  //     Serial.print("value2:");
  //     Serial.println(Value2);
  // }
};

//creating objects for right wheel and left wheel
MotorController leftWheel(L_FORW, L_BACK, L_enablePin, L_encoderPin1, L_encoderPin2, tickPerRevolution_LW);
MotorController rightWheel(R_FORW, R_BACK, R_enablePin, R_encoderPin1, R_encoderPin2, tickPerRevolution_RW);

#define LED_PIN 2
#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}
void timer_callback(rcl_timer_t * imuTimer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (imuTimer != NULL) {
    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    struct timespec time_stamp = getTime();

    imu_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;
    imu_msg.header.frame_id=micro_ros_string_utilities_set(imu_msg.header.frame_id, "imu");

    imu_msg.linear_acceleration.x = a.acceleration.x;
    imu_msg.linear_acceleration.y = a.acceleration.y;
    imu_msg.linear_acceleration.z = a.acceleration.z;

    imu_msg.angular_velocity.x = g.gyro.x;
    imu_msg.angular_velocity.y = g.gyro.y;
    imu_msg.angular_velocity.z = g.gyro.z+0.03;
    imu_msg.orientation_covariance[0]=0.001;
    imu_msg.orientation_covariance[4]=0.001;
    imu_msg.orientation_covariance[8]=0.001;
    imu_msg.angular_velocity_covariance[0]=6.09e-7;
    imu_msg.angular_velocity_covariance[4]=6.09e-7;
    imu_msg.angular_velocity_covariance[8]=6.09e-7;
    imu_msg.linear_acceleration_covariance[0]=8e-9;
    imu_msg.linear_acceleration_covariance[4]=8e-9;
    imu_msg.linear_acceleration_covariance[8]=8e-9;

  }
}

//subscription callback function

void setup() {
  Wire.begin(16,4);
  Serial.begin(115200);

  //initializing the pid constants
  leftWheel.initPID(kp_l, ki_l, kd_l);
  rightWheel.initPID(kp_r, ki_r, kd_r);
  //initializing interrupt functions for counting the encoder tick values
  attachInterrupt(digitalPinToInterrupt(leftWheel.EncoderPinB), updateEncoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(rightWheel.EncoderPinA), updateEncoderR, RISING);
  //initializing pwm signal parameters
  ledcSetup(pwmChannelL, freq, resolution);
  ledcAttachPin(leftWheel.Enable, pwmChannelL);
  ledcSetup(pwmChannelR, freq, resolution);
  ledcAttachPin(rightWheel.Enable, pwmChannelR);

  set_microros_transports();

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));

  // create subscriber for cmd_vel topic
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));
  
  RCCHECK(rclc_publisher_init_default(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data"));
  //create a odometry publisher
  RCCHECK(rclc_publisher_init_default(
    &odom_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom/unfiltered"));

  //timer function for controlling the motor base. At every samplingT time
  //MotorControll_callback function is called
  //Here I had set SamplingT=10 Which means at every 10 milliseconds MotorControll_callback function is called
  const unsigned int timer_timeout = 50;
  RCCHECK(rclc_timer_init_default(
    &imuTimer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  
  const unsigned int samplingT = 50;
  RCCHECK(rclc_timer_init_default(
    &ControlTimer,
    &support,
    RCL_MS_TO_NS(samplingT),
    MotorControll_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  // RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_timer(&executor, &ControlTimer));
  RCCHECK(rclc_executor_add_timer(&executor, &imuTimer));

  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

void subscription_callback(const void* msgin) {
  prev_cmd_time = millis();
}

//function which controlles the motor
void MotorControll_callback(rcl_timer_t* timer, int64_t last_call_time) {
  float linearVelocity;
  float angularVelocity;
  //linear velocity and angular velocity send cmd_vel topic
  linearVelocity = msg.linear.x;
  angularVelocity = msg.angular.z;
  //linear and angular velocities are converted to leftwheel and rightwheel velocities
  float vL = (linearVelocity - (angularVelocity * 1 / 2)) * 20;
  float vR = (linearVelocity + (angularVelocity * 1 / 2)) * 20;
  //current wheel rpm is calculated
  float currentRpmL = leftWheel.getRpm();
  float currentRpmR = rightWheel.getRpm();
  //pid controlled is used for generating the pwm signal
  float actuating_signal_LW = leftWheel.pid(vL, currentRpmL);
  float actuating_signal_RW = rightWheel.pid(vR, currentRpmR);
  if (vL == 0 && vR == 0) {
    leftWheel.stop();
    rightWheel.stop();
    actuating_signal_LW = 0;
    actuating_signal_RW = 0;
  } else {
    rightWheel.moveBase(actuating_signal_RW, threshold, pwmChannelR);
    leftWheel.moveBase(actuating_signal_LW, threshold, pwmChannelL);
  }
  //odometry
  float average_rps_x = ((float)(currentRpmL + currentRpmR) / 2) / 60.0;  // RPM
  float linear_x = average_rps_x * wheel_circumference_;                  // m/s
  float average_rps_a = ((float)(-currentRpmL + currentRpmR) / 2) / 60.0;
  float angular_z = (average_rps_a * wheel_circumference_) / (wheels_y_distance_ / 2.0);  //  rad/s
  float linear_y = 0;
  unsigned long now = millis();
  float vel_dt = (now - prev_odom_update) / 1000.0;
  prev_odom_update = now;
  odometry.update(
    vel_dt,
    linear_x,
    linear_y,
    angular_z);
  publishData();
}

//interrupt function for left wheel encoder.
void updateEncoderL() {
  if (digitalRead(leftWheel.EncoderPinB) > digitalRead(leftWheel.EncoderPinA))
    leftWheel.EncoderCount.data++;
  else
    leftWheel.EncoderCount.data--;
  encodervalue_l = leftWheel.EncoderCount;
}

//interrupt function for right wheel encoder
void updateEncoderR() {
  if (digitalRead(rightWheel.EncoderPinA) > digitalRead(rightWheel.EncoderPinB))
    rightWheel.EncoderCount.data++;
  else
    rightWheel.EncoderCount.data--;
  encodervalue_r = rightWheel.EncoderCount;
}

//function which publishes wheel odometry.
void publishData() {
  odom_msg = odometry.getData();
  ;

  struct timespec time_stamp = getTime();

  odom_msg.header.stamp.sec = time_stamp.tv_sec;
  odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
}
void syncTime() {
  // get the current time from the agent
  unsigned long now = millis();
  RCCHECK(rmw_uros_sync_session(10));
  unsigned long long ros_time_ms = rmw_uros_epoch_millis();
  // now we can find the difference between ROS time and uC time
  time_offset = ros_time_ms - now;
}

struct timespec getTime() {
  struct timespec tp = { 0 };
  // add time difference between uC time and ROS time to
  // synchronize time with ROS
  unsigned long long now = millis() + time_offset;
  tp.tv_sec = now / 1000;
  tp.tv_nsec = (now % 1000) * 1000000;
  return tp;
}

