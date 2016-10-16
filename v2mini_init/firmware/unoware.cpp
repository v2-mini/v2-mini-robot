#include <Arduino.h>
#include <ros.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"

ros::NodeHandle nh;

const float pi = 3.1416;

const int max_speed = 1000;         // rps ...update val
const int wheel_radius = 4;         // cm
const int base_radius = 16;         // cm
const int accel_delay = 1;         // s/bit (~3.8s for full speed)
// NO CAN'T ACCEL LIKE THIS...ITS JUST BLOCKING...

// Front-Right base motor.
int motorFR_pwm = 3;
int motorFR_d1 = 13;
int motorFR_d2 = 4;

// Front-Left base motor.
int motorFL_pwm = 5;
int motorFL_d1 = 7;
int motorFL_d2 = 10;

// Back-Right base motor.
int motorBR_pwm = 6;
int motorBR_d1 = 10;
int motorBR_d2 = 10;

// Back-Left base motor.
int motorBL_pwm = 9;
int motorBL_d1 = 10;
int motorBL_d2 = 10;

// Only need to calculate motor speeds for FR & FL.
float motor_vectorFR[] = {-0.7071, 0.7071, 1};
float motor_vectorFL[] = {-0.7071, -0.7071, 1};

float motorFR_speed, motorFL_speed;
int velX, velY, velW, torso_height;
bool cwFR, cwFL;

void motion_cb(const geometry_msgs::Twist& motion_cmds) {
  velX = motion_cmds.linear.x;
  velY = motion_cmds.linear.y;
  velW = motion_cmds.angular.z;
  torso_height = motion_cmds.linear.z;
}

// Subscribe to ROS topic "/base_cmds"
ros::Subscriber<geometry_msgs::Twist> sub_motion(
  "base_cmds", &motion_cb);

// --- FOR TESTING
std_msgs::Float32 float_msg;
ros::Publisher debugger("calcs", &float_msg);

void move_base() {

  // Calculate motor speeds.
  motorFR_speed =
  motor_vectorFR[0] * velX + motor_vectorFR[1] * velY +
  motor_vectorFR[2] * velW * pi / 180 * base_radius;

  motorFL_speed =
  motor_vectorFL[0] * velX + motor_vectorFL[1] * velY +
  motor_vectorFL[2] * velW * pi / 180 * base_radius;

  float_msg.data = motorFL_speed;

  // Motor direction.
  cwFR = motorFR_speed < 0 ? true : false;
  cwFL = motorFL_speed < 0 ? true : false;

  motorFR_speed = abs(motorFR_speed);
  motorFL_speed = abs(motorFL_speed);

  // Motor Speeds are capped so preserve speed ratio.
  if (motorFR_speed >= motorFL_speed && motorFR_speed > max_speed) {
    motorFL_speed = motorFL_speed * max_speed / motorFR_speed;
    motorFR_speed = max_speed;

  } else if (motorFL_speed >= motorFR_speed && motorFL_speed > max_speed) {
    motorFR_speed = motorFR_speed * max_speed / motorFL_speed;
    motorFL_speed = max_speed;
  }

  // Convert speeds to 8-bit.
  motorFR_speed = map(motorFR_speed, 0, max_speed, 0, 255);
  motorFL_speed = map(motorFL_speed, 0, max_speed, 0, 255);

  // Set motor direction.
  if (cwFR == true) {
    digitalWrite(motorFR_d1, LOW);
    digitalWrite(motorFR_d2, HIGH);
    digitalWrite(motorBL_d1, HIGH);
    digitalWrite(motorBL_d2, LOW);
  } else {
    digitalWrite(motorFR_d1, HIGH);
    digitalWrite(motorFR_d2, LOW);
    digitalWrite(motorBL_d1, LOW);
    digitalWrite(motorBL_d2, HIGH);
  }

  if (cwFL == true) {
    digitalWrite(motorFL_d1, LOW);
    digitalWrite(motorFL_d2, HIGH);
    digitalWrite(motorBR_d1, HIGH);
    digitalWrite(motorBR_d2, LOW);
  } else {
    digitalWrite(motorFL_d1, HIGH);
    digitalWrite(motorFL_d2, LOW);
    digitalWrite(motorBR_d1, LOW);
    digitalWrite(motorBR_d2, HIGH);
  }

  // Accelerate motors.
  // --> may need varying rates..
  for (int i = 0; i < 255; i++) {

    if (motorFR_speed >= i) {
      analogWrite(motorFR_pwm, i);
      analogWrite(motorBL_pwm, i);
    }
    if (motorFL_speed >= i) {
      analogWrite(motorFL_pwm, i);
      analogWrite(motorBR_pwm, i);
    }
    delay(accel_delay);
  }
}

void move_torso() {

}

void setup() {

  pinMode(motorFR_pwm, OUTPUT);
  pinMode(motorFR_d1, OUTPUT);
  pinMode(motorFR_d2, OUTPUT);

  pinMode(motorFL_pwm, OUTPUT);
  pinMode(motorFL_d1, OUTPUT);
  pinMode(motorFL_d2, OUTPUT);

  pinMode(motorBR_pwm, OUTPUT);
  pinMode(motorBR_d1, OUTPUT);
  pinMode(motorBR_d2, OUTPUT);

  pinMode(motorBL_pwm, OUTPUT);
  pinMode(motorBL_d1, OUTPUT);
  pinMode(motorBL_d2, OUTPUT);

  // ros setup
  nh.initNode();
  nh.subscribe(sub_motion);

  // --- FOR TESTING
  nh.advertise(debugger);

}

void loop() {

  // State 1: Control the base ---
  move_base(); // TODO will need to pass error adjustments as arg...

  // State 2: Control the torso ---
  move_torso();

  // State 3: Measure the batteries ---

  // State 4: Measure ultrasonics ---

  // State 5: Calculate base error ---

  debugger.publish(&float_msg);
  nh.spinOnce();
  delay(1);
}
