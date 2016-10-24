#include <Arduino.h>
#include <ros.h>
#include "geometry_msgs/Twist.h"

ros::NodeHandle nh;

const int max_speed = 1000;         // rps ...update val
const int wheel_radius = 4;         // cm
const int base_radius = 16;         // cm
const int accel_const = 10;

// Front-Right base motor.
const int motorFR_pwm = 3;
const int motorFR_d1 = 2;
const int motorFR_d2 = 4;

// Front-Left base motor.
const int motorFL_pwm = 5;
const int motorFL_d1 = 7;
const int motorFL_d2 = 8;

// Back-Right base motor.
const int motorBR_pwm = 6;
const int motorBR_d1 = 10;
const int motorBR_d2 = 11;

// Back-Left base motor.
const int motorBL_pwm = 9;
const int motorBL_d1 = 12;
const int motorBL_d2 = 13;

// Only need to calculate motor speeds for FR & FL.
const float motor_vectorFR[] = {-0.7071, 0.7071, 1.0};
const float motor_vectorFL[] = {-0.7071, -0.7071, 1.0};

float velX, velY, velW, torso_height;

volatile bool cwFR_actual, cwFL_actual = true; // TODO correct init? --> NO
volatile float motorFR_speedPrev, motorFL_speedPrev = 0;
volatile float motorBR_speedPrev, motorBL_speedPrev = 0;
volatile float motorFR_speedErr, motorFL_speedErr = 0;
volatile float motorBR_speedErr, motorBL_speedErr = 0;

// "base_cmds" subscription callback.
void motion_cb(const geometry_msgs::Twist& motion_cmds) {
  velX = motion_cmds.linear.x;
  velY = motion_cmds.linear.y;
  velW = motion_cmds.angular.z;
  torso_height = motion_cmds.linear.z;
}

// Subscribe to ROS topic "/base_cmds"
ros::Subscriber<geometry_msgs::Twist> sub_motion(
  "base_cmds", &motion_cb);

/*********************************************************
..../... todo

speed always > 0.

prev_speed > 0 to accel and < 0 to decel.

*********************************************************/
float motor_speed(float speed, float prev_speed) {

  float delta_speed = prev_speed - speed;
  float abs_delta_speed = abs(delta_speed);
  float delta_accel = accel_const - abs_delta_speed;
  float final_speed;

  if (delta_speed > 0) {
    // Increase final_speed to accelerate.
    if (delta_accel > 0) {
      final_speed = prev_speed + abs_delta_speed;
    } else {
      final_speed = prev_speed + accel_const;
    }

  } else {
    // Decrease final_speed to decelerate.
    if (delta_accel > 0) {
      final_speed = prev_speed - abs_delta_speed;
    } else {
      final_speed = prev_speed - accel_const;
    }
  }

  return final_speed;
}

/*********************************************************
..../... todo
*********************************************************/
void move_base() {

  float motorFR_speed, motorBL_speed, motorFL_speed, motorBR_speed;
  bool cwFR_desired, cwFL_desired;

  // Calculate motor speeds.
  motorFR_speed =
  motor_vectorFR[0] * velX + motor_vectorFR[1] * velY +
  motor_vectorFR[2] * velW * PI / 180 * base_radius;

  motorFL_speed =
  motor_vectorFL[0] * velX + motor_vectorFL[1] * velY +
  motor_vectorFL[2] * velW * PI / 180 * base_radius;

  motorBL_speed = -motorFR_speed;
  motorBR_speed = -motorFL_speed;

  // Adjust for error.
  motorFR_speed = motorFR_speed + motorFR_speedErr;
  motorFL_speed = motorFL_speed + motorFL_speedErr;
  motorBR_speed = motorBR_speed + motorBR_speedErr;
  motorBL_speed = motorBL_speed + motorBL_speedErr;

  // Motor direction.
  cwFR_desired = motorFR_speed < 0 ? true : false;
  cwFL_desired = motorFL_speed < 0 ? true : false;

  motorFR_speed = abs(motorFR_speed);
  motorFL_speed = abs(motorFL_speed);
  motorBR_speed = abs(motorBR_speed);
  motorBL_speed = abs(motorBL_speed);

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
  motorBR_speed = map(motorBR_speed, 0, max_speed, 0, 255);
  motorBL_speed = map(motorBL_speed, 0, max_speed, 0, 255);


  // TODO -- NEED TO THINK ABOUT WHEN STOPPED AND RIGHT AFTER INIT... vv


  // Are the FR & BL motors already turning in the desired direction?
  if (cwFR_desired == cwFR_actual) {

    // Yes, so just set speed up or down.
    motorFR_speedPrev = motor_speed(motorFR_speed, motorFR_speedPrev);
    motorBL_speedPrev = motor_speed(motorBL_speed, motorBL_speedPrev);

  } else {

      // No, so slow-down and change direction at inflection.
      motorFR_speedPrev = motor_speed(motorFR_speed, -motorFR_speedPrev);
      motorBL_speedPrev = motor_speed(motorBL_speed, -motorBL_speedPrev);

      // If the motor speed comes back negative, change direction.
      if (motorFR_speedPrev < 0) {

        motorFR_speedPrev = abs(motorFR_speedPrev);
        motorBL_speedPrev = abs(motorBL_speedPrev);

        cwFR_actual = cwFR_desired;
        cwFL_actual = cwFL_desired;

        if (cwFR_desired == true) {
          // turn CW
          digitalWrite(motorFR_d1, LOW);
          digitalWrite(motorFR_d2, HIGH);
          digitalWrite(motorBL_d1, HIGH);
          digitalWrite(motorBL_d2, LOW);
        } else {
          // turn CCW
          digitalWrite(motorFR_d1, HIGH);
          digitalWrite(motorFR_d2, LOW);
          digitalWrite(motorBL_d1, LOW);
          digitalWrite(motorBL_d2, HIGH);
        }
      }
    }






  if (cwFL_desired == cwFL_actual) {
  //
  //   motorFL_speedPrev = motor_speed(motorFL_speed, motorFL_speedPrev);
  //   motorBR_speedPrev = motor_speed(motorBR_speed, motorBR_speedPrev);
  //
  // } else {
  //
  //   if (cwFL_desired == true) {
  //     digitalWrite(motorFL_d1, LOW);
  //     digitalWrite(motorFL_d2, HIGH);
  //     digitalWrite(motorBR_d1, HIGH);
  //     digitalWrite(motorBR_d2, LOW);
  //   } else {
  //     digitalWrite(motorFL_d1, HIGH);
  //     digitalWrite(motorFL_d2, LOW);
  //     digitalWrite(motorBR_d1, LOW);
  //     digitalWrite(motorBR_d2, HIGH);
  //   }
  //
  //   motorFL_speedPrev = motor_speed(motorFL_speed, -motorFL_speedPrev);
  //   motorBR_speedPrev = motor_speed(motorBR_speed, -motorBR_speedPrev);
  }

  // cwFR_actual = cwFR_desired;
  // cwFL_actual = cwFL_desired;

  analogWrite(motorFR_pwm, motorFR_speedPrev);
  analogWrite(motorBL_pwm, motorBL_speedPrev);
  analogWrite(motorFL_pwm, motorFL_speedPrev);
  analogWrite(motorBR_pwm, motorBR_speedPrev);

}

void move_torso() {
  return;
}

void measure_batts() {
  return;
}

void measure_ultras() {
  return;
}

void calc_base_error() {
  return;
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

}

void loop() {

  // State 1: Control the base ---
  move_base();

  // State 2: Control the torso ---
  move_torso();

  // State 3: Measure the batteries ---
  measure_batts();

  // State 4: Measure ultrasonics ---
  measure_ultras();

  // State 5: Calculate base error ---
  calc_base_error();

  nh.spinOnce();
}
