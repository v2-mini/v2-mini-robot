#include <Arduino.h>
#include <ros.h>
#include "geometry_msgs/Twist.h"

ros::NodeHandle nh;

// --> probably need min threshold speed too (ie not 0).
const int max_speed = 1000;         // rps ...update val
const int wheel_radius = 4;         // cm ...
const int base_radius = 16;         // cm ...
const int speed_increment = 10;

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

volatile bool cwFR_actual, cwFL_actual;
volatile bool FR_isStopped, FL_isStopped = true;
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
Get the new motor speed from the desired speed and the
previous speed. Escentially, increment the previous speed
until it equals the desired speed.

Args:
  speed:
    0 if desired speed is 0 or if the desired motor
    direction and actual motor direction are opposing.
    Else, it's a (1-255).

  prev_speed:
    The previous motor speed used (0-255).

Returns:
  new_speed:
    The previous speed incremented in the direction of
    the desired speed (0-255).
*********************************************************/
float new_motor_speed(float speed, float prev_speed) {

  float new_speed;
  float delta_speed = prev_speed - speed;

  bool same_dir = speed > 0 ? true: false;
  bool full_increment = abs(delta_speed) > speed_increment ? true: false;
  bool decrease_speed = delta_speed > 0 ? true: false;

  if (!same_dir) {
    if (full_increment) {
      // Decrease speed.
      new_speed = prev_speed - speed_increment;
    } else {
      // Come to a stop.
      new_speed = 0;
    }

  } else if (decrease_speed) {
    if (full_increment) {
      // Decrease speed.
      new_speed = prev_speed - speed_increment;
    } else {
      // Set to desired.
      new_speed = speed;
    }

  } else {
    if (full_increment) {
      // Increase speed.
      new_speed = prev_speed + speed_increment;
    } else {
      // Set to desired.
      new_speed = speed;
    }
  }
  return new_speed;
}

/*********************************************************
Control the velocity of the base:

1 Calculate the desired motor speeds from ROS subscribed
  velocity values.

2 Adjust for previous error in motor speeds.

3 Accelerate or Decelerate the motors by comparing the
  desired state with the previous state.

- Acceleration is determined by the size of 'speed_increment',
  and by the delay between each 'analogWrite()'

*********************************************************/
void move_base() {

  float motorFR_speed, motorBL_speed, motorFL_speed, motorBR_speed;
  bool cwFR_desired, cwFL_desired;

  // Calculate desired motor speeds.
  motorFR_speed =
  motor_vectorFR[0] * velX + motor_vectorFR[1] * velY +
  motor_vectorFR[2] * velW * PI / 180 * base_radius;

  motorFL_speed =
  motor_vectorFL[0] * velX + motor_vectorFL[1] * velY +
  motor_vectorFL[2] * velW * PI / 180 * base_radius;

  motorBL_speed = -motorFR_speed;
  motorBR_speed = -motorFL_speed;

  // Adjust for error. ------>  (TODO VARIFY METHOD).
  motorFR_speed = motorFR_speed + motorFR_speedErr;
  motorFL_speed = motorFL_speed + motorFL_speedErr;
  motorBR_speed = motorBR_speed + motorBR_speedErr;
  motorBL_speed = motorBL_speed + motorBL_speedErr;

  // Reset error
  motorFR_speedErr, motorBL_speedErr = 0;
  motorFL_speedErr, motorBR_speedErr = 0;

  // Calculate desired motor direction.
  cwFR_desired = motorFR_speed < 0 ? true : false;
  cwFL_desired = motorFL_speed < 0 ? true : false;

  // Take the magnitude.
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

  // Acceleration Logic for motor sets FR and FL -------------------- :
  // 1. Stopped and want to move or stay stopped.
  // 2. Already moving in direction and want to speed up or down.
  // 3. Currently moving in the opposite direction and want to switch.

  // FR Motor Set ---- :
  if (FR_isStopped) {

      if (motorFR_speed > 0) {

        FR_isStopped = false;

        if (cwFR_desired) {
          // Set motor rotation CW
          digitalWrite(motorFR_d1, LOW);
          digitalWrite(motorFR_d2, HIGH);
          digitalWrite(motorBL_d1, HIGH);
          digitalWrite(motorBL_d2, LOW);

        } else {
          // Set motor rotation CCW
          digitalWrite(motorFR_d1, HIGH);
          digitalWrite(motorFR_d2, LOW);
          digitalWrite(motorBL_d1, LOW);
          digitalWrite(motorBL_d2, HIGH);
        }

        cwFR_actual = cwFR_desired;

        // Accelerate motors
        motorFR_speedPrev = new_motor_speed(motorFR_speed, motorFR_speedPrev);
        motorBL_speedPrev = new_motor_speed(motorBL_speed, motorBL_speedPrev);
      }

  } else if (cwFR_desired == cwFR_actual && motorFR_speed > 0) {

    // Accelerate OR Decelerate motors
    motorFR_speedPrev = new_motor_speed(motorFR_speed, motorFR_speedPrev);
    motorBL_speedPrev = new_motor_speed(motorBL_speed, motorBL_speedPrev);

  } else {

    // Decelerate motors (to an eventual stop)
    motorFR_speedPrev = new_motor_speed(0, motorFR_speedPrev);
    motorBL_speedPrev = new_motor_speed(0, motorBL_speedPrev);

    if (motorFR_speedPrev == 0 || motorBL_speedPrev == 0) {
      FR_isStopped = true;
    }

  }

  // FL Motor Set ---- :
  if (FL_isStopped) {

      if (motorFL_speed > 0) {

        FL_isStopped = false;

        if (cwFL_desired) {
          // Set motor rotation CW
          digitalWrite(motorFL_d1, LOW);
          digitalWrite(motorFL_d2, HIGH);
          digitalWrite(motorBR_d1, HIGH);
          digitalWrite(motorBR_d2, LOW);

        } else {
          // Set motor rotation CCW
          digitalWrite(motorFL_d1, HIGH);
          digitalWrite(motorFL_d2, LOW);
          digitalWrite(motorBR_d1, LOW);
          digitalWrite(motorBR_d2, HIGH);
        }

        cwFL_actual = cwFL_desired;

        // Accelerate motors
        motorFL_speedPrev = new_motor_speed(motorFL_speed, motorFL_speedPrev);
        motorBR_speedPrev = new_motor_speed(motorBR_speed, motorBR_speedPrev);
      }

  } else if (cwFL_desired == cwFL_actual && motorFL_speed > 0) {

    // Accelerate OR Decelerate motors
    motorFL_speedPrev = new_motor_speed(motorFL_speed, motorFL_speedPrev);

    motorBR_speedPrev = new_motor_speed(motorBR_speed, motorBR_speedPrev);

  } else {

    // Decelerate motors (and eventually stop)
    motorFL_speedPrev = new_motor_speed(0, motorFL_speedPrev);
    motorBR_speedPrev = new_motor_speed(0, motorBR_speedPrev);

    if (motorFL_speedPrev == 0 || motorBR_speedPrev == 0) {
      FL_isStopped = true;
    }

  }

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
