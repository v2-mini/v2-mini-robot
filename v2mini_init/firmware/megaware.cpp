#include <Arduino.h>
#include <ros.h>
#include "geometry_msgs/Twist.h"

ros::NodeHandle nh;

// --> probably need min threshold speed too (ie not 0).
const int max_speed = 1000;         // rps ...update val
const int wheel_radius = 400;         // cm ...
const int base_radius = 16;         // cm ...
const int speed_increment = 1;

// Front-Right base motor.
const int motorF_pwm = 5;
const int motorF_d1 = 48;
const int motorF_d2 = 46;

// Front-Left base motor.
const int motorL_pwm = 3;
const int motorL_d1 = 40;
const int motorL_d2 = 42;

// Back-Right base motor.
const int motorR_pwm = 4;
const int motorR_d1 = 50;
const int motorR_d2 = 52;

// Back-Left base motor.
const int motorB_pwm = 2;
const int motorB_d1 = 44;
const int motorB_d2 = 38;

// Only need to calculate motor speeds for FR & FL.
const float motor_vectorR[] = {0, 1.0, 1.0};
const float motor_vectorF[] = {-1.0, 0, 1.0};

float velX, velY, velW, torso_height;

volatile bool cwR_actual, cwF_actual;
volatile bool R_isStopped, F_isStopped = true;
volatile float motorF_speedPrev, motorL_speedPrev = 0;
volatile float motorR_speedPrev, motorB_speedPrev = 0;
volatile float motorF_speedErr, motorL_speedErr = 0;
volatile float motorR_speedErr, motorB_speedErr = 0;

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

  float motorF_speed, motorB_speed, motorL_speed, motorR_speed;
  bool cwR_desired, cwF_desired;

  // Calculate desired motor speeds.
  motorF_speed =
  motor_vectorR[0] * velX + motor_vectorR[1] * velY +
  motor_vectorR[2] * velW * PI / 180 * base_radius;

  motorL_speed =
  motor_vectorF[0] * velX + motor_vectorF[1] * velY +
  motor_vectorF[2] * velW * PI / 180 * base_radius;

  motorB_speed = -motorF_speed;
  motorR_speed = -motorL_speed;

  // Adjust for error. ------>  (TODO VARIFY METHOD).
  motorF_speed = motorF_speed + motorF_speedErr;
  motorL_speed = motorL_speed + motorL_speedErr;
  motorR_speed = motorR_speed + motorR_speedErr;
  motorB_speed = motorB_speed + motorB_speedErr;

  // Reset error
  motorF_speedErr, motorB_speedErr = 0;
  motorL_speedErr, motorR_speedErr = 0;

  // Calculate desired motor direction.
  cwR_desired = motorF_speed < 0 ? true : false;
  cwF_desired = motorL_speed < 0 ? true : false;

  // Take the magnitude.
  motorF_speed = abs(motorF_speed);
  motorL_speed = abs(motorL_speed);
  motorR_speed = abs(motorR_speed);
  motorB_speed = abs(motorB_speed);

  // Motor Speeds are capped so preserve speed ratio.
  if (motorF_speed >= motorL_speed && motorF_speed > max_speed) {
    motorL_speed = motorL_speed * max_speed / motorF_speed;
    motorF_speed = max_speed;

  } else if (motorL_speed >= motorF_speed && motorL_speed > max_speed) {
    motorF_speed = motorF_speed * max_speed / motorL_speed;
    motorL_speed = max_speed;
  }

  // Convert speeds to 8-bit.
  motorF_speed = map(motorF_speed, 0, max_speed, 0, 255);
  motorL_speed = map(motorL_speed, 0, max_speed, 0, 255);
  motorR_speed = map(motorR_speed, 0, max_speed, 0, 255);
  motorB_speed = map(motorB_speed, 0, max_speed, 0, 255);

  // Acceleration Logic for motor sets FR and FL -------------------- :
  // 1. Stopped and want to move or stay stopped.
  // 2. Already moving in direction and want to speed up or down.
  // 3. Currently moving in the opposite direction and want to switch.

  // FR Motor Set ---- :
  if (R_isStopped) {

      if (motorF_speed > 0) {

        R_isStopped = false;

        if (cwR_desired) {
          // Set motor rotation CW
          digitalWrite(motorF_d1, LOW);
          digitalWrite(motorF_d2, HIGH);
          digitalWrite(motorB_d1, HIGH);
          digitalWrite(motorB_d2, LOW);

        } else {
          // Set motor rotation CCW
          digitalWrite(motorF_d1, HIGH);
          digitalWrite(motorF_d2, LOW);
          digitalWrite(motorB_d1, LOW);
          digitalWrite(motorB_d2, HIGH);
        }

        cwR_actual = cwR_desired;

        // Accelerate motors
        motorF_speedPrev = new_motor_speed(motorF_speed, motorF_speedPrev);
        motorB_speedPrev = new_motor_speed(motorB_speed, motorB_speedPrev);
      }

  } else if (cwR_desired == cwR_actual && motorF_speed > 0) {

    // Accelerate OR Decelerate motors
    motorF_speedPrev = new_motor_speed(motorF_speed, motorF_speedPrev);
    motorB_speedPrev = new_motor_speed(motorB_speed, motorB_speedPrev);

  } else {

    // Decelerate motors (to an eventual stop)
    motorF_speedPrev = new_motor_speed(0, motorF_speedPrev);
    motorB_speedPrev = new_motor_speed(0, motorB_speedPrev);

    if (motorF_speedPrev == 0 || motorB_speedPrev == 0) {
      R_isStopped = true;
    }

  }

  // FL Motor Set ---- :
  if (F_isStopped) {

      if (motorL_speed > 0) {

        F_isStopped = false;

        if (cwF_desired) {
          // Set motor rotation CW
          digitalWrite(motorL_d1, LOW);
          digitalWrite(motorL_d2, HIGH);
          digitalWrite(motorR_d1, HIGH);
          digitalWrite(motorR_d2, LOW);

        } else {
          // Set motor rotation CCW
          digitalWrite(motorL_d1, HIGH);
          digitalWrite(motorL_d2, LOW);
          digitalWrite(motorR_d1, LOW);
          digitalWrite(motorR_d2, HIGH);
        }

        cwF_actual = cwF_desired;

        // Accelerate motors
        motorL_speedPrev = new_motor_speed(motorL_speed, motorL_speedPrev);
        motorR_speedPrev = new_motor_speed(motorR_speed, motorR_speedPrev);
      }

  } else if (cwF_desired == cwF_actual && motorL_speed > 0) {

    // Accelerate OR Decelerate motors
    motorL_speedPrev = new_motor_speed(motorL_speed, motorL_speedPrev);

    motorR_speedPrev = new_motor_speed(motorR_speed, motorR_speedPrev);

  } else {

    // Decelerate motors (and eventually stop)
    motorL_speedPrev = new_motor_speed(0, motorL_speedPrev);
    motorR_speedPrev = new_motor_speed(0, motorR_speedPrev);

    if (motorL_speedPrev == 0 || motorR_speedPrev == 0) {
      F_isStopped = true;
    }

  }

  analogWrite(motorF_pwm, motorF_speedPrev);
  analogWrite(motorB_pwm, motorB_speedPrev);
  analogWrite(motorL_pwm, motorL_speedPrev);
  analogWrite(motorR_pwm, motorR_speedPrev);

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

  pinMode(motorF_pwm, OUTPUT);
  pinMode(motorF_d1, OUTPUT);
  pinMode(motorF_d2, OUTPUT);

  pinMode(motorL_pwm, OUTPUT);
  pinMode(motorL_d1, OUTPUT);
  pinMode(motorL_d2, OUTPUT);

  pinMode(motorR_pwm, OUTPUT);
  pinMode(motorR_d1, OUTPUT);
  pinMode(motorR_d2, OUTPUT);

  pinMode(motorB_pwm, OUTPUT);
  pinMode(motorB_d1, OUTPUT);
  pinMode(motorB_d2, OUTPUT);

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
