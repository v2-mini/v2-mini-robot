#include <Arduino.h>
#include <ros.h>
#include "geometry_msgs/Twist.h"

ros::NodeHandle nh;

// --> probably need min threshold speed too (ie not 0).
const int MAX_MOTOR_SPEED = 150;         // rps ...update vals
const int WHEEL_RADIUS = 400;         // cm ...
const int BASE_RADIUS = 16;         // cm ...
const int SPEED_INC = 1;

// PINS { F, B, R, L }
const int BASE_MOTOR_PWM[] = {5, 2, 4, 3};
const int BASE_MOTOR_D1[] = {48, 44, 50, 40};
const int BASE_MOTOR_D2[] = {46, 38, 52, 42};

const float ROT_FACTOR = PI / 180 * BASE_RADIUS;

// Motor Matrix: (F, B, R, L) ^ T
// Each row is: { -sin(a), cos(a), ROT_FACTOR}
// Where a is the angle from the x-axis to the motor-axis
const float MOTOR_MATRIX[4][3] = {
  { -1.0, 0, ROT_FACTOR },
  { 1.0, 0, ROT_FACTOR },
  { 0, 1.0, ROT_FACTOR },
  { 0, -1.0, ROT_FACTOR },
};

// { x vel, y vel, angular vel }
float input_velocity[] = {0, 0, 0};
float torso_height = 0;

// { F, B, R, L }
float motor_speed_previous[] = {0, 0, 0, 0};
float motor_speed_error[] = {0, 0, 0, 0};
bool motor_stopped[] = {true, true, true, true};
bool cw_rotation_actual[3];

// "base_cmds" subscription callback.
void motion_cb(const geometry_msgs::Twist& motion_cmds) {
  input_velocity[0] = motion_cmds.linear.x;
  input_velocity[1] = motion_cmds.linear.y;
  input_velocity[2] = motion_cmds.angular.z;
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
  bool full_increment = abs(delta_speed) > SPEED_INC ? true: false;
  bool decrease_speed = delta_speed > 0 ? true: false;

  if (!same_dir) {
    if (full_increment) {
      // Decrease speed.
      new_speed = prev_speed - SPEED_INC;
    } else {
      // Come to a stop.
      new_speed = 0;
    }

  } else if (decrease_speed) {
    if (full_increment) {
      // Decrease speed.
      new_speed = prev_speed - SPEED_INC;
    } else {
      // Set to desired.
      new_speed = speed;
    }

  } else {
    if (full_increment) {
      // Increase speed.
      new_speed = prev_speed + SPEED_INC;
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

- Acceleration is determined by the size of 'SPEED_INC',
  and by the delay between each 'analogWrite()'

*********************************************************/
void move_base() {

  // F, B, R, L
  float motor_speed[] = {0, 0, 0, 0};
  bool cw_rotation[3];

  // Calculate desired motor speeds
  // These values are targets to ramp up or down towards.
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      motor_speed[i] += input_velocity[j] * MOTOR_MATRIX[i][j];
    }
  }

  // Vars for the determining the min and max speeds
  int min_elem, max_elem = 0;
  int min_speed, max_speed;

  for (int i = 0; i < 3; i++) {

    // Adjust for error. ------>  (TODO VARIFY METHOD)
    motor_speed[i] += motor_speed_error[i];

    motor_speed_error[i] = 0;

    // Calculate desired motor direction.
    cw_rotation[i] = motor_speed[i] < 0 ? true : false;

    // Take the magnitude.
    motor_speed[i] = abs(motor_speed[i]);

    // Keep track of the min and max speed elements
    if (i == 0) {
      min_speed = motor_speed[0];
      max_speed = motor_speed[0];

    } else {

      if (motor_speed[i] > max_speed) {
        max_elem = i;
        max_speed = motor_speed[i];

      } else if (motor_speed[i] < min_speed) {
        min_elem = i;
        min_speed = motor_speed[i];

      }
    }
  }

  for (int i = 0; i < 3; i++) {

    // Motor Speeds are capped so preserve speed ratio.
    // TODO THIS WILL NOT PRESERVE SPEED RATIO.. WHICH IS REQUIRED
    // TODO use min_speed and max_speed to scale. (could 'map()' it..?)

    // Convert speeds to 8-bit.
    motor_speed[i] = map(motor_speed[i], 0, MAX_MOTOR_SPEED, 0, 255);

    // Acceleration Logic for each motor -------------------- :
    // 1. Stopped and want to move or stay stopped.
    // 2. Already moving in direction and want to speed up or down.
    // 3. Currently moving in the opposite direction and want to switch

    if (motor_stopped[i]) {

      if (motor_speed[i] > 0) {

        motor_stopped[i] = false;

        if (cw_rotation[i]) {
          // Set motor rotation CW
          digitalWrite(BASE_MOTOR_D1[i], LOW);
          digitalWrite(BASE_MOTOR_D2[i], HIGH);

        } else {
          // Set motor rotation CCW
          digitalWrite(BASE_MOTOR_D1[i], HIGH);
          digitalWrite(BASE_MOTOR_D2[i], LOW);

        }

        cw_rotation_actual[i] = cw_rotation[i];

        // Get the incremented up motor speed
        motor_speed[i] = new_motor_speed(motor_speed[i], 0);

      }

    } else if (cw_rotation[i] == cw_rotation_actual[i] && motor_speed[i] > 0) {

      // Get the incremented up-or-down motor speed
      motor_speed[i] = new_motor_speed(
        motor_speed[i], motor_speed_previous[i]);

    } else {

      // Get the incremented down motor speed
      motor_speed[i] = new_motor_speed(0, motor_speed_previous[i]);

      if (motor_speed[i] == 0) {
        motor_stopped[i] = true;
      }

    }

    motor_speed_previous[i] = motor_speed[i];

  }

  for (int i = 0; i < 3; i++) {
    analogWrite(BASE_MOTOR_PWM[i], motor_speed[i]);
  }

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

  // setup pins
  for (int i = 0; i < 3; i++) {
    pinMode(BASE_MOTOR_PWM[i], OUTPUT);
    pinMode(BASE_MOTOR_D1[i], OUTPUT);
    pinMode(BASE_MOTOR_D2[i], OUTPUT);
  }

  // setup ros
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
