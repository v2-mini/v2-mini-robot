#include <Arduino.h>
#include <ros.h>
#include "geometry_msgs/Twist.h"

ros::NodeHandle nh;

const int TORSO_D = 8;

void motion_cb(const geometry_msgs::Twist& motion_cmds) {
  // msg type to use? probably don't need twist
}

// Subscribe to ROS topic "/torso_cmds"
ros::Subscriber<geometry_msgs::Twist> sub_motion(
  "torso_cmds", &motion_cb);

void move_torso() {
  return;
}

void setup() {

  // setup pins
  pinMode(TORSO_D, OUTPUT);

  // setup ros
  nh.initNode();
  nh.subscribe(sub_motion);
}

void loop() {

  move_torso();

  nh.spinOnce();
}
