#include <Arduino.h>

#include <ros.h>
#include "geometry_msgs/Twist.h"

ros::NodeHandle nh;

// arduino vars
int out1 = 13;

// ros vars
int velX, velY, velAngular, torso_height;

void motion_cb(const geometry_msgs::Twist& motion_cmds) {
  velX = motion_cmds.linear.x;
  velY = motion_cmds.linear.y;
  velAngular = motion_cmds.angular.z;
  torso_height = motion_cmds.linear.z;
}

ros::Subscriber<geometry_msgs::Twist> sub_motion(
  "base_cmds", &motion_cb);

void move_base() {

  digitalWrite(out1, LOW);
  delay(velY*1000);
  digitalWrite(out1, HIGH);
  delay(velY*1000);
}

void move_torso() {
}

void setup() {

  // arduino setup
  pinMode(out1, OUTPUT);

  // ros setup
  nh.initNode();
  nh.subscribe(sub_motion);

}

void loop() {

  // State 1: Move the base ---
  move_base();


  // State 2: Move the torso ---

  // State 3: Measure the batteries ---

  // State 4: Measure ultrasonics ---

  // State 5: Calculate base error ---

  nh.spinOnce();
  delay(1);
}
