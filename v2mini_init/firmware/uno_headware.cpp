#include <Arduino.h>
#include <ros.h>
#include "geometry_msgs/Twist.h"

ros::NodeHandle nh;

const int RED = 13;
const int GREEN = 45;
const int BLUE = 46;

int curr_color = RED;
int face_input = 0;

void motion_cb(const geometry_msgs::Twist& motion_cmds)
{
  // todo msg type to use? probably don't need twist
  face_input = motion_cmds.linear.x;
}

// Subscribe to ROS topic "/torso_cmds"
ros::Subscriber<geometry_msgs::Twist> sub_motion(
  "torso_cmds", &motion_cb);

void move_torso()
{
  return;
}

void toggle_face()
{
  if (face_input != 0)
  {
    if (curr_color == RED)
    {
      digitalWrite(curr_color, LOW);
      digitalWrite(GREEN, HIGH);
      curr_color = GREEN;
    }
    else if (curr_color == GREEN)
    {
      digitalWrite(curr_color, LOW);
      digitalWrite(BLUE, HIGH);
      curr_color = BLUE;
    }
    else
    {
      digitalWrite(curr_color, LOW);
      digitalWrite(RED, HIGH);
      curr_color = RED;
    }
  }
}

void setup()
{
  // setup pins
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);

  // setup ros
  nh.initNode();
  nh.subscribe(sub_motion);
}

void loop()
{
  move_torso();

  toggle_face();

  nh.spinOnce();
}
