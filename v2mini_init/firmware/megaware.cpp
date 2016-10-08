#include <ros.h>
#include <std_msgs/String.h>

#include <Arduino.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher megaware("megaware", &str_msg);

char hello[16] = "hello fn world!";

void setup()
{
  nh.initNode();
  nh.advertise(megaware);
}

void loop()
{
  str_msg.data = hello;
  megaware.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
