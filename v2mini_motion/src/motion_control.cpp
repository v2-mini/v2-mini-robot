#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <ros/console.h>

/**
 * Control movements of the base and torso autonomously, or with a UI.
 */

void keyboard_ui(geometry_msgs::Twist& base_msg) {

  // get the commands...

  base_msg.linear.x = 1;
  base_msg.linear.y = 1;
  base_msg.linear.z = 1;
  base_msg.angular.z = 1;
}

void autonomous(geometry_msgs::Twist& base_msg) {
  // todo
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "motion_control");
  ros::NodeHandle n;

  std::string control_type;
  ros::NodeHandle pnh("~");
  // Need the type of control (_control:=auto or _control:=remote).
  pnh.getParam("control", control_type);

  if (control_type != "remote" && control_type != "auto") {
    control_type = "remote"; // change default to auto later...
  }

  ROS_DEBUG_STREAM("control type:" << control_type);

  // Publish base velocities and torso height
  ros::Publisher base_pub = n.advertise<geometry_msgs::Twist>(
    "base_cmds", 3);

  // loop at 10 Hz
  ros::Rate loop_rate(10);

  while (ros::ok()) {

    geometry_msgs::Twist base_msg;

    if (control_type == "remote"){
      keyboard_ui(base_msg);
    } else {
      autonomous(base_msg);
    }

    base_pub.publish(base_msg);

    loop_rate.sleep();

  }
  return 0;
}
