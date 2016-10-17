#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <stdio.h>
#include <ncurses.h>

/**
 * Control movements of the base and torso autonomously or with a UI.
 */

class Robot_UI {  // ------> move to separate node? or header for remote...

private:
  ros::NodeHandle n;
  ros::Publisher base_pub;

public:
  Robot_UI(ros::NodeHandle &nh) {
    n = nh;
    base_pub = n.advertise<geometry_msgs::Twist>("base_cmds", 1);
  }

  void keyboard_interface() {

    ros::Rate loop_rate(50);

    // Limitation of ncurses - can only read a single key at a time ..
    char key_cmd;
    initscr();
    noecho();
    cbreak();
    timeout(10);

    while(ros::ok()){

      geometry_msgs::Twist base_cmds;

      key_cmd = getch();

        if(key_cmd == 'w') {
          // set y-velocity -> forwards
          base_cmds.linear.y = 1000;

        } else if(key_cmd == 's') {
          // set y-velocity -> backwards
          base_cmds.linear.y = -600;

        } else if(key_cmd == 'd') {
          // set x-velocity -> right
          base_cmds.linear.x = 1000;

        } else if(key_cmd == 'a') {
          // set x-velocity -> left
          base_cmds.linear.x = -600;

        } else if(key_cmd == 'i') {
          // set angular velocity -> CCW
          base_cmds.angular.z = 1000;

        } else if(key_cmd == 'o') {
          // set angular velocity -> CW
          base_cmds.angular.z = -1000;

        } else if(key_cmd == 'r') {
          // set delta torso height -> up
          base_cmds.linear.z = 1000;

        } else if(key_cmd == 'f') {
          // set delta torso height -> down
          base_cmds.linear.z = -1000;

        } else if(key_cmd == 'b') {
          break;

        }
        //publish the movement commands
        base_pub.publish(base_cmds);
        loop_rate.sleep();
      }

      nocbreak();
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motion_control");
  ros::NodeHandle nh;

  // Get type of control from param (auto or remote)
  std::string control_type;
  ros::NodeHandle pnh("~");
  pnh.getParam("control", control_type);

  ROS_DEBUG_STREAM("control type:" << control_type);

  if (control_type == "remote"){
    Robot_UI robocontrol(nh);
    robocontrol.keyboard_interface();

  } else {
    // todo --> autonomous
  }

  return 0;
}
