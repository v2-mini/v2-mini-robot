/*
 * Ultrasonic Range Finder
 *
 * 
 */
 
#include <ros.h>
#include <std_msgs/Float32.h>

//Setup ROS
std_msgs::Float32 sonar_msg;
ros::Publisher pub_sonar("sonar", &sonar_msg);
ros::NodeHandle nh;

const int pingPin = 7;

void setup() {
  nh.initNode();
  nh.advertise(pub_sonar);
 
}

void loop() {

  float duration, dist_cm;

  // Send pulse
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // Read the return signal
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);

  // convert the time into a distance:
  // Vsound = 29us/cm
  dist_cm = duration / 29 / 2;
  
  sonar_msg.data = dist_cm;

  pub_sonar.publish( &sonar_msg );

  nh.spinOnce();

  delay(500);
  
}










