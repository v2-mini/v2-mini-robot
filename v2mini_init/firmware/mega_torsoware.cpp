#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include "geometry_msgs/Twist.h"

ros::NodeHandle nh;

const int TORSO_MAXV = 5; // deg/s

const int servoPin[] = {23,25,27,29,31};
const int expressions[][8] =  {{100,90,80,90,80,0,50,50},    //neutral
                              {150,115,90,70,30,0,0,50},     //sad
                              {20,90,80,90,160,0,50,0},      //happy
                              {80,65,80,115,100,50,0,0},     //mad
                              {20,85,150,95,160,50,50,0},    //interested
                              {20,105,180,75,90,50,0,50}};   //uncertain

const int rgbPins[] = {45,44,46};
const int rgbScale[] = {120,120,120};

const int TORSO_PIN = 53;
const int TORSO_MAXH = 130;
const int TORSO_MINH = 50;
const int TORSO_AVGH = (TORSO_MAXH + TORSO_MINH) / 2;

const int HEAD_D1 = 1;  // todo get actual pinout
const int HEAD_D2 = 2;
const int HEAD_PWM = 3;

Servo s0;
Servo s1;
Servo s2;
Servo s3;
Servo s4;
Servo torso;

int val;
int lastread;
int face_input = 0;
int expression = 0;
int torso_input = 0;
int torso_current = TORSO_AVGH;
int head_input = 0;

unsigned long prev_time;

void motion_cb(const geometry_msgs::Twist& motion_cmds)
{
  // todo msg type to use? don't need twist..
  face_input = motion_cmds.linear.x;
  torso_input = motion_cmds.linear.y;
  head_input = motion_cmds.linear.z;
}

// Subscribe to ROS topic "/torso_cmds"
ros::Subscriber<geometry_msgs::Twist> sub_motion(
  "torso_cmds", &motion_cb);

void tilt_head()
{
  //

}

void set_torso_servo(float increm)
{
  // value must be within limits
  if (torso_current < TORSO_MAXH && torso_current > TORSO_MINH)
  {
    // move torso up or down
    torso_current += increm;
    torso.write(torso_current);
  }
}

void move_torso()
{
  if (torso_input > 0)
  {
    set_torso_servo(1);
  }
  else if (torso_input < 0)
  {
    set_torso_servo(-1);
  }
}

void ledWrite(int r,int g,int b){
  digitalWrite(rgbPins[0], r);
  digitalWrite(rgbPins[1], g);
  digitalWrite(rgbPins[2], b);
}

void expressionSet(int exp){
  s1.write(expressions[exp][1]);
  s3.write(expressions[exp][3]);

  delay(50);
  s0.write(expressions[exp][0]);

  delay(50);
  s2.write(expressions[exp][2]);
  s4.write(expressions[exp][4]);

  ledWrite(expressions[exp][5],expressions[exp][6],expressions[exp][7]);
}

void allServo(int val){
  s0.write(val);
  s1.write(val);
  s2.write(val);
  s3.write(val);
  s4.write(val);
}

void eyesSet(int val){
   s2.write(val);
}

void toggle_face()
{
  if(face_input != 0)
  {
      if(!lastread){
        expression ++;
        lastread = 1;
         if(expression == 6)
         {
          expression = 0;
         }
       }
  }
  else
  {
    lastread = 0;
  }
   //s0.write(5*expression);
   //allServo(90+(10*expression));
   expressionSet(expression);
   delay(50);
}

void setup()
{
  // setup pins
  pinMode(44, OUTPUT);
  pinMode(45, OUTPUT);
  pinMode(46, OUTPUT);
  pinMode(33, INPUT_PULLUP);
  s0.attach(servoPin[0]);
  s1.attach(servoPin[1]);
  s2.attach(servoPin[2]);
  s3.attach(servoPin[3]);
  s4.attach(servoPin[4]);
  torso.attach(TORSO_PIN);

  prev_time = millis();

  // setup ros
  nh.initNode();
  nh.subscribe(sub_motion);

  torso.write(TORSO_AVGH);
}

void loop()
{
  if ((millis() - prev_time) > (1000.0 / TORSO_MAXV))
  {
    move_torso();
    prev_time = millis();
  }

  toggle_face();

  tilt_head();

  nh.spinOnce();
}
