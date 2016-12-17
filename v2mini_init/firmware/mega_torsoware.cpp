#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include "geometry_msgs/Twist.h"

ros::NodeHandle nh;

const int TOTAL_FACE_SERVOS = 5;
const int FACE_PINS[] = {23,25,27,29,31};
const int EYE_RGB_PINS[] = {45,44,46};
const int EYE_RGB_SCALE[] = {120,120,120};
const int EXPRESSION_SET[][8] =  {{100,90,80,90,80,0,50,50},     //neutral
                                  {150,115,90,70,30,0,0,50},     //sad
                                  {20,90,80,90,160,0,50,0},      //happy
                                  {80,65,80,115,100,50,0,0},     //mad
                                  {20,85,150,95,160,50,50,0},    //interested
                                  {20,105,180,75,90,50,0,50}};   //uncertain

const int TILTVEL_PIN = 9;
const int TILTPOS_PIN = 0;
const int TILTUP_PIN = 4;
const int TILTDOWN_PIN = 5;
const int TILT_MAX = 800;
const int TILT_MIN = 200;
const int TILT_AVG = (TILT_MAX + TILT_MIN) / 2;

const int TORSO_MAXV = 5; // deg/s
const int TORSO_PIN = 53;
const int TORSO_MAXH = 95;
const int TORSO_MINH = 50;
const int TORSO_AVGH = (TORSO_MAXH + TORSO_MINH) / 2;

// torso vars
Servo torso;
int torso_input = 0;
int torso_current = TORSO_AVGH;
unsigned long prev_time;

// head vars
int head_current = TILT_AVG;
int head_input = 0;

// face vars
int lastread;
int face_input = 0;
int expression = 0;
Servo face_servos[TOTAL_FACE_SERVOS];

void motion_cb(const geometry_msgs::Twist& motion_cmds)
{
  face_input = motion_cmds.linear.x;
  torso_input = motion_cmds.linear.y;
  head_input = motion_cmds.linear.z;
}

// Subscribe to ROS topic "/torso_cmds"
ros::Subscriber<geometry_msgs::Twist> sub_motion(
  "torso_cmds", &motion_cb);

void tiltHead()
{
  int target_pos;
  int actual_pos;
  int tilt_vel;

  if (head_input > TILT_MAX)
  {
    target_pos = TILT_MAX;
  }
  else if (head_input < TILT_MIN)
  {
    target_pos = TILT_MIN;
  }
  else
  {
    target_pos = head_input;
  }

  actual_pos = analogRead(TILTPOS_PIN);

  if (actual_pos > target_pos)
  {
    digitalWrite(TILTUP_PIN, HIGH);
    digitalWrite(TILTDOWN_PIN, LOW);
  }
  else
  {
    digitalWrite(TILTUP_PIN,LOW);
    digitalWrite(TILTDOWN_PIN,HIGH);
  }

  tilt_vel = min(abs(actual_pos-target_pos) * 10, 255);
  analogWrite(TILTVEL_PIN, tilt_vel);
}

void setTorsoServo(float increm)
{
  // value must be within limits
  if (torso_current < TORSO_MAXH && torso_current > TORSO_MINH)
  {
    // move torso up or down
    torso_current += increm;
    torso.write(torso_current);
  }
}

void moveTorso()
{
  if (torso_input > 0)
  {
    setTorsoServo(1);
  }
  else if (torso_input < 0)
  {
    setTorsoServo(-1);
  }
}

void setEyeColor(int r,int g,int b){
  digitalWrite(EYE_RGB_PINS[0], r);
  digitalWrite(EYE_RGB_PINS[1], g);
  digitalWrite(EYE_RGB_PINS[2], b);
}

void setFacialExpression(int exp)
{
  face_servos[1].write(EXPRESSION_SET[exp][1]);
  face_servos[3].write(EXPRESSION_SET[exp][3]);

  delay(50);
  face_servos[0].write(EXPRESSION_SET[exp][0]);

  delay(50);
  face_servos[2].write(EXPRESSION_SET[exp][2]);
  face_servos[4].write(EXPRESSION_SET[exp][4]);

  setEyeColor(
    EXPRESSION_SET[exp][5],
    EXPRESSION_SET[exp][6],
    EXPRESSION_SET[exp][7]);
}

void toggleFace()
{
  if(face_input != 0)
  {
    if(!lastread)
    {
      expression++;
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

  setFacialExpression(expression);
}

void setup()
{
  // init pin modes
  pinMode(33, INPUT_PULLUP);

  for (int i = 0; i < 14; i++)
  {
    pinMode(i, OUTPUT);

    if (i < 3)
    {
      pinMode(EYE_RGB_PINS[i], OUTPUT);
    }
  }

  // attach servos to pins
  torso.attach(TORSO_PIN);

  for (int i = 0; i < TOTAL_FACE_SERVOS; i++)
  {
    face_servos[i].attach(FACE_PINS[i]);
  }

  // init timer for torso speed
  prev_time = millis();

  // init torso to average height
  torso.write(TORSO_AVGH);

  // init ros stuff
  nh.initNode();
  nh.subscribe(sub_motion);

}

void loop()
{
  // control torso speed by limiting occurrance (replace with timer interrupt)
  if ((millis() - prev_time) > (1000.0 / TORSO_MAXV))
  {
    moveTorso();
    prev_time = millis();
  }

  toggleFace();

  tiltHead();

  nh.spinOnce();
}
