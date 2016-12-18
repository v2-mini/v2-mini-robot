#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include <AFMotor.h>
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

const int TILT_APIN = 0;
const int TILT_MAX = 800;
const int TILT_MIN = 200;
const int TILT_AVG = (TILT_MAX + TILT_MIN) / 2;

const int TORSO_MAXV = 5; // deg/s
const int TORSO_PIN = 53;
const int TORSO_MAXH = 130;
const int TORSO_MINH = 50;
const int TORSO_AVGH = (TORSO_MAXH + TORSO_MINH) / 2;

// torso vars
Servo torso;
int torso_input = 0;
int torso_current = TORSO_AVGH;
unsigned long prev_time;

// head vars
AF_DCMotor tilt_motor(2);
int head_current, head_input = TILT_AVG;

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

geometry_msgs::Twist debug_msg;
ros::Publisher torso_debugger("torso_debugger", &debug_msg);

void tiltHead()
{
  int actual_pos;
  int pos_error;
  int tilt_vel;

  actual_pos = analogRead(TILT_APIN);

  // set limits for input value
  if (head_input > TILT_MAX)
  {
    head_input = TILT_MAX;
  }
  else if (head_input < TILT_MIN)
  {
    head_input = TILT_MIN;
  }

  pos_error = abs(actual_pos - head_input);

  debug_msg.linear.x = head_input;
  debug_msg.linear.y = actual_pos;
  debug_msg.linear.z = pos_error;

  // todo --> replace with PID
  tilt_vel = min(max(pos_error * 2, 16), 255);

  tilt_motor.setSpeed(tilt_vel);

  if (pos_error < 8)
  {
    //turns motor off
    tilt_motor.run(RELEASE);
  }
  else if (actual_pos > head_input)
  {
    tilt_motor.run(FORWARD);
  }
  else if(actual_pos < head_input)
  {
    tilt_motor.run(BACKWARD);
  }

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

  // set rx & tx to input
  pinMode(18, INPUT);
  pinMode(19, INPUT);

  for (int i = 0; i < 3; i++)
  {
    pinMode(EYE_RGB_PINS[i], OUTPUT);
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
  nh.advertise(torso_debugger); // comment out when not debugging

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

  torso_debugger.publish(&debug_msg); // comment out when not debugging
  nh.spinOnce();
}
