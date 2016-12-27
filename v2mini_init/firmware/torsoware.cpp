#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include "AFMotor.h"
#include "geometry_msgs/Twist.h"

ros::NodeHandle nh;

const int TOTAL_FACE_SERVOS = 5;
const int FACE_PINS[] = {23,25,27,29,31};
const int TOTAL_BUTTON_INS = 7;
const int BUTTON_INS[] = {37,39,41,43,47,49,51};
const int EYE_RGB_PINS[] = {45,44,46};
const int EYE_RGB_SCALE[] = {120,120,120};
const int EXPRESSION_SET[][8] =  {{100,90,80,90,80,0,50,50},     //neutral
                                  {150,115,90,70,30,0,0,50},     //sad
                                  {20,90,80,90,160,0,50,0},      //happy
                                  {80,65,80,115,100,50,0,0},     //mad
                                  {20,85,150,95,160,50,50,0},    //interested
                                  {20,105,180,75,90,50,0,50}};   //uncertain

const int LINACT_APIN[] = {10,0,12,11};
const int LINACT_MAX[] = {800,800,800,800};
const int LINACT_MIN[] = {200,200,200,200};


const int TORSO_MAXV = 5; // deg/s
const int TORSO_PIN = 53;
const int TORSO_MAXH = 120;
const int TORSO_MINH = 70;
const int TORSO_AVGH = (TORSO_MAXH + TORSO_MINH) / 2;

// button vars
int button_vals[] = {0,0,0,0,0,0,0};
int button_last_vals[] = {0,0,0,0,0,0,0};
int button_active[] = {0,0,0,0,0,0,0};

// torso vars
Servo torso;
int torso_input = 0;
int torso_current = TORSO_AVGH;
unsigned long prev_time;

// head vars
AF_DCMotor wrist_motor(1);
AF_DCMotor tilt_motor(2);
AF_DCMotor lh_motor(3);
AF_DCMotor rh_motor(4);

AF_DCMotor linacts[] = {wrist_motor,tilt_motor,lh_motor,rh_motor};

int linact_inputs[] = {0,0,0,0};
int prev_linact_pos[] = {512,512,300,300};
int currentAct = 0;

// face vars
int lastread;
int face_input = 0;
int expression = 0;
Servo face_servos[TOTAL_FACE_SERVOS];

void motion_cb(const geometry_msgs::Twist& motion_cmds)
{
 face_input = motion_cmds.linear.x;
 torso_input = motion_cmds.linear.y;
 linact_inputs[1] = motion_cmds.linear.z;
}

// Subscribe to ROS topic "/torso_cmds"
ros::Subscriber<geometry_msgs::Twist> sub_motion(
 "torso_cmds", &motion_cb);

// geometry_msgs::Twist debug_msg;
// ros::Publisher torso_debugger("torso_debugger", &debug_msg);

void linAct(int actuator_num)
{
  int actual_pos;
  int target_pos;
  int pos_error;
  int linact_vel;

  // increment the target position up or down each cycle
  target_pos = prev_linact_pos[actuator_num] + linact_inputs[actuator_num];
  actual_pos = analogRead(LINACT_APIN[actuator_num]);

  // set limits for input value
  if (target_pos > LINACT_MAX[actuator_num])
  {
    target_pos = LINACT_MAX[actuator_num];
  }
  else if (target_pos < LINACT_MIN[actuator_num])
  {
    target_pos = LINACT_MIN[actuator_num];
  }

  pos_error = abs(actual_pos - target_pos);

  // if(currentAct == actuator_num){
  //   debug_msg.linear.x = target_pos; // ----> comment out
  //   debug_msg.linear.y = actual_pos;
  //   debug_msg.linear.z = actuator_num;
  // }


  // todo --> replace with PID
  linact_vel = min(max(pos_error * 2, 16), 255);

  linacts[actuator_num].setSpeed(linact_vel);

  if (pos_error < 8)
  {
    //turns motor off
    linacts[actuator_num].run(RELEASE);
  }
  else if (actual_pos > target_pos)
  {
    linacts[actuator_num].run(FORWARD);
  }
  else if(actual_pos <  target_pos)
  {
    linacts[actuator_num].run(BACKWARD);
  }

  prev_linact_pos[actuator_num] = target_pos;

}

void setTorsoServo(float increm)
{
  // values are within allowable range
  if (torso_current <= TORSO_MAXH && torso_current >= TORSO_MINH)
  {
    if (torso_current == TORSO_MAXH && increm < 0)
    {
      // only allow increment down
      torso_current += increm;
    }
    else if (torso_current == TORSO_MINH && increm > 0)
    {
      // only allow increment up
      torso_current += increm;
    }
    else if (torso_current != TORSO_MAXH && torso_current != TORSO_MINH)
    {
      // increment up or down
      torso_current += increm;
    }
  }

  // keep value in range for any increment
  if (torso_current > TORSO_MAXH)
  {
    torso_current = TORSO_MAXH;
  }
  else if (torso_current < TORSO_MINH)
  {
    torso_current = TORSO_MINH;
  }

  torso.write(torso_current);
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

void setEyeColor(int r,int g,int b)
{
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

void readButtons()
{
  for (int i = 0; i < TOTAL_BUTTON_INS; i++)
  {
    button_last_vals[i] = button_vals[i];
    button_vals[i]=digitalRead(BUTTON_INS[i]);
    button_active[i]= !button_vals[i] && button_last_vals[i];
  }

  face_input = button_active[6];
  setTorsoServo(button_vals[5]-button_vals[4]);
  currentAct += button_active[3]-button_active[2];

  if(currentAct < 0 )
  {
    currentAct = 3;
  }

  if(currentAct > 3 )
  {
    currentAct = 0;
  }

  linact_inputs[currentAct] = (button_vals[1]-button_vals[0]);

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

  for (int i = 0; i < TOTAL_BUTTON_INS; i++)
  {
    pinMode(BUTTON_INS[i], INPUT_PULLUP);
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
 // nh.advertise(torso_debugger); // comment out when not debugging

}

void loop()
{
  // control torso speed by limiting occurrance (replace with timer interrupt)
  if ((millis() - prev_time) > (1000.0 / TORSO_MAXV))
  {
    moveTorso();
    prev_time = millis();
  }
  readButtons();

  toggleFace();

  linAct(0);
  linAct(1);
  linAct(2);
  linAct(3);

 // torso_debugger.publish(&debug_msg); // comment out when not debugging
 nh.spinOnce();
}
