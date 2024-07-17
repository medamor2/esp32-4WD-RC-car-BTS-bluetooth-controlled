/*
 BTS7960 -> Arduino uno
 MotorRight_R_EN - 4
 MotorRight_L_EN - 5
 MotorLeft_R_EN - 8
 MotorLeft_L_EN - 9
 Rpwm1 - 6
 Lpwm1 - 7
 Rpwm2 - 10
 Lpwm2 - 11
 
 ROBOT CONTROL STATES:
 0 - stop_Robot
 1 - turn_Right
 2 - turn_Left
 3 - go_Forward
 4 - go_Backward
 5 - move_RightForward
 6 - move_LeftForward
 7 - move_RightBackward
 8 - move_LeftBackward
 */
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
const int MotorRight_R_EN = 16; // Pin to control the clockwise direction of Right Motor
const int MotorRight_L_EN = 17; // Pin to control the counterclockwise direction of Right Motor 
const int MotorLeft_R_EN = 14; // Pin to control the clockwise direction of Left Motor
const int MotorLeft_L_EN = 27; // Pin to control the counterclockwise direction of Left Motor
const int Rpwm1 = 5; // pwm output - motor A
const int Lpwm1 = 18; // pwm output - motor B
const int Rpwm2 = 26; // pwm output - motor A
const int Lpwm2 = 25; // pwm output - motor B
long pwmLvalue = 255;
long pwmRvalue = 255;
byte pwmChannel;

const char startOfNumberDelimiter = '<';
const char endOfNumberDelimiter = '>';
int robotControlState;
int last_mspeed;

char c;
void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32cAr"); //Bluetooth device name
  //Setup Right Motors
  pinMode(MotorRight_R_EN, OUTPUT); //Initiates Motor Channel A1 pin
  pinMode(MotorRight_L_EN, OUTPUT); //Initiates Motor Channel A2 pin

  //Setup Left Motors
  pinMode(MotorLeft_R_EN, OUTPUT); //Initiates Motor Channel B1 pin
  pinMode(MotorLeft_L_EN, OUTPUT); //Initiates Motor Channel B2 pin
  
  //Setup PWM pins as Outputs
  pinMode(Rpwm1, OUTPUT);
  pinMode(Lpwm1, OUTPUT);
  pinMode(Rpwm2, OUTPUT);
  pinMode(Lpwm2, OUTPUT);
  
  
  stop_Robot();
}// void setup()

void loop(){
  //int i = 0;  
  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
    c=SerialBT.read();
    processInput();
  }
}// void loop()

void processInput (){
  static long receivedNumber = 0;
  static boolean negative = false;
  switch (c){
  case endOfNumberDelimiter:
    if (negative)
      SetPWM(- receivedNumber, pwmChannel);
    else
      SetPWM(receivedNumber, pwmChannel);

    // fall through to start a new number
  case startOfNumberDelimiter:
    receivedNumber = 0;
    negative = false;
    pwmChannel = 0;
    break;

  case 'F': // Go FORWARD
    go_Forward(255);
    //Serial.println("forward");
    break;

  case 'B': // Go BACK
    go_Backwad(255);
    break;

  case 'R':
    turn_Right(255);
    break;

  case 'L':
    turn_Left(255);
    break;

  case 'I': // Top Right
    move_RightForward(255);
    break; 

  case 'G': // Top Left
    move_LeftForward(255);
    break;  

  case 'J': // Bottom Right
    move_RightBackward(255);
    break; 

  case 'H': // Bottom Left
    move_LeftBackward(255);
    break;  

  case 'S':
    stop_Robot();
    break;

  case 'X':
    pwmChannel = 1; // Rpwm1
    break;
  case 'Y': // Lpwm1
    pwmChannel = 2;
    break;

  case '1' ... '9':
    receivedNumber *= 10;
    receivedNumber += c - '0';
    break;

  case '-':
    negative = true;
    break;
  } // end of switch
} // void processInput ()

void stop_Robot(){ // robotControlState = 0
  if(robotControlState!=0){
    //SetMotors(2); 
    analogWrite(Rpwm1, 0);
    analogWrite(Lpwm1, 0);
    analogWrite(Rpwm2, 0);
    analogWrite(Lpwm2, 0);
    robotControlState = 0;
  }
}// void stopRobot()

void turn_Right(int mspeed){ // robotControlState = 1
  if(robotControlState!=10 || last_mspeed!=mspeed){
    SetMotors(1);
    analogWrite(Rpwm1, 0);
    analogWrite(Lpwm1, mspeed);
    analogWrite(Rpwm2, mspeed);
    analogWrite(Lpwm2, 0);
    robotControlState=10;
    last_mspeed=mspeed;
  }
}// void turn_Right(int mspeed)

void turn_Left(int mspeed){ // robotControlState = 2
  if(robotControlState!=20 || last_mspeed!=mspeed){
    SetMotors(1);
    analogWrite(Rpwm1, mspeed);
    analogWrite(Lpwm1, 0);
    analogWrite(Rpwm2, 0);
    analogWrite(Lpwm2, mspeed);
    robotControlState=20;
    last_mspeed=mspeed;
  }
}// void turn_Left(int mspeed)

void go_Forward(int mspeed){ // robotControlState = 3
  if(robotControlState!=30 || last_mspeed!=mspeed){
    SetMotors(1);
    analogWrite(Rpwm1, mspeed);
    analogWrite(Lpwm1, 0);
    analogWrite(Rpwm2, mspeed);
    analogWrite(Lpwm2, 0);
    robotControlState=30;
    last_mspeed=mspeed;
  }
}// void goForward(int mspeed)

void go_Backwad(int mspeed){ // robotControlState = 4
  if(robotControlState!=40 || last_mspeed!=mspeed){
    SetMotors(1);
    analogWrite(Rpwm1, 0);
    analogWrite(Lpwm1, mspeed);
    analogWrite(Rpwm2, 0);
    analogWrite(Lpwm2, mspeed);
    robotControlState=40;
    last_mspeed=mspeed;
  }
}// void goBackwad(int mspeed)

void move_RightForward(int mspeed){ // robotControlState = 5
  if(robotControlState!=50 || last_mspeed!=mspeed){
    SetMotors(1);
    analogWrite(Rpwm1, mspeed*0.4);
    analogWrite(Lpwm1, 0);
    analogWrite(Rpwm2, mspeed);
    analogWrite(Lpwm2, 0);
    robotControlState=50;
    last_mspeed=mspeed;
  }
}// void move_RightForward(int mspeed)

void move_LeftForward(int mspeed){ // robotControlState = 6
  if(robotControlState!=60 || last_mspeed!=mspeed){
    SetMotors(1);
    analogWrite(Rpwm1, mspeed);
    analogWrite(Lpwm1, 0);
    analogWrite(Rpwm2, mspeed*0.4);
    analogWrite(Lpwm2, 0);
    robotControlState=60;
    last_mspeed=mspeed;  
  }
}// move_LeftForward(int mspeed)

void move_RightBackward(int mspeed){ // robotControlState = 7
  if(robotControlState!=70 || last_mspeed!=mspeed){
    SetMotors(1);
    analogWrite(Rpwm1, 0);
    analogWrite(Lpwm1, mspeed*0.4);
    analogWrite(Rpwm2, 0);
    analogWrite(Lpwm2, mspeed);
    robotControlState=70;
    last_mspeed=mspeed;  
  }
}// void move_RightBackward(int mspeed)

void move_LeftBackward(int mspeed){ // robotControlState = 8
  if(robotControlState!=80 || last_mspeed!=mspeed){
    SetMotors(1);
    analogWrite(Rpwm1, 0);
    analogWrite(Lpwm1, mspeed);
    analogWrite(Rpwm2, 0);
    analogWrite(Lpwm2, mspeed*0.4);
    robotControlState=80; 
    last_mspeed=mspeed; 
  }
}// void move_LeftBackward(int mspeed)

void stopRobot(int delay_ms){
  SetMotors(2);
  analogWrite(Rpwm1, 0);
  analogWrite(Lpwm1, 0);
  analogWrite(Rpwm2, 0);
  analogWrite(Lpwm2, 0);
  delay(delay_ms);
}// void stopRobot(int delay_ms)

void SetPWM(const long pwm_num, byte pwm_channel){
  if(pwm_channel==1){ // DRIVE MOTOR
    analogWrite(Rpwm1, 0);
    analogWrite(Rpwm2, 0);
    analogWrite(Lpwm1, pwm_num);
    analogWrite(Lpwm2, pwm_num);
    pwmRvalue = pwm_num;
  }
  else if(pwm_channel==2){ // STEERING MOTOR
    analogWrite(Lpwm1, 0);
    analogWrite(Lpwm2, 0);
    analogWrite(Rpwm1, pwm_num);
    analogWrite(Rpwm2, pwm_num);
    pwmLvalue = pwm_num;
  }
}// void SetPWM (const long pwm_num, byte pwm_channel)  

void SetMotors(int controlCase){
  switch(controlCase){
    case 1:
      digitalWrite(MotorRight_R_EN, HIGH);  
      digitalWrite(MotorRight_L_EN, HIGH); 
      digitalWrite(MotorLeft_R_EN, HIGH);  
      digitalWrite(MotorLeft_L_EN, HIGH); 
    break;
    case 2:
      digitalWrite(MotorRight_R_EN, LOW);  
      digitalWrite(MotorRight_L_EN, LOW); 
      digitalWrite(MotorLeft_R_EN, LOW);  
      digitalWrite(MotorLeft_L_EN, LOW); 
    break;
  } 
}// void SetMotors(int controlCase)
