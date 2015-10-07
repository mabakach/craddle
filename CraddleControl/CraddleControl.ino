/**
 * Baby craddle drive control sketch.
 * 
 * Inspired by JamesKnopfSelbst and his Youtube video https://www.youtube.com/watch?v=ktmIXVn6ftc
 * 
 * Required hardware:
 * 1) Arduino motor shield: https://www.arduino.cc/en/pmwiki.php?n=Main/ArduinoMotorShieldR3
 * 2) Any potentiometer
 * 
 * 
 * The MIT License (MIT)
 * 
 * Copyright (c) 2015 Marc Baumgartner
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

// Librarys for formated output 
#include <stdarg.h>

// Motor Values
static int MAX_CURRENT_MILI_AMPERE = 2000;
int motor1DirectionPin = 12;
int motor1BreakPin = 9;
int motor1SpeedPin = 3;
int motor1CurrentSensingPin = 0;
int motor1DirectionLevel = LOW;

// Potentiometer
int potiPin = 4;

// Direction and speed calculation
// Actual speed of the motor
int actualSpeed = 0;

// Theoretically needed max speed
const float THEORETICAL_MAX_SPEED = 300.0f;

// Max speed with the used motorshield and motor.
const int MOTOR_MAX_SPEED = 255;
const int MOTOR_MIN_SPEED = 50;

const int DIRECTION_UP = 1;
const int DIRECTION_DOWN = -1;

// +1 => up, 0 0=> stopped, -1 => down
int actualDirection = 0;

// +1 => up, 0 0=> stopped, -1 => down
int lastDirection = 0;

// raw angle value offset at which a direction change should be made.
const int RAW_ANGLE_AMPLITUDE = 25;

// Lower limit for the raw angle value => If raw value is lower change direction
float lowerRawAngleLimit = -1.0f;

// Upper limit for the raw angle value => If raw value is higher change direction
float upperRawAngleLimit = -1.0f;

// Center between upperRawAngleLimit and lowerRawAngleLimit
float centerRawAngle = 0.0f;

//  Current raw angle value (between 0 and 1023). -1  means not initialized.
int currentRawAngle = -1;

//  Last raw angle value (between 0 and 1023). -1  means not initialized.
int lastRawAngle = -1;

const int MAX_DURATION_WITHOUT_DIRECTION_CHANGE_MILIS = 2000;

const int NEUTRAL_RUN_WAIT_AT_DIRECTION_CHANGE_MILIS = 200;

// "Timestamp" of the last direction change
long milisLastDirectionChange = 0;
 
void setup() {
  centerRawAngle = analogRead(potiPin);
  lowerRawAngleLimit = centerRawAngle - RAW_ANGLE_AMPLITUDE;
  upperRawAngleLimit = centerRawAngle + RAW_ANGLE_AMPLITUDE;
  updateAngleAndDirection();
  initMotor();
  Serial.begin(9600);
}

void initMotor(){
  // declare all pins as output
  pinMode(motor1DirectionPin, OUTPUT);
  pinMode(motor1BreakPin, OUTPUT);
  pinMode(motor1SpeedPin, OUTPUT);

  // set direction
  digitalWrite(motor1DirectionPin, motor1DirectionLevel);

  // disengage break
  digitalWrite(motor1BreakPin, LOW);
}

void printSerial(char *fmt, ... ){
  char buf[128]; // resulting string limited to 128 chars
  va_list args;
  va_start (args, fmt );
  vsnprintf(buf, 128, fmt, args);
  va_end (args);
  Serial.print(buf);
}

void loop() {
  updateAngleAndDirection();
  updateSpeed();
  char* directionStr;
  if (actualDirection == DIRECTION_UP){
     motor1DirectionLevel = LOW;
     directionStr = "Right";
  } else {
    motor1DirectionLevel = HIGH;
    directionStr = "Left";
  }
  digitalWrite(motor1DirectionPin, motor1DirectionLevel);
  analogWrite(motor1SpeedPin, actualSpeed);

  printSerial("Direction: %i Speed: %d Angle: %i Time since last direction change: %d\n", actualDirection, actualSpeed, currentRawAngle);

  // Take a short break if direction changes ==> motor should run in neutral for smoother direction changes.
  if (actualDirection != lastDirection){
    delay(NEUTRAL_RUN_WAIT_AT_DIRECTION_CHANGE_MILIS);
  }
  
}

void updateAngleAndDirection(){
  lastDirection = actualDirection;
  currentRawAngle = analogRead(potiPin);
  if (currentRawAngle > upperRawAngleLimit){
    actualDirection = DIRECTION_DOWN;
  } else if (currentRawAngle < lowerRawAngleLimit){
    actualDirection = DIRECTION_UP;
  } else if (actualDirection == 0){
    if (currentRawAngle < centerRawAngle){
      actualDirection = DIRECTION_UP;
    } else {
      actualDirection = DIRECTION_DOWN;
    }
  }
  if (actualDirection != lastDirection){
    milisLastDirectionChange = millis();
  }
}

void updateSpeed(){
  if (millis() - milisLastDirectionChange < MAX_DURATION_WITHOUT_DIRECTION_CHANGE_MILIS){
    // if we have a change of direction take a short break => no motor power
    if (actualDirection != lastDirection){
      actualSpeed = 0;
    } else {
      if (currentRawAngle <= centerRawAngle){
        actualSpeed =(int) (currentRawAngle - lowerRawAngleLimit) * (THEORETICAL_MAX_SPEED / (centerRawAngle - lowerRawAngleLimit));
      } else {
        actualSpeed =(int) (currentRawAngle - centerRawAngle) * ((-1 *THEORETICAL_MAX_SPEED) / (upperRawAngleLimit - centerRawAngle)) + THEORETICAL_MAX_SPEED;
      }
      
      // between 0 and MOTOR_MIN_SPEED the motor is to week => boost it up
      if (actualSpeed < MOTOR_MIN_SPEED){
        actualSpeed = MOTOR_MIN_SPEED;
      }
  
      // calculation assumes a theoretical maximum motor power of THEORETICAL_MAX_SPEED -> if speed it to high limit it to MOTOR_MAX_SPEED
      if (actualSpeed > MOTOR_MAX_SPEED){
        actualSpeed = MOTOR_MAX_SPEED;
      } else if (actualSpeed < 0){
        actualSpeed = 0;
      }
    }
  } else {
    actualSpeed = 0;
  }
}

int readCurrent(){
  int currentRaw = analogRead(motor1CurrentSensingPin);
  Serial.println(currentRaw);
  return (int) MAX_CURRENT_MILI_AMPERE / 1024.0f * currentRaw ;
}

