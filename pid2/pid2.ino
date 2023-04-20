#include "IRremote.h"
#include "IR.h"
#include <math.h>
#define RECEIVER 6
//yellow
const int encoder_pin1 = 2;
const int encoder_pin2 = 4;
const int encoder_pin3 = 3;
const int encoder_pin4 = 8;
//green
volatile int count1 = 0;
float position1 = 0.0;
bool lastA;
volatile int count2 = 0;
float position2 = 0.0;
bool lastB;
// Define the desired angle
 float desiredAngle1 = 360;
  float desiredAngle2 = 360;
// Define the PID constants
const float Kp = 1;
const float Ki = 0.2;
const float Kd = 0.0;
const float maxOutput = 100;
const float integralMax = 40.0;
const float integralMin = -40.0;
// Define the motor control pins
const int motorPin1 = 7;
const int motorPin2 = 6;
const int motorPin3 = 12;
const int motorPin4 = 13;
const int motorSpeedPin1 = 9;
const int motorSpeedPin2 = 10;
// Create variables for the PID loop
float error1 = 0.0;
float lastError1 = 0.0;
float integral1 = 0.0;
float derivative1 = 0.0;
float output1 = 0.0;
float lastOutput1 = 0.0;
float error2 = 0.0;
float lastError2 = 0.0;
float integral2 = 0.0;
float derivative2 = 0.0;
float output2 = 0.0;
float lastOutput2 = 0.0;
// Define the deadband
const float deadband = 5.0;
IRrecv irrecv(RECEIVER); // create instance of 'irrecv'
decode_results results; // create instance of 'decode_results'
void setup() {
  pinMode(encoder_pin1, INPUT_PULLUP);
  pinMode(encoder_pin2, INPUT_PULLUP);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorSpeedPin1, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoder_pin1), encoder_callback, CHANGE);
  lastA = digitalRead(encoder_pin1);
  pinMode(encoder_pin3, INPUT_PULLUP);
  pinMode(encoder_pin4, INPUT_PULLUP);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  pinMode(motorSpeedPin2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoder_pin3), encoder_callback2, CHANGE);
  lastB = digitalRead(encoder_pin3);
  Serial.begin(9600);
}
static String message = "";
void loop() {
 while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { // if we encounter the delimiter
      if (message.startsWith("joystick:")) { // check if the message is a joystick message
        message.remove(0, 9); // remove the "joystick:" prefix
        int comma_index = message.indexOf(',');
        String x_str = message.substring(0, comma_index);
        String y_str = message.substring(comma_index + 1);
        int x = x_str.toInt();
        int y = y_str.toInt();
        // do something with x and y
        double target = atan2(y, x)* 180.0 / M_PI;
        if(target<0) target = 360+target;
        changeTarget(target, target);
        Serial.println(y);
        Serial.println(atan2(y, x)* 180.0 / M_PI);
      }
      else if(message.startsWith("right_trigger:")){
        //message.remove(0, 14);
        //int depression = message.toInt();
        //Serial.println("THE RIGHT DEPRISSION IS: %d", depression);
      }
      else if(message.startsWith("left_trigger:")){
        //message.remove(0, 14);
        //int depression = message.toInt();
        //Serial.println("THE LEFT DEPRISSION IS: %d", depression);
      }
      message = ""; // clear the accumulated message
    } else {
      message += c; // accumulate the message
    }
  }
  // Calculate the current error
  error1 = desiredAngle1 - position1;
  error2 = desiredAngle2 - position2;
  //Preventing divide by zero errors
  if(desiredAngle1==0) desiredAngle1 = 1;
  if(desiredAngle2==0) desiredAngle2 = 1;
  // Calculate the proportional term
  float proportional1 = Kp * 200 * error1/desiredAngle1;
  float proportional2 = Kp * 200 * error2/desiredAngle2;
  // Calculate the integral1 term
  integral1 += Ki * 70 * error1/desiredAngle1;
  integral2 += Ki * 70 * error2/desiredAngle2;
  //Limiting size of integral Value
  if (integral1 > integralMax) integral1 = integralMax;
  else if (integral1 < integralMin) integral1 = integralMin;
  if (integral2 > integralMax) integral2 = integralMax;
  else if (integral2 < integralMin) integral2 = integralMin;
  // Calculate the derivative term (Currenlty not used)
  derivative1 = Kd * (error1 - lastError1);
  derivative2 = Kd * (error2 - lastError2);
  // Calculate the output of the PID controller
  output1 = proportional1 + integral1 + derivative1;
  output2 = proportional2 + integral2 + derivative2;
  // Apply the deadband to the output
  if (abs(error1) < deadband) output1 = 0.0;
  if (abs(error2) < deadband) output2 = 0.0;
  //Applying Anti wind up
  float deltaOutput1 = output1 - lastOutput1;
  if (deltaOutput1 > 10.0 || deltaOutput1 < -10.0) {
    integral1 -= Ki * deltaOutput1;
    output1 = lastOutput1 + deltaOutput1;
  }
  float deltaOutput2 = output2 - lastOutput2;
  if (deltaOutput2 > 10.0 || deltaOutput2 < -10.0) {
    integral2 -= Ki * deltaOutput2;
    output2 = lastOutput2 + deltaOutput2;
  }
  //Setting absolute maximum values of motor speeds
  if(output1>maxOutput) output1 = maxOutput;
  if(output1<-maxOutput) output1 = -maxOutput;
  if(output2>maxOutput) output2 = maxOutput;
  if(output2<-maxOutput) output2 = -maxOutput;
  // Use the output value to control the motor speed via the motor controller
  if (output1 > 0) {
    if(lastError1<0){
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, LOW);
      //delay(50);
    }
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    analogWrite(motorSpeedPin1, output1);
  } else {
    if(lastError1>0){
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, LOW);
      //delay(50);
    }
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    analogWrite(motorSpeedPin1, -output1);
    //Serial.println("-------------------------Motor11111111111111111-------------------------");
  }
  if (output2 > 0) {
    if(lastError2<0){
      digitalWrite(motorPin3, LOW);
      digitalWrite(motorPin4, LOW);
      //delay(50);
    }
    digitalWrite(motorPin3, LOW);
    digitalWrite(motorPin4, HIGH);
    analogWrite(motorSpeedPin2, output2);
  } else {
    if(lastError2>0){
      digitalWrite(motorPin3, LOW);
      digitalWrite(motorPin4, LOW);
      //delay(50);
    }
    digitalWrite(motorPin3, HIGH);
    digitalWrite(motorPin4, LOW);
    analogWrite(motorSpeedPin2, -output2);
  //Serial.println("-------------------------Motor222222222222222222-------------------------");
}
  // Store the current error for the next loop iteration
  lastError1 = error1;
  lastOutput1 = output1;
  lastError2 = error2;
  lastOutput2 = output2;
  // Print the current motor position and the PID output for debugging purposes
  //-----------------------------------LOGGING----------------------------------------//
  Serial.print("Motor position: ");
  Serial.print(position2);
  Serial.print(" degrees, PID output: ");
  Serial.println(output2);
  Serial.print("Target is " );
  Serial.println(desiredAngle2);
  //-----------------------------------LOGGING----------------------------------------//
  // Wait for a short time before the next loop iteration
  delay(100);
}
void encoder_callback() {
  bool newA = digitalRead(encoder_pin1);
  bool newB = digitalRead(encoder_pin2);
  if (newA != lastA) {
    if (newB != lastA) {
      count1++;
    } else {
      count1--;
    }
    lastA = newA;
  }
  position1 = count1 * 0.270072993*9*13.89/82;
}
void encoder_callback2() {
  bool newA = digitalRead(encoder_pin3);
  bool newB = digitalRead(encoder_pin4);
  if (newA != lastB) {
    if (newB != lastB) {
      count2++;
    } else {
      count2--;
    }
    lastB = newA;
  }
  position2 = count2 * 0.270072993*9*13.89/82;
}
void changeTarget(int newTarget1, int newTarget2){
  desiredAngle1 = newTarget1;
  desiredAngle2 = newTarget2;
}