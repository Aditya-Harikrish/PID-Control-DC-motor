#include "analogWrite.h"
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;
int motor1Pin1 = 27; 
int motor1Pin2 = 26; 
int enable1Pin = 14; 
int encoderA = 25;
int encoderB = 33;

volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
float eprev = 0;
float eintegral = 0;


void setup() {
  // sets the pins as outputs:
  analogWriteResolution(enable1Pin, 8);
  pinMode(encoderA,INPUT);
  pinMode(encoderB,INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderA),readEncoder,RISING);
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
//  pinMode(enable1Pin, OUTPUT);


  Serial.begin(115200);

  // testing
  Serial.print("Testing DC Motor...");
  Serial.println("target pos");
}



void loop() {

  // set target position
  //int target = 1200;
  int target = 150;

  // PID constants
  float kp = 1;
  float kd = 0.025;
  float ki = 0.0;

//  time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

//  // Read the position in an atomic block to avoid a potential
//  // misread if the interrupt coincides with this code running
//  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  int pos = 0; 
  noInterrupts();  
    pos = posi;
  interrupts();
//  
  // error
  int e = pos - target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr,enable1Pin,motor1Pin1,motor1Pin2);


  // store previous error
  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
//
//  analogWrite(14,pwmVal);
  analogWrite(pwm,pwmVal);
  ledcWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void readEncoder(){
  int b = digitalRead(encoderB);
//  Serial.println(b);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}
