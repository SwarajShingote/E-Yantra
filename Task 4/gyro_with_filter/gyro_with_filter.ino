#include <Wire.h>
#include "config.h"
#include "imu.h"
#include "motion.h"

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);  //established communication with xbee @9600 baud on Serial 2 port
  //initialize motors, buzzers and magnets in off state
  Serial.println("beginning");
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(DIRA1, OUTPUT);
  pinMode(DIRA2, OUTPUT);
  pinMode(DIRB1, OUTPUT);
  pinMode(DIRB2, OUTPUT);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin , LOW);
  //  pinMode(buzzer, OUTPUT);
  //  pinMode(magnet, OUTPUT);

  //  digitalWrite(buzzer , HIGH);
  //  digitalWrite(magnet , LOW);

  //  pinMode(encoderPinL1, INPUT);
  //  pinMode(encoderPinL2, INPUT);
  //
  //  digitalWrite(encoderPinL1, HIGH); //turn pullup resistor on
  //  digitalWrite(encoderPinL2, HIGH); //turn pullup resistor on
  //
  //  attachInterrupt(4, updateEncoderL, CHANGE);
  //  attachInterrupt(5, updateEncoderL, CHANGE);

  pinMode(encoderPinR1, INPUT);
  pinMode(encoderPinR2, INPUT);

  digitalWrite(encoderPinR1, HIGH); //turn pullup resistor on
  digitalWrite(encoderPinR2, HIGH); //turn pullup resistor on

  attachInterrupt(0, updateEncoderR, CHANGE);
  attachInterrupt(1, updateEncoderR, CHANGE);

  Stop();
  MPU_Init();
  delay(1000);
  //  loopStart = micros();




  //  right();
  loopStart = micros();
}
void loop() {
  newPos = encoderValueR;
  dist = encoderValueR * tick2dist / 100;
  newTime = millis();
  vel = ((newPos - oldPos) * tick2dist * 10) / (newTime - oldTime);
  //  Serial.print("new:  "); Serial.print(newPos);
  //  Serial.print(" old:  "); Serial.print(oldPos);
  //  Serial.print(" Time:  "); Serial.print(newTime - oldTime);
  oldPos = newPos;
  oldTime = newTime;
  read_accel();
  read_gyro();
  comp_filter_pitch();
  comp_filter_roll();
  theta = pitch;
  theta_dot = (gx + gyroOff);
  xbee_data();
  float tiltOut = ((dist * K[0]) +  (vel * K[1]) + ((theta + theta_offSet) * K[2]) + (theta_dot * K[3]));
  torque = tiltOut * 255 / cutOff;
  if (abs(theta) < cutOffAng * deg2rad) {
    if (torque > 0)
      backward(abs(torque));
    else
      forward(abs(torque));
  }
  else
    Stop();
  //  Serial.print(" dist: "); Serial.print(dist);
  //  Serial.print("  vel: ");
  //  Serial.println(vel);
  //  Serial.print(" theta:  ");
  //  Serial.println( theta_offSet);
  //  Serial.print(" theta_dot:  "); Serial.print(theta_dot);
  //  Serial.print(" tiltOut:  ");  Serial.print(tiltOut);
  //  Serial.print(" torque:  "); Serial.print(torque);
  //  Serial.println("");
  while (micros() - loopStart < 10000);
  loopStart = micros();
}
