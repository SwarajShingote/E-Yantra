/*
  Team Id: #BP2155
  Author List: Aayush Singh,Mrunal Joshi,Swaraj Shingote,Datta Dhore
  Filename: gyro_with_filter
  Theme:Biped patrol
  Files: config.h, Encoder.h, Xbee.h, motion.h, imu.h, SubRoutines.h, peripherals.h
  Functions: void setup(), void loop()
  Global Variables: NONE
*/

#include <Wire.h>
#include "config.h"
#include "Encoder.h"
#include "Xbee.h"
#include "motion.h"
#include "imu.h"
#include "SubRoutines.h"
#include "peripherals.h"
/*
   Function Name: setup()
   Input: NONE
   Output: NONE
   Logic: Initialize pinModes, serial communication, Timers, Encoder
   Example Call: Called by the controller automatically on startup
*/

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);  //established communication with xbee @9600 baud on Serial 2 port
  Serial.println("Initializing");

  //setup all output and input devices
  init_motors();   //setup motor pins
  init_led_buzz(); //setup RGB led and buzzer
  init_magnets();  //setup electromagnet modules
  init_encoder();  //setup right encoder motor as external input interrupt

  Stop();  //stop the motors to avoid unnecessary motion
  MPU_Init(); //Initialize MPU for reading data through i2c bus
  delay(1000);
  RGB_color(1, 0, 0, 1);  //turn ON red led and buzzer indicating MPU calibration
  thetaOff = calibrate_angle_offset();  //calculate offset in MPU angle
  delay(500);
//  timer3_init();
//  start_timer3();
  init_timer5();  //initialize and start Timer 5 for reading MPU values
  init_timer4();  //initialize and start Timer 4 for calculating and feeding PWM value to motors
}
/*
   Function Name: loop()
   Input: NONE
   Output: NONE
   Logic: Reading Data from Xbee
   Example Call: Bydefault called after setup function is executed
*/
void loop() {

  xbee_data(); //read the serial data from the xbee module

  //  Serial.print(dist);
  //  Serial.print(" ");
  //  Serial.print(vel);
  //  Serial.print("  ");
  //  Serial.print(theta);
  //  Serial.print(" ");
  //  Serial.println(theta_dot);
  //  Serial.print("t ");
  //  Serial.println(torque);
  //  Serial.print(" ");

}
