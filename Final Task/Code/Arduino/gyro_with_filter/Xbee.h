/*
  Team Id: #BP2155
  Author List: Aayush Singh,Mrunal Joshi,Swaraj Shingote,Datta Dhore
  Filename: Xbee.h
  Theme: Biped Patrol
  Functions: readData(), xbee_data()
  Global Variables: NONE
*/

/*
   Function Name: readData()
   Input: serial data from xbee
   Output: boolean indicating valid data
   Logic: reading serial data from xbee and calculating joystick values and detecting button presses
   Example Call: readData();
*/
bool readData(void) {
  //  Serial.println("reading");
  int tempData[5];   //temporary array to store relevant bytes of received data
  if (Serial2.available() > 18) {  //wait for serial buffer to receive 18 bytes of data

    if (Serial2.read() == 0x7E) {  //start delimitter of every API frame
      //      Serial.print("  yo");
      for (ui i = 0; i < 11; i++)  //ignore starting 12 bytes of data
        Serial2.read();
      for (ui i = 0; i < 5; i++)  //read button state & joystick values which is 5 bytes of data
        tempData[i] = Serial2.read();
      Serial2.read(); //empty the buffer by reading last byte of data

      //merge the HIGH and LOW bytes of 10 bit ADC values of joystick
      potX = (tempData[1] << 8) | tempData[2];           //joystick in x-direction
      potY = (tempData[3] << 8) | tempData[4];           //joystick in y-direction
      button = tempData[0];    //Button pressed
      return 1;
    }
    return 0;  //if data read succesfully
  }
  return 0;  //to avoid misbehaviour of bot if data is not available
}

/*
   Function Name: xbee_data()
   Input: processed serial data of containing joystick and button states
   Output: Set cutoff values to drive bot forward/backward with varying speed
   Logic: using joystick values traverse the bot in specified direction and button to turn on LEDs,Buzzer and switching to bridge mode traversal
   Example Call: xbee_data();
*/

void xbee_data() {
  //read data from xbee
  if (readData()) {
    if (potY == 1023 && (potX > 500 && potX < 550) ) { //joystick toggled vertically upwards i.e bot traverses forward
      theta_offSet = -3.5 * deg2rad;  //Add -3.5° offset in angular position
      FLeftOff = 80;      //Increase motor speeed in forward direction by 80
      FRightOff = 80;
      encoderValueR = 0;  //Reset encoder count to avoid shakiness after moving
      oldPos = 0;         //reset old position to avoid garbage in velocity
//      Serial.println(1);
    }
    else if (potY < 10 && (potX > 500 && potX < 550)) { //joystick toggled vertically downwards i.e bot traverses backward
      theta_offSet = 3.5 * deg2rad; //Add 3.5° offset in angular position
      FLeftOff = 80;
      FRightOff = 80;
      encoderValueR = 0;
      oldPos = 0;
//      Serial.println(2);
    }
    else if (potX == 1023 && (potY > 500 && potY < 550)) {  //joystick toggled horizontally rightwards bot moves rightwards by rotating left wheel
//      Serial.println(-1);
      theta_offSet = -2.5;  //Add -2.5° offset in angular position
      FLeftOff = 50;   //Increase left motor speeed in forward direction by 50
      FRightOff = 0;
      encoderValueR = 0;
      oldPos = 0;
    }
    else if (potX < 10 && (potY > 500 && potY < 550)) {  //joystick toggled horizontally leftwards bot moves leftwards by rotating right wheel
//      Serial.println(-2);
      theta_offSet = 2.5;  //Add 2.5° offset in angular position
      encoderValueR = 0;
      BLeftOff = 50;  //Increase left motor speeed in backward direction by 50
      BRightOff = 0;
      oldPos = 0;
    }

    else { //stop the motors if joystick centred and reset all the offsets
      theta_offSet = 0;
      FLeftOff = 0;
      FRightOff = 0;
      BLeftOff = 0;
      BRightOff = 0;
      //Serial.println(0);
    }


    //Serial.println(button);
    if (button == 32) {  //if triangle button pressed
      RGB_color(1, 0, 0, 1);  //red indicator and buzzer while climbing bridge
      bridgeBoost = 50;  //boost speed to climb bridge
    }
    else if ( button == 16) {  //if square button pressed
      RGB_color(0, 1, 0, 1);  //green indicator and buzzer
    }
    else if ( button == 4) {  //if circle button pressed
      digitalWrite(magnetFront , HIGH);  //Enable front electromagnet
    }
    else if ( button == 8) {  //if cross button pressed
      digitalWrite(magnetBack , HIGH);  //Enable back electromagnet
    }
    else {  //turn off the buzzer and magnet and led's
      RGB_color(0, 0, 0, 0);
      bridgeBoost = 0;  //reset bridge boost
      digitalWrite(magnetFront , LOW);
      digitalWrite(magnetBack , LOW);
    }
  }
}
