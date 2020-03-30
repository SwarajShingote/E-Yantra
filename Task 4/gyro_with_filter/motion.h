void updateEncoderL() {

  int MSB = digitalRead(encoderPinL1); //MSB = most significant bit
  int LSB = digitalRead(encoderPinL2); //LSB = least significant bit

  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum  = (lastEncodedL << 2) | encoded; //adding it to the previous encoded value

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValueL ++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValueL --;

  lastEncodedL = encoded; //store this value for next time
  //  Serial.println(encoderValueL);
}

void updateEncoderR() {

  int MSB = digitalRead(encoderPinR1); //MSB = most significant bit
  int LSB = digitalRead(encoderPinR2); //LSB = least significant bit

  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum  = (lastEncodedR << 2) | encoded; //adding it to the previous encoded value

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValueR ++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValueR --;

  lastEncodedR = encoded; //store this value for next time
  //  Serial.println(encoderValueR);
}

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
void xbee_data() {
  //read data from xbee
  if (readData()) {
//    Serial.println(potX); Serial.print(" ");
//    Serial.print(potY); Serial.print("a");
    if (potY < 10 && (potX > 500 && potX < 550)) { //joystick toggled vertically  downwards
      //      backward(200);
      theta_offSet = 2.5 * deg2rad;
      leftOff = 20;
      rightOff = 20;
      encoderValueR = 0;
      oldPos = 0;
//      Serial.println(1);
      //      Serial.print(" xbb"); Serial.print(potX); Serial.print(" ");Serial.print(potY);
    }
    else if (potY == 1023 && (potX > 500 && potX < 550) ) { //joystick toggled vertically upwards
      theta_offSet = -2.5 * deg2rad;
      leftOff = 110;
      rightOff = 110;
      encoderValueR = 0;
      oldPos = 0;
//      Serial.println(2);
      //      Serial.print(" xbu "); Serial.print(potX); Serial.print(" ");Serial.print(potY);
      //      forward(200);
    }
    else if (potX == 1023 && (potY > 500 && potY < 550)) {  //joystick toggled horizontally rightwards bot moves rightwards by rotating left wheel
//      Serial.println(-1);
      leftOff = 60;
      rightOff = 0;
      encoderValueR = 0;
      oldPos = 0;
    }
    else if (potX < 10 && (potY > 500 && potY < 550)) {  //joystick toggled horizontally leftwards bot moves leftwards by rotating right wheel
//      Serial.println(-2);
      leftOff = 0;
      rightOff = 50;
      encoderValueR = 0;
      oldPos = 0;
    }
    else { //stop the motors if joystick centred
//      Serial.println(0);
      theta_offSet = 0;
      leftOff = 0;
      rightOff = 0;
      //      Serial.print(" xbs "); Serial.print(potX); Serial.print(" ");Serial.print(potY);
      //      Stop();
    }
  }
}


void forward(unsigned int torque) {
  digitalWrite(DIRA1, HIGH);
  digitalWrite(DIRA2, LOW);
  digitalWrite(DIRB1, HIGH);
  digitalWrite(DIRB2, LOW);


  analogWrite(ENA, constrain(torque + 54  + rightOff, 0, 255));     //right motor   54
  analogWrite(ENB, constrain(torque + 96 + leftOff, 0, 255));    // 96
}
void backward(unsigned int torque) {
  digitalWrite(DIRA1, LOW);
  digitalWrite(DIRA2, HIGH);
  digitalWrite(DIRB1, LOW);
  digitalWrite(DIRB2, HIGH);
  analogWrite(ENA, constrain(torque + 54 , 0, 255));
  analogWrite(ENB, constrain(torque + 96 , 0, 255));
}
void right() {
  digitalWrite(DIRA1, LOW);
  digitalWrite(DIRA2, HIGH);
  digitalWrite(DIRB1, HIGH);
  digitalWrite(DIRB2, LOW);
  analogWrite(ENA, 60);
  analogWrite(ENB, 0);
}
void left() {
  digitalWrite(DIRA1, HIGH);
  digitalWrite(DIRA2, LOW);
  digitalWrite(DIRB1, LOW);
  digitalWrite(DIRB2, HIGH);
  analogWrite(ENA, 0);
  analogWrite(ENB, 200);
}
void Stop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void calibrateOffset() {
  Serial.println("Calibrating...");
  while (!k) {
    newPos = encoderValueR;
    dist = encoderValueR * tick2dist / 100;
    newTime = millis();
    vel = ((newPos - oldPos) * tick2dist * 10) / (newTime - oldTime);
    oldPos = newPos;
    oldTime = newTime;
    read_accel();
    read_gyro();
    comp_filter_pitch();
    comp_filter_roll();
    theta = pitch;
    theta_dot = (gx + gyroOff);
    float tiltOut = ((dist * K[0]) +  (vel * K[1]) + (theta * K[2]) + (theta_dot * K[3]));
    if (!k && ((abs(floor(abs(theta)*rad2deg - threshAngle ))) < 1)) {
      cutOff = abs(tiltOut);
      k = 1;
      digitalWrite(ledPin , HIGH);
      Serial.print("cutOff:  "); Serial.println(cutOff);
    }

  }

}
