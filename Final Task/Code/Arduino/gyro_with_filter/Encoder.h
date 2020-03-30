/*
  Team Id: #BP2155
  Author List: Aayush Singh,Mrunal Joshi,Swaraj Shingote,Datta Dhore
  Filename: Encoder.h
  Theme: Biped Patrol
  Functions: init_encoder(), updateEncoderL(), updateEncoderR()
  Global Variables: NONE
*/

/*
   Function Name: init_encoder()
   Input: NONE
   Output: NONE
   Logic: initialize and enable interrupt for both pins of right encoder motor
          On interrupt updateEncoderR() will be called as ISR
   Example Call: init_encoder()
*/
void init_encoder() {
  pinMode(encoderPinR1, INPUT);
  pinMode(encoderPinR2, INPUT);
  
  //enable pullup resistor on interrupt pin of encoder
  digitalWrite(encoderPinR1, HIGH); //turn pullup resistor on
  digitalWrite(encoderPinR2, HIGH); //turn pullup resistor on

  //connect interrupt to ISR triggered at level change
  attachInterrupt(0, updateEncoderR, CHANGE);
  attachInterrupt(1, updateEncoderR, CHANGE);
}


/*
   Function Name: updateEncoderL()
   Input: NONE
   Output: NONE
   Logic: Reading 2 phase from encoder and incrementing count based on phase difference
   Example Call: called when external interrupt occurs due to left encoder
*/

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


/*
   Function Name: updateEncoderR()
   Input: NONE
   Output: NONE
   Logic: Reading 2 phase from encoder and incrementing count based on phase difference
   Example Call: called when external interrupt occurs due to right encoder
*/
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
