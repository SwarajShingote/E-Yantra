/*
  Team Id: #BP2155
  Author List: Aayush Singh,Mrunal Joshi,Swaraj Shingote,Datta Dhore
  Filename: motion.h
  Theme: Biped Patrol
  Functions: init_motors(), forward(), backward(), right(), left(), stop()
  Global Variables: NONE
*/


/*
   Function Name: init_motors()
   Input: NONE
   Output: NONE
   Logic: Declare all motor pins as OUTPUT pins
   Example Call: init_motors();
*/
void init_motors(void) {
  pinMode(ENA, OUTPUT);    //enable pin for right motor
  pinMode(ENB, OUTPUT);    //enable pin for left motor
  pinMode(DIRA1, OUTPUT);  //direction pins for right motor
  pinMode(DIRA2, OUTPUT);  //direction pins for right motor
  pinMode(DIRB1, OUTPUT);  //direction pins for left motor
  pinMode(DIRB2, OUTPUT);  //direction pins for left motor
}



/*
   Function Name: forward()
   Input: torque
   Output: NONE
   Logic: Applying constrain to torque obtained from LQR to rotate motor in forward direction
   Example Call: forward(50);
*/
void forward(unsigned int torque) {
  digitalWrite(DIRA1, HIGH);
  digitalWrite(DIRA2, LOW);
  digitalWrite(DIRB1, HIGH);
  digitalWrite(DIRB2, LOW);

  analogWrite(ENA, constrain(torque + rightMotorSpeed + FRightOff + bridgeBoost, 0, 255)); //vary PWM as per conditions
  analogWrite(ENB, constrain(torque + leftMotorSpeed + FLeftOff + bridgeBoost , 0, 255)); // 
}

/*
   Function Name: backward()
   Input: torque
   Output: NONE
   Logic: Applying constrain to torque obtained from LQR to rotate motor in backward direction
   Example Call: backward(50);
*/

void backward(unsigned int torque) {
  digitalWrite(DIRA1, LOW);
  digitalWrite(DIRA2, HIGH);
  digitalWrite(DIRB1, LOW);
  digitalWrite(DIRB2, HIGH);

  analogWrite(ENA, constrain(torque + rightMotorSpeed + BRightOff , 0, 255));
  analogWrite(ENB, constrain(torque + leftMotorSpeed + BLeftOff, 0, 255));
}

/*
   Function Name: right()
   Input: torque
   Output: NONE
   Logic: feeding torque obtained from LQR to rotate motor in right direction
   Example Call: right();
*/

void right() {
  digitalWrite(DIRA1, LOW);
  digitalWrite(DIRA2, HIGH);
  digitalWrite(DIRB1, HIGH);
  digitalWrite(DIRB2, LOW);
  analogWrite(ENA, 60);
  analogWrite(ENB, 0);
}
/*
   Function Name: left()
   Input: torque
   Output: NONE
   Logic: feeding torque obtained from LQR to rotate motor in left direction
   Example Call: left();
*/

void left() {
  digitalWrite(DIRA1, HIGH);
  digitalWrite(DIRA2, LOW);
  digitalWrite(DIRB1, LOW);
  digitalWrite(DIRB2, HIGH);
  analogWrite(ENA, 0);
  analogWrite(ENB, 200);
}
/*
   Function Name: stop()
   Input: NONE
   Output: NONE
   Logic: To stop the motor
   Example Call: stop();
*/

void Stop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
