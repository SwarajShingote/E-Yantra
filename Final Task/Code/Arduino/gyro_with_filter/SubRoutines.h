/*
  Team Id: #BP2155
  Author List: Aayush Singh,Mrunal Joshi,Swaraj Shingote,Datta Dhore
  Filename: SubRoutines.h
  Theme: Biped Patrol
  Functions: init_timer5(void), ISR(TIMER5_COMPA_vect), init_timer4(void), ISR(TIMER4_COMPA_vect),
             init_timer3(), start_timer3(), ISR(TIMER3_OVF_vect), epoch()             
  Global Variables: NONE
*/

/*
   Function Name: init_timer3()
   Input: NONE
   Output: TIMER3 Initialize - Prescaler: 1024
           WGM: 0 Normal, TOP=0xFFFF, BOTTOM=0xFD30
   Logic: initializing timer 3 as counter
   Example Call: init_timer3();
*/
void init_timer3()
{
  TCCR3B = 0x00;    // Stop Timer
  TCNT3  = 0xC667;  // 0.0009999593097s (~0.001s)
  OCR3A  = 0x0000;  // Output Compare Register (OCR) - Not used
  OCR3B  = 0x0000;  // Output Compare Register (OCR) - Not used
  OCR3C  = 0x0000;  // Output Compare Register (OCR) - Not used
  ICR3   = 0x0000;  // Input Capture Register (ICR)  - Not used
  TCCR3A = 0x00;
  TCCR3C = 0x00;
}

/*
   Function Name: start_timer3()
   Input: NONE
   Output: start timer 3
   Logic: None
   Example Call: start_timer3();
*/
void start_timer3()
{
  TCCR3B = 0x01;    // Prescaler None 0-0-1
  TIMSK3 = 0x01;    // Enable Timer Overflow Interrupt
}

/*
   Function Name: ISR(TIMER3_OVF_vect)
   Input: Timer 3 overflow vector
   Output: Time in millisecond
   Logic: None
   Example Call: called automatically
*/
ISR(TIMER3_OVF_vect)
{
  TCNT3 = 0xC667;   // Reload counter value
  time_ms++;      // Increment ms value

  if (time_ms >= 1000)
  {
    time_ms = 0;  // Reset ms value
    time_sec++;   // Increments seconds value
  }
}


/*
   Function Name: epoch()
   Input: None
   Output: Time in millisecond
   Logic: Similar to millis() but due to implementation of other timers millis gets stalled
   Example Call: epoch()
*/
unsigned long epoch()
{
  unsigned long elapsed_time;
  elapsed_time = time_sec * 1000 + time_ms;
  return elapsed_time;
}



/*
   Function Name: init_timer4()
   Input: NONE
   Output: timer starts counting for 10 ms
   Logic: initializing timer 4 in CTC mode for 10 ms
   Example Call: init_timer4();
*/
void init_timer4(void) {
  noInterrupts();           // disable all interrupts
  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4  = 0;
  OCR4A = 624;            // compare match register 16MHz/256/2Hz
  TCCR4B |= (1 << WGM42);   // CTC mode
  TCCR4B |= (1 << CS42);    // 256 prescaler
  TIMSK4 |= (1 << OCIE4A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts
}

/*
   Function Name: ISR(TIMER4_COMPA_vect)
   Input: NONE
   Output: motor speed and direction is set
   Logic: Calculating distance, velocity, angle, angular velocity and calculating the input to the motors using LQR every 10 ms
   Example Call: ISR called after every 10 ms.
*/

ISR(TIMER4_COMPA_vect)     // timer compare interrupt service routine
{
  newPos = encoderValueR; //inital position 
  dist = encoderValueR * tick2dist / 100;  //distance travelled in metres
  
  vel = ((newPos - oldPos) * tick2dist * 10) / 10;  //velocity = (finalPos - initalPos)/ change in Time. in m/s
  oldPos = newPos;  //store old pos to be used in next iteration
  
  theta = pitch - thetaOff;   //angular position
  theta_dot = (gx + gyroOff); //angular velocity
  
  tiltOut = ((dist * K[0]) +  (vel * K[1]) + ((theta + theta_offSet) * K[2]) + (theta_dot * K[3])); //LQR output k1 ∗ x + k2 ∗ x_dot + k3 ∗ Θ + k4 ∗ Θ_dot
  torque = tiltOut * torque2pwm;  //converting to torque to PWM
  
  if (abs(theta) < cutOffAng * deg2rad) {  //If bot in state of stabilization
    if (torque > 0)  //move backward to stabilize
      backward(abs(torque));
    else  //move forward to stabilize
      forward(abs(torque));
  }
  else  //stop if motors cannot stabilize the bot
    Stop();
}

/*
   Function Name: init_timer5()
   Input: NONE
   Output: timer starts counting for 10 ms
   Logic: initializing timer 5 in CTC mode for 10 ms
   Example Call: init_timer5();
*/
void init_timer5(void) {
  noInterrupts();           // disable all interrupts
  TCCR5A = 0;
  TCCR5B = 0;
  TCNT5  = 0;
  OCR5A = 625;            // compare match register 16MHz/256/2Hz
  TCCR5B |= (1 << WGM52);   // CTC mode
  TCCR5B |= (1 << CS52);    // 256 prescaler
  TIMSK5 |= (1 << OCIE5A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts
}

/*
   Function Name: ISR(TIMER5_COMPA_vect)
   Input: NONE
   Output: global values of theta and theta_dot are updated
   Logic: Reading accelerometer and gyroscope values and applying complementry filter in every 10 ms
   Example Call: ISR called after every 10 ms.
*/

ISR(TIMER5_COMPA_vect)     // timer compare interrupt service routine
{
  sei();  //enable interrupts for communicating with MPU
  read_accel();  //read raw values from accelerometer
  read_gyro();   //read raw values from gyroscope
  comp_filter_pitch(lpf_y, lpf_z, -hpf_x);  //apply complimentary filter on Low Pass and High Pass Values of acceleromter and gyroscope
}
