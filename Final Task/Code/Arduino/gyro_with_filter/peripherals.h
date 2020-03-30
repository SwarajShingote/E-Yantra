/*
  Team Id: #BP2155
  Author List: Aayush Singh,Mrunal Joshi,Swaraj Shingote,Datta Dhore
  Filename: peripherals.h
  Theme: Biped Patrol
  Functions: init_led_buzz(),init_magnets(), RGB_color()
  Global Variables: NONE
*/


/*
   Function Name: init_led_buzz()
   Input: NONE
   Output: NONE
   Logic: declare led's as output and Set a digital pin to Always HIGH logic level(not recommended)
   Example Call: init_led_buzz();
*/
void init_led_buzz(void) {
  pinMode(Vcc, OUTPUT);   //virtual Vcc created on digital arduino pin(not recommended)
  pinMode(red, OUTPUT); //set red led as OUTPUT
  pinMode(green, OUTPUT); //set green led as OUTPUT
  pinMode(blue, OUTPUT); //set blue led as OUTPUT
  pinMode(buzzer, OUTPUT); //set buzzer as OUTPUT
  digitalWrite(Vcc , HIGH);  // 5v at digital pin
  RGB_color(0, 0, 0, 0);  //turn off led's and buzzer
}




/*
  Function Name: init_magnets()
  Input: NONE
  Output: NONE
  Logic: declare electromagnet as output and Disable them at beginning
  Example Call: init_magnets();
*/
void init_magnets(void) {

  pinMode(magnetFront, OUTPUT);
  pinMode(magnetBack, OUTPUT);

  digitalWrite(magnetFront , LOW); //turn off front electromagnet
  digitalWrite(magnetBack , LOW); //turn off back electromagnet
}

/*
  Function Name: RGB_color()
  Input: (R,G,B,Buzz)  -->All booleans
        booleans indicating required state of RGB led and buzzer
  Output: turn ON/OFF led's and buzzer
  Logic: both the led's and buzzer are Active Low, hence inverting the states to produce desired result
  Example Call: RGB_color(1,0,0,1); --> turns ON red led and buzzzer
*/
void RGB_color(bool red_light_value, bool green_light_value, bool blue_light_value, bool buzz)
{
  digitalWrite(red, !red_light_value); //turn ON/OFF red led
  digitalWrite(green, !green_light_value); //turn ON/OFF green led
  digitalWrite(blue, !blue_light_value); //turn ON/OFF blue led
  digitalWrite(buzzer, !buzz); //turn ON/OFF buzzer
}
