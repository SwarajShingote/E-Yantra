#define ENA     3    //enable pin for motor A
#define ENB     4    //enable pin for motor B
#define DIRA1   5    //direction pin 1 of motor A
#define DIRA2   6    //direction pin 2 of motor A
#define DIRB1   7    //direction pin 1 of motor B
#define DIRB2   8    //direction pin 2 of motor B
#define magnet  9    //Electromagnet pin
#define buzzer 10    //buzzer pin

typedef unsigned int ui;
int button, potX, potY;  //global variables to store processed values of API frame
bool readData(void);  //function to read API frames

void setup() {
  //declared motors buzzer and electromagnet pins as output
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(DIRA1, OUTPUT);
  pinMode(DIRA2, OUTPUT);
  pinMode(DIRB1, OUTPUT);
  pinMode(DIRB2, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(magnet, OUTPUT);

  Serial.begin(9600);
  
  Serial1.begin(9600);  //established communication with xbee @9600 baud on Serial 1 port
  //initialize motors, buzzers and magnets in off state
  Stop();
  digitalWrite(buzzer , HIGH);
  digitalWrite(magnet , LOW);
}

void loop() {
  //check if data is read and processed
  if (readData()) {
    Serial.print(button);   Serial.print("  ");
    Serial.print(potX);     Serial.print("  ");
    Serial.print(potY);     Serial.println("");

    if ((potX > 510 || potX < 545) && potY == 0) {  //joystick toggled vertically upwards  
      forward();
//      Serial.println("forward");
    }
    else if ((potX > 510 || potX < 545) && potY == 1023) {  //joystick toggled vertically downwards
      backward();
//      Serial.println("backward");
    }
    else if (potX == 1023 && (potY > 510 || potY < 545)) {  //joystick toggled horizontally rightwards
      right();
//      Serial.println("right");
    }
    else if (potX == 0 && (potY > 510 || potY < 545)) {  //joystick toggled horizontally leftwards
      left();
//      Serial.println("left");
    }
    else{  //stop the motors if joystick centred
      Stop();
//      Serial.println("motor off");
      }
      
    if (button == 4) {  //if higher button pressed, data received = 8
      digitalWrite(buzzer , LOW);
//      Serial.println("buzzer");
    }
    else if ( button == 8) {  //if lower button pressed, data received = 4
      digitalWrite(magnet , HIGH);
//      Serial.println("magnet");
    }
    else {  //turn off the buzzer and magnet
      digitalWrite(buzzer , HIGH);  //active LOW device
      digitalWrite(magnet , LOW);
//      Serial.println("off devices");
    }

  }

}



bool readData(void) {
  int tempData[5];   //temporary array to store relevant bytes of received data
  if (Serial1.available() > 18) {  //wait for serial buffer to receive 18 bytes of data
    if (Serial1.read() == 0x7E) {  //start delimitter of every API frame
      for (ui i = 0; i < 11; i++)  //ignore starting 12 bytes of data
        Serial1.read();
      for (ui i = 0; i < 5; i++)  //read button state & joystick values which is 5 bytes of data
        tempData[i] = Serial1.read();
      Serial1.read(); //empty the buffer by reading last byte of data

      //merge the HIGH and LOW bytes of 10 bit ADC values of joystick
      potX = (tempData[1] << 8) | tempData[2];           //joystick in x-direction 
      potY = (tempData[3] << 8) | tempData[4];           //joystick in y-direction
      button = tempData[0];    //Button pressed
    }
    return 1;  //if data read succesfully
  }
  return 0;  //to avoid misbehaviour of bot if data is not available
}


void forward() {
  digitalWrite(DIRA1, HIGH);
  digitalWrite(DIRA2, LOW);
  digitalWrite(DIRB1, HIGH);
  digitalWrite(DIRB2, LOW);
  analogWrite(ENA, 200);
  analogWrite(ENB, 200);
}
void backward() {
  digitalWrite(DIRA1, LOW);
  digitalWrite(DIRA2, HIGH);
  digitalWrite(DIRB1, LOW);
  digitalWrite(DIRB2, HIGH);
  analogWrite(ENA, 200);
  analogWrite(ENB, 200);
}
void right() {
  digitalWrite(DIRA1, LOW);
  digitalWrite(DIRA2, HIGH);
  digitalWrite(DIRB1, HIGH);
  digitalWrite(DIRB2, LOW);
  analogWrite(ENA, 200);
  analogWrite(ENB, 200);
}
void left() {
  digitalWrite(DIRA1, HIGH);
  digitalWrite(DIRA2, LOW);
  digitalWrite(DIRB1, LOW);
  digitalWrite(DIRB2, HIGH);
  analogWrite(ENA, 200);
  analogWrite(ENB, 200);
}
void Stop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}
