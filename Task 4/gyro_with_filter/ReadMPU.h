#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define LED_PIN 13
bool blinkState = false;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;

float ypr[3];           // [yaw, pitch, roll]

volatile bool mpuInterrupt = false;
void dmpDataReady()
{
  mpuInterrupt = true;
}
int y_c = 0;
int p_c = 0;
int r_c = 0;
int count = 0;
int a = 0;

bool MPU_init() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24;
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
//  mpu.setXGyroOffset(205);
//  mpu.setYGyroOffset(17);
//  mpu.setZGyroOffset(-19);
//  mpu.setZAccelOffset(1349);
  if (devStatus == 0) {

    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    return 1;
  }
  else
  {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
    return 0;
  }
  pinMode(LED_PIN, OUTPUT);

}

/*//read data from xbee
    if (readData()) {
      Serial.print(button);   Serial.print("  ");
      Serial.print(potX);     Serial.print("  ");
      Serial.print(potY);     Serial.println("");

      if ((potX > 510 || potX < 545) && potY == 0) {  //joystick toggled vertically upwards
        forward();
        Serial.println("forward");
      }
      else if ((potX > 510 || potX < 545) && potY == 1023) {  //joystick toggled vertically downwards
        backward();
        Serial.println("backward");
      }
      else if (potX == 1023 && (potY > 510 || potY < 545)) {  //joystick toggled horizontally rightwards
        right();
        Serial.println("right");
      }
      else if (potX == 0 && (potY > 510 || potY < 545)) {  //joystick toggled horizontally leftwards
        left();
        Serial.println("left");
      }
      else { //stop the motors if joystick centred
        Stop();
        Serial.println("motor off");
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
  */
