/*
  Team Id: #BP2155
  Author List: Aayush Singh,Mrunal Joshi,Swaraj Shingote,Datta Dhore
  Filename: config.h
  Theme: Biped Patrol
  Functions: declaration of global variables and functions
*/

typedef unsigned int ui;
typedef unsigned long ul;

#define Pi 3.1415926535897932384626433832795
#define rad2deg 57.2957786
#define deg2rad Pi / 180

#define Radius 0.0325                                 //in metre
#define fullTick 1080                                 //encoder count on 1 full revolution (270 * 4)
#define tick2dist (2 * Pi* Radius * 100) / fullTick   //convert encoder count to distance in cm

#define torque2pwm 255/12  //linear relation between torque and pwm(not so direct)
#define rightMotorSpeed 60 //saturation PWM value for right motor
#define leftMotorSpeed 60  //saturation PWM value for left motor

#define cutOffAng 30  //stop the motors on exceeding 30°

#define blue 49 //blue led
#define green 47 //green led
#define red 51  //red led
#define Vcc 45  //virtual Vcc pin
#define buzzer 11    //buzzer pin

#define magnetFront  12    //Electromagnet pin
#define magnetBack  13    //Electromagnet pin

#define ENA     10   //enable pin for motor A right
#define ENB     9    //enable pin for motor B left  defective
#define DIRA1   6    //direction pin 1 of motor A
#define DIRA2   5    //direction pin 2 of motor A
#define DIRB1   8    //direction pin 1 of motor B
#define DIRB2   7    //direction pin 2 of motor B

#define encoderPinL1  19 //pin1 left encoder motor
#define encoderPinL2  18 //pin2 left encoder motor
#define encoderPinR1  3  //pin1 right encoder motor
#define encoderPinR2  2  //pin2 right encoder motor


int button, potX, potY;  //global variables to store processed values of API frame

//global variables to hold MPU raw, filtered values
volatile float lpf_x = 0, lpf_y = 0, lpf_z = 0,
               hpf_x = 0, hpf_y = 0, hpf_z = 0,
               prev_hpf_x = 0, prev_hpf_y = 0, prev_hpf_z = 0,
               prev_gx = 0, prev_gy = 0, prev_gz = 0,
               ax, ay, az,
               gx, gy, gz,
               roll, pitch, yaw, theta, theta_dot;

const int MPU = 0x68; // MPU6050 I2C address
int f_cut = 5;  //cutoff frequency for low pass and high pass filter
int torque; //converted value from LQR --> PWM
float acc,  //accelerometer value
      gyroOff = 0.04,  //angular velocity offset value
      dT = 0.01,  //sampling interval of system -->10ms (10/1000 = 0.01s)
      tiltOut,    //LQR output
      thetaOff,   //angular position offset
      theta_offSet = 0;  //introduce error in angle value to make the bot move forward or backward using joystick

volatile ul time_ms = 0;
volatile ul time_sec = 0;
ul total, timeTaken, lastTime;


volatile int lastEncodedL = 0;
volatile long encoderValueL = 0;
long lastencoderValueL = 0;


volatile int lastEncodedR = 0;    //last count of encoder
volatile long encoderValueR = 0;  //current encoder count
long lastencoderValueR = 0;       //previous encoder value

long newPos = 0, oldPos = 0;  //current and previous position using encoder used to calculate velocity
float vel, dist;  //variables to store distance and velocity
ul newTime = 0, oldTime = 0;  //current and previous time for calculating velocity

ul loopStart;  //time when execution of loop begins
int FLeftOff = 0, FRightOff = 0, BLeftOff = 0, BRightOff = 0, bridgeBoost = 0;  //offset values given to motors for motion, extra boost added on bridge

// -2.13732, -15.4659, -15.0473, -2.24193  best value  54 R L 96
//-2.14379,-15.2732,-14.7623,-2.21979  alpha 0.33
//-2.13625,-15.497,-15.093,-2.24549
//-2.33688,-15.6648,-15.3172,-2.26444
//-2.43203,-15.7099,-15.3728,-2.26948
//-2.52356,-15.7532,-15.4261,-2.2743
//-2.69034,-15.9811,-15.7871,-2.30041  x balance hua thoda
//-2.77289,-16.0193,-15.8333,-2.30467
//-2.85302,-16.0564,-15.878,-2.3088  best x displacement
//-2.93095,-16.0924,-15.9214,-2.31281 x good gyro good
// -2.70993, -15.8226, -15.5009, -2.28738 good on remote control with 2 degree offset
//-2.70795,-15.866,- 15.5641,-2.29235
//-2.70598, -15.9087, -15.6264, -2.29726 last video recorded using this
double K[] = { -2.23715, -15.0141, -14.9737, -2.29441};  //K matrix obtained from syste modelling on octave

//Initialize Xbee
bool readData();
void xbee_data();

//Initialize Timer 3, 4, 5
void init_timer3();
void start_timer3();
void init_timer4();
void init_timer5();
ul epoch();

//Initialize motors, encoders, led's, magnets
void init_motors();
void init_magnets();
void init_led_buzz();
void RGB_color(bool , bool , bool, bool);
void init_encoder();

//Initialize MPU 
void MPU_Init();
void read_accel();
void read_gyro();
void lowpassfilter();
void highpassfilter();
void comp_filter_pitch(float, float, float);
void comp_filter_roll(float, float, float);
float calibrate_angle_offset();

//Initialize ISR on encoder interrupt
void updateEncoderL();
void updateEncoderR();

//Functions to make the bot travel in either direction or stop
void forward(ui torque);
void backward(ui torque);
void right();
void left();
void Stop();
