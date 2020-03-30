#define Pi 3.1415926535897932384626433832795
#define Radius 0.0325                                 //in metre
#define fullTick 1080                                 //ticks in 1 full revolution (270 * 4)
#define tick2dist (2 * Pi* Radius * 100) / fullTick   //in cm
#define rad2deg 57.2957786
#define deg2rad Pi / 180
#define torque2pwm 255/12
#define leftSpeed 100
#define rightSpeed 100
#define threshAngle 60
#define cutOffAng 30


#define ledPin 13
#define ENA     10    //enable pin for motor A right
#define ENB     9    //enable pin for motor B left  defective
#define DIRA1   5    //direction pin 1 of motor A
#define DIRA2   6    //direction pin 2 of motor A
#define DIRB1   7    //direction pin 1 of motor B
#define DIRB2   8    //direction pin 2 of motor B
//#define magnet  9    //Electromagnet pin
//#define buzzer 10    //buzzer pin

#define encoderPinL1  19
#define encoderPinL2  18
#define encoderPinR1  2
#define encoderPinR2  3

typedef unsigned int ui;
int button, potX, potY;  //global variables to store processed values of API frame

float lpf_x = 0, lpf_y = 0, lpf_z = 0,
      hpf_x = 0, hpf_y = 0, hpf_z = 0,
      prev_hpf_x = 0, prev_hpf_y = 0, prev_hpf_z = 0,
      prev_gx = 0, prev_gy = 0, prev_gz = 0,
      ax, ay, az,
      gx, gy, gz,
      dT, Tau, alpha,
      roll, pitch, yaw, theta, theta_dot, theta_offSet = 0;
const int MPU = 0x68; // MPU6050 I2C address
int f_cut;
float acc;
float gyroOff = 0.04;

volatile int lastEncodedL = 0;
volatile long encoderValueL = 0;
long lastencoderValueL = 0;


volatile int lastEncodedR = 0;
volatile long encoderValueR = 0;
long lastencoderValueR = 0;

long newPos = 0, oldPos = 0;
float vel, dist;
unsigned long newTime = 0, oldTime = 0;

unsigned long loopStart;
ui cutOff = 12, loopCount = 0;
int leftOff = 0, rightOff = 0;
int torque;
bool k = 0;

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
double K[] = {-2.70598,-15.9087,-15.6264,-2.29726};


bool readData();  //function to read API frames
void xbee_data();
void MPU_Init();
void read_accel();
void read_gyro();
void lowpassfilter();
void highpassfilter();
void comp_filter_pitch();
void comp_filter_roll();
void calibrateOffset();
void updateEncoderL();
void updateEncoderR();
bool readData(void);
void forward(unsigned int torque);
void backward(unsigned int torque);
void right();
void left();
void Stop();
