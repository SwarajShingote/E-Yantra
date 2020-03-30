/*
  Team Id: #BP2155
  Author List: Aayush Singh,Mrunal Joshi,Swaraj Shingote,Datta Dhore
  Filename: imu.h
  Theme: Biped Patrol
  Functions: MPU_Init(), calibrate_angle_offset(), read_accel(), read_gyro(), lowpassfilter(), highpassfilter(), comp_filter_pitch(), comp_filter_roll()
  Global Variables: NONE
*/

/*
   Function Name: MPU_Init()
   Input: NONE
   Output: NONE
   Logic: Initializing communication with MPU attached at I2C bus
   Example Call: MPU_Init();
*/

void MPU_Init() {

  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x01);                  // place a 01 into the 6B register
  Wire.endTransmission(true);        //end the transmission

  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x00);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)

  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                  //Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x00);                  //Set the register bits as 00000000 (+/- 250degree/sec. full scale range)
  Wire.endTransmission(true);
  Serial.println("Mpu initialized");
}

/*
   Function Name: calibrate_angle_offset()
   Input: NONE
   Output: float, angular offset value
   Logic: Read the MPU values till it settles down and last read value will be the offset
   Example Call: calibrate_angle_offset();
*/
float calibrate_angle_offset() {
  RGB_color(1, 0, 0, 1);  //turn ON red led and buzzer indicating MPU calibration
  unsigned long calibrationTime = micros(); //start loop time
  unsigned int skipCount = 0; //set skipCount  
  while (skipCount < 250) {  //read and ignore 250 values
    read_accel();  //read accelerometer values
    read_gyro();   //read gyroscope values
    comp_filter_pitch(lpf_y, lpf_z, -hpf_x);  //pass raw values to complimentary filter
    theta = pitch;  //ignore the read value
    skipCount++;  //increment skipCount
    while (micros() - calibrationTime < 10000);  //loop for 10 ms (10000 microseconds)
    calibrationTime = micros();  //restart timer
  }
  RGB_color(0, 1, 0, 0); //Disable buzzer and turn ON green led indicating succesfull calibration
  return pitch;  // return the last value of pitch
}

/*
   Function Name: read_accel()
   Input: NONE
   Output: Set global values of ax,ay,az
   Logic: Reading raw data from accelerometer registers
   Example Call: read_accel();
*/

void read_accel() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers

  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  ax = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  ay = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  az = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  lowpassfilter();
}
/*
   Function Name: read_gyro()
   Input: NONE
   Output: Set global values of ax,ay,az
   Logic: Reading raw data from gyroscope registers
   Example Call: read_gyro();
*/

void read_gyro() {
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers

  gx = (Wire.read() << 8 | Wire.read()) * deg2rad / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  gy = (Wire.read() << 8 | Wire.read()) * deg2rad / 131.0;
  gz = (Wire.read() << 8 | Wire.read()) * deg2rad / 131.0;
  highpassfilter();

}

/*
   Function Name: lowpassfilter()
   Input: Raw accelerometer values
   Output: Set global value of lpf_x, lpf_y, lpf_z
   Logic: passing raw data from accelerometer to low pass filter
   Example Call: lowpassfilter();
*/

void lowpassfilter() {
  float Tau = 1 / (2 * Pi * f_cut);
  float alpha = Tau / (Tau + dT);

  lpf_x = ((1 - alpha) * ax) + (alpha * (lpf_x));
  lpf_y = ((1 - alpha) * ay) + (alpha * (lpf_y));
  lpf_z = ((1 - alpha) * az) + (alpha * (lpf_z));

}

/*
   Function Name: highpassfilter()
   Input: Raw gyroscope values
   Output: Set global value of hpf_x, hpf_y, hpf_z
   Logic: passing raw data from gyroscope to high pass filter
   Example Call: highpassfilter();
*/

void highpassfilter() {
  float Tau = 1 / (2 * Pi * f_cut);  
  float alpha = Tau / (Tau + dT);

  hpf_x = ((1 - alpha) * prev_hpf_x) + ((1 - alpha) * (gx - prev_gx));
  prev_gx = gx;
  prev_hpf_x = hpf_x;

  hpf_y = ((1 - alpha) * prev_hpf_y) + ((1 - alpha) * (gy - prev_gy));
  prev_gy = gy;
  prev_hpf_y = hpf_y;

  hpf_z = ((1 - alpha) * prev_hpf_z) + ((1 - alpha) * (gz - prev_gz));
  prev_gz = gz;
  prev_hpf_z = hpf_z;

}

/*
   Function Name: comp_filter_pitch()
   Input: ay,az,gx
   Output: set global value of pitch
   Logic: using complementry filter to find pitch angle
   Example Call: comp_filter_pitch(lpf_y, lpf_z, -hpf_x);
*/

void comp_filter_pitch(float ay, float az, float gx) {
  float alpha = 0.02;  //filter tuning parameter

  acc = asin(ay / (sqrt((az * az) + (ay * ay))));
  pitch = ((1 - alpha) * (pitch + gx * dT)) + (alpha * acc);

}
/*
   Function Name: comp_filter_roll()
   Input: ax,az,gy
   Output: set global value of roll
   Logic: using complementry filter to find roll angle
   Example Call: comp_filter_roll(lpf_x, lpf_z, -hpf_y);
*/

void comp_filter_roll(float ax, float az, float gy) {
  float alpha = 0.005;  //filter tuning parameter 
  
  acc = asin(ax / (sqrt((az * az) + (ax * ax)))) ;
  roll = ((1 - alpha) * (roll + gy * dT)) + (alpha * acc);
}
