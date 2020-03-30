void MPU_Init() {

  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x01);                  // Make reset - place a 0 into the 6B register
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
void read_gyro() {
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers

  gx = (Wire.read() << 8 | Wire.read()) * deg2rad / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  gy = (Wire.read() << 8 | Wire.read()) * deg2rad/ 131.0;
  gz = (Wire.read() << 8 | Wire.read()) * deg2rad/ 131.0;
  highpassfilter();

}
void lowpassfilter() {
  dT = 0.01;
  Tau = 1 / (2 * Pi * f_cut);
  alpha = Tau / (Tau + dT);

  lpf_x = ((1 - alpha) * ax) + (alpha * (lpf_x));
  lpf_y = ((1 - alpha) * ay) + (alpha * (lpf_y));
  lpf_z = ((1 - alpha) * az) + (alpha * (lpf_z));
}
void highpassfilter() {
  dT = 0.01;
  Tau = 1 / (2 * Pi * f_cut);
  alpha = Tau / (Tau + dT);

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

void comp_filter_pitch() {
  f_cut = 5;
  dT = 0.01;
  alpha = 0.033;

  acc = asin(ay / (sqrt((az * az) + (ay * ay))));
  pitch = ((1 - alpha) * (pitch + gx * dT)) + (alpha * acc);
//  Serial.print("pitch:  ");
//  Serial.print(pitch);
  
}
void comp_filter_roll() {

  f_cut = 5;
  dT = 0.01;

  alpha = 0.033;

  acc = asin(ax / (sqrt((az * az) + (ax * ax)))) ;
  roll = ((1 - alpha) * (roll + gy * dT)) + (alpha * acc);
//  Serial.print("   roll   ");
//  Serial.println(roll);
}
