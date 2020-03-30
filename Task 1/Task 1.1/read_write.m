global A = csvread('csv_matter.csv');  #do not change this line

################################################
global lpf_x=0 lpf_y=0 lpf_z=0;   #output of low pass filter initialized with 0
global hpf_x=0 hpf_y=0 hpf_z=0 prev_hpf_x=0 prev_hpf_y=0 prev_hpf_z=0;   #output and previous values of high pass filter initialized with 0
global prev_gx=0 prev_gy=0 prev_gz=0;  #previous values of gyroscope readings initialized with 0
global pitch=0 roll=0;                 #variables to hold pitch and roll initialized with 0
################################################


function read_accel(axl,axh,ayl,ayh,azl,azh)  
  
  #################################################
  Reg_add=@(x,y) bitor(bitshift(x,8),y); #anonymous function to add high and low registers by bitshifting and or operation
  thresh=@(x) ifelse(x>32767,x-65536,x); #anonymous function for thresholding of values above 32767
  f_cut=5; scaling_accel=16384;  #scaling factor for accelerometer set using AFS_SEl=0

  ax=thresh(Reg_add(axh,axl))/scaling_accel;  #raw acceleration in x-axis after thresholding and scaling

  ay=thresh(Reg_add(ayh,ayl))/scaling_accel;  #raw acceleration in y-axis after thresholding and scaling
  
  az=thresh(Reg_add(azh,azl))/scaling_accel;  #raw acceleration in z-axis after thresholding and scaling
  #################################################


  ####################################################
  lowpassfilter(ax,ay,az,f_cut);  #passing raw values of acceleration to low pass filter with cutoff frequency set to 5
  ####################################################

endfunction

function read_gyro(gxl,gxh,gyl,gyh,gzl,gzh)
  
  #################################################
  Reg_add=@(x,y) bitor(bitshift(x,8),y);  #anonymous function to add high and low registers by bitshifting and or operation
  thresh=@(x) ifelse(x>32767,x-65536,x);  #anonymous function for thresholding of values above 32767
  f_cut=5; scaling_gyro=131;              #scaling factor for gyroscope set using FS_SEl=0

  gx=thresh(Reg_add(gxh,gxl))/scaling_gyro;  #raw gyroscope in x-axis after thresholding and scaling
  
  gy=thresh(Reg_add(gyh,gyl))/scaling_gyro;  #raw gyroscope in y-axis after thresholding and scaling
   
  gz=thresh(Reg_add(gzh,gzl))/scaling_gyro;  #raw gyroscope in z-axis after thresholding and scaling
  #################################################


  #####################################################
  highpassfilter(gx,gy,gz,f_cut);  #passing raw values of gyroscope to high pass filter with cutoff frequency set to 5
  #####################################################;

endfunction



function lowpassfilter(ax,ay,az,f_cut)
  dT = 0.01;  #time in seconds
  Tau= 1/(2*pi*f_cut);
  alpha = Tau/(Tau+dT);                #do not change this line
  
  ################################################
  global lpf_x lpf_y lpf_z;  #using values defined in global
  lpf_x=((1-alpha)*ax) + (alpha*(lpf_x));   #value of acceleration in x-axis after being passed through low pass filter
  lpf_y=((1-alpha)*ay) + (alpha*(lpf_y));   #value of acceleration in y-axis after being passed through low pass filter
  lpf_z=((1-alpha)*az) + (alpha*(lpf_z));   #value of acceleration in z-axis after being passed through low pass filter
  ################################################
  
endfunction



function highpassfilter(gx,gy,gz,f_cut)
  dT = 0.01;  #time in seconds
  Tau= 1/(2*pi*f_cut);
  alpha = Tau/(Tau+dT);                #do not change this line
  
  ################################################
  global hpf_x hpf_y hpf_z prev_hpf_x prev_hpf_y prev_hpf_z prev_gx prev_gy prev_gz; #using values defined in global
  
  hpf_x = ((1-alpha)*prev_hpf_x) + ((1-alpha)*(gx-prev_gx)); #value of gyroscope in x-axis after being passed through high pass filter
  prev_gx=gx;prev_hpf_x=hpf_x;  #storing values of raw gyroscope readings of and output of high pass filter to be used as previous values in next iteration 
  
  
  hpf_y = ((1-alpha)*prev_hpf_y) + ((1-alpha)*(gy-prev_gy)); #value of gyroscope in y-axis after being passed through high pass filter
  prev_gy=gy;prev_hpf_y=hpf_y;  #storing values of raw gyroscope readings and output of high pass filter to be used as previous values in next iteration
  
  
  hpf_z = ((1-alpha)*prev_hpf_z) + ((1-alpha)*(gz-prev_gz)); #value of gyroscope in z-axis after being passed through high pass filter
  prev_gz=gz;prev_hpf_z=hpf_z;  #storing values of raw gyroscope readings and output of high pass filter to be used as previous values in next iteration
  ################################################
  
endfunction

function comp_filter_pitch(ax,ay,az,gx,gy,gz)

  ##############################################
  global pitch;  #using globally defined pitch
  f_cut=5;
  dT = 0.01;  #time in seconds
  alpha = 0.03;
  #use the changes in values of accelerometer readings of
  #Y-axis & Z-axis and gyroscope readings of X-axis.
  acc = asind(ay/(sqrt((az*az)+(ay*ay))));  #calculating angle of inclination along x-axis using acceleration in z and y direction
  
  pitch = ((1-alpha)*(pitch+gx*dT))+(alpha*acc); #combining the values obtained by gyroscope and accelerometer after filtering to find the pitch
  ##############################################

endfunction 

function comp_filter_roll(ax,ay,az,gx,gy,gz)

  ##############################################
  global roll;  #using globally defined roll
  f_cut=5;
  dT = 0.01;  #time in seconds
  
  alpha = 0.03;
  #use the changes in values of accelerometer readings of
  #X-axis & Z-axis and gyroscope readings of Y-axis.
  acc = asind(ax/(sqrt((az*az)+(ax*ax))));  #calculating angle of inclination along y-axis using acceleration in z and x direction
  
  roll=((1-alpha)*(roll+gy*dT))+(alpha*acc);  #combining the values obtained by gyroscope and accelerometer after filtering to find the roll
  ##############################################

endfunction 

function execute_code
  
  global A pitch roll; #using globally defined pitch roll and data of file saved in A
  global lpf_x lpf_y lpf_z hpf_x hpf_y hpf_z n; #output of high pass and low pass filter
  for n = 1:rows(A)                    #do not change this line
    
    ###############################################
    read_accel(A(n,2),A(n,1),A(n,4),A(n,3),A(n,6),A(n,5));   #convert high and low register values of accelerometer into raw 16 bit signed integer
    read_gyro(A(n,8),A(n,7),A(n,10),A(n,9),A(n,12),A(n,11)); #convert high and low register values of gyroscope into raw 16 bit signed integer
    comp_filter_pitch(lpf_x,lpf_y,lpf_z,-hpf_x,hpf_y,hpf_z);  #pass low pass and high pass values of gyroscope and accelerometer to complimentary filter to calculate roll
    comp_filter_roll(lpf_x,lpf_y,lpf_z,hpf_x,-hpf_y,hpf_z);   #pass low pass and high pass values of gyroscope and accelerometer to complimentary filter to calculate pitch
    B(n,1)=pitch; B(n,2)=roll;  #storing pitch and roll values in B
    ###############################################
    
  endfor
  csvwrite('output_data.csv',B);        #do not change this line
endfunction


execute_code                           #do not change this line
