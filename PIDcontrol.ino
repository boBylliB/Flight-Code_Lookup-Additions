//PID Control Parameters for Pitch
#define kp_pitch 0.0
#define ki_pitch 0.0
#define kd_pitch 0.0

//PID Control Parameters for Yaw
#define kp_yaw 0.0
#define ki_yaw 0.0
#define kd_yaw 0.0

//Target Pitch and Yaw Angles
float target_pitch = 0.0;
float target_yaw = 0.0;

//Initializing Integral Error
float int_pitch = 0.0; 
float int_yaw = 0.0;

void PIDcontrol(float orient[6], float outputang[2], unsigned long dt) {

  //Assign orientation values to pitch, yaw, pitch_rate, and yaw_rate variables
  float pitch = orient[1];
  float yaw = orient[2];
  float pitch_rate = orient[4];
  float yaw_rate = orient[5];


  //Error calculation
  float pitch_error = target_pitch - pitch;
  float yaw_error = target_yaw - yaw;

  //Proportional error
  float prop_pitch = kp_pitch*pitch_error;
  float prop_yaw = kp_yaw*yaw_error;

  //Integral error
  int_pitch = int_pitch + (ki_pitch*pitch_error*dt);
  int_yaw = int_yaw + (ki_yaw*yaw_error*dt);

  //Derivative error
  float der_pitch = kd_pitch*pitch_rate;
  float der_yaw = kd_yaw*yaw_rate;

  //PID Output
  float output_pitch = prop_pitch + int_pitch - der_pitch;
  float output_yaw = prop_yaw + int_yaw - der_yaw;

  //Updating outputang[] array with PID processed correction angles
  outputang[1] = output_pitch;
  outputang[2] = output_yaw;
  
}

