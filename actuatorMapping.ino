#define actuator1Forward  0
#define actuator1Backward 1
#define actuator2Forward  2
#define actuator2Backward 3
#define actuator3Forward  4
#define actuator3Backward 5

struct actuator {
      byte forwardPin, backwardPin;
      double currentLength, displacement, driveTime;
      int pwm;
};

void setup() {
  pinMode(actuator1Forward, OUTPUT);
  pinMode(actuator1Backward, OUTPUT);
  pinMode(actuator2Forward, OUTPUT);
  pinMode(actuator2Backward, OUTPUT);
  pinMode(actuator3Forward, OUTPUT);
  pinMode(actuator3Backward, OUTPUT);
}

void loop() {
  float lengthInputs[3] = getLengths(); // inputs for getLengths taken from PIDcontrol.ino
  driveActuators(lengthInputs);
}

float getLengths(float inputPitch, float inputYaw) {
	
}

int compareDriveTimes(const actuator* actuator1, const actuator* actuator2) {
  return actuator1->driveTime - actuator2->driveTime;
}

byte actuatorDirection(float displacement, byte forwardPin, byte backwardPin) {
  if (displacement >= 0)
    return forwardPin;
  else
    return backwardPin;
}

void driveActuators(float* lengthInputs) {
  /*  Assume constants are acceleration and max speed, must be experimentally found
   *  Limitations: error propagates over time since there's no way to check position past initial conditions
   */ 
  // Initial conditions
  static int iteration = 0;
  // TODO: Find actual max speed and acceleration time
  float maxSpeed = 3; // m/s
  float accel = 3; // m/s^2
  float accelTime = maxSpeed/accel; // s
  float lengthTransient = 0.5*accel*pow(accelTime, 2); // m
  actuator actuators[3];

  for (int i = 1; i <= 3; ++i) {
    actuators[i].forwardPin = 2 * (i-1);
    actuators[i].backwardPin = 2*i;
    if (iteration == 0) {
      // TODO: Find actual neutral length
      actuators[i].currentLength = 1; // m, same length for all actuators when motor is straight
    }
    actuators[i].displacement = lengthInputs[i] - actuators[i].currentLength;

    // Calculating drive times for actuators at max power (function of desired length and current length)
    if (lengthInputs[i] >= lengthTransient) {
      //length1 = currentLength1 + lengthTransient*t/accelTime; // m
      actuators[i].driveTime = abs((lengthInputs[i] - actuators[i].currentLength))*accelTime/lengthTransient*1000; // ms
    } else {
      // length1 = currentLength1 + lengthTransient + maxSpeed*(t-accelTime); // m
      actuators[i].driveTime = (abs((lengthInputs[i] - actuators[i].currentLength - lengthTransient))/maxSpeed + accelTime)*1000; // ms
    }
  }
  // Assign drive timings and associated actuators from shortest to longest
  qsort(actuators, 3, sizeof(actuator), compareDriveTimes);

  // Slowing down the other two actuators to have the same drive time as the longest one
  for (int i = 1; i <= 2; ++i) {
    actuators[i].pwm = actuators[i].driveTime/actuators[3].driveTime*255;
    actuators[i].driveTime = actuators[i].driveTime / actuators[i].pwm * 255; // ms
  }

  // Sort by drive time again for stopping the actuators at the right time
  qsort(actuators, 3, sizeof(actuator), compareDriveTimes);

  // Driving actuators
  for (int i = 1; i <= 3; ++i) {
    analogWrite(actuatorDirection(actuators[i].displacement, actuators[i].forwardPin, actuators[i].backwardPin), actuators[i].pwm);
  }
  for (int i = 1; i <= 3; ++i) {
    delay(actuators[i].driveTime);
    analogWrite(actuatorDirection(actuators[i].displacement, actuators[i].forwardPin, actuators[i].backwardPin), 0);
  }

  // Set initial conditions for next iteration
  actuators[1].currentLength = lengthInputs[1]; // m
  actuators[2].currentLength = lengthInputs[2]; // m
  actuators[3].currentLength = lengthInputs[3]; // m
  ++iteration;
}
