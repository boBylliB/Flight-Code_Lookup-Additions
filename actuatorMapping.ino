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
  actuators[1].forwardPin = actuator1Forward;
  actuators[1].backwardPin = actuator1Backward;
  actuators[2].forwardPin = actuator2Forward;
  actuators[2].backwardPin = actuator2Backward;
  actuators[3].forwardPin = actuator3Forward;
  actuators[3].backwardPin = actuator3Backward;
  for (int i = 1; i < 4; ++i) {
    if (iteration == 0) {
      // TODO: Find actual neutral lengths
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
  actuators[1].pwm = actuators[1].driveTime/actuators[3].driveTime*255;
  actuators[2].pwm = actuators[2].driveTime/actuators[3].driveTime*255;
  actuators[3].pwm = 255;
  // Assume acceleration and max velocity remain the same (duty cycles)
  actuators[1].driveTime = actuators[1].driveTime / actuators[1].pwm * 255; // ms
  actuators[2].driveTime = actuators[2].driveTime / actuators[2].pwm * 255; // ms

  // Sort by drive time again for stopping the actuators at the right time
  qsort(actuators, 3, sizeof(actuator), compareDriveTimes);

  // Driving actuators
  analogWrite(actuatorDirection(actuators[1].displacement, actuators[1].forwardPin, actuators[1].backwardPin), actuators[1].pwm);
  analogWrite(actuatorDirection(actuators[2].displacement, actuators[2].forwardPin, actuators[2].backwardPin), actuators[2].pwm);
  analogWrite(actuatorDirection(actuators[3].displacement, actuators[3].forwardPin, actuators[3].backwardPin), actuators[3].pwm);
  delay(actuators[1].driveTime);
  analogWrite(actuatorDirection(actuators[1].displacement, actuators[1].forwardPin, actuators[1].backwardPin), 0);
  delay(actuators[2].driveTime);
  analogWrite(actuatorDirection(actuators[2].displacement, actuators[2].forwardPin, actuators[2].backwardPin), 0);
  delay(actuators[3].driveTime);
  analogWrite(actuatorDirection(actuators[3].displacement, actuators[3].forwardPin, actuators[3].backwardPin), 0);

  // Set initial conditions for next iteration
  actuators[1].currentLength = lengthInputs[1]; // m
  actuators[2].currentLength = lengthInputs[2]; // m
  actuators[3].currentLength = lengthInputs[3]; // m
  ++iteration;
}
