#define actuator1Forward  0
#define actuator1Backward 1
#define actuator2Forward  2
#define actuator2Backward 3
#define actuator3Forward  4
#define actuator3Backward 5

struct actuator {
      byte forwardPin, backwardPin;
      double currentLength, displacement, driveTime;
  } actuator1, actuator2, actuator3;

void setup() {
  pinMode(actuator1Forward, OUTPUT);
  pinMode(actuator1Backward, OUTPUT);
  pinMode(actuator2Forward, OUTPUT);
  pinMode(actuator2Backward, OUTPUT);
  pinMode(actuator3Forward, OUTPUT);
  pinMode(actuator3Backward, OUTPUT);
  actuator1 = {actuator1Forward, actuator1Backward,,,};
  actuator2 = {actuator2Forward, actuator2Backward,,,};
  actuator3 = {actuator3Forward, actuator3Backward,,,};
}

void loop() {
  lengthInputs = getLengths(); // inputs for getLengths taken from PIDcontrol.ino
  driveActuators(lengthInputs(1), lengthInputs(2), lengthInputs(3));
}

double getLengths(double inputPitch, double inputYaw) {
	
}

int compare(const actuator* actuator1, const actuator* actuator2) {
  return actuator1->driveTime - actuator2->driveTime;
}

byte actuatorDirection(double displacement, byte forwardPin, byte backwardPin) {
  if (displacement >= 0)
    return forwardPin;
  else
    return backwardPin;
}

void driveActuators(double length1, double length2, double length3) {
	/*  Assume constants are acceleration and max speed, must be experimentally found
	 *  Limitations: error propagates over time since there's no way to check position past initial conditions
	 */ 
  // Initial conditions
  static int iteration = 0;
  // TODO: Find actual max speed and acceleration time
	double maxSpeed = 3; // m/s
	double accel = 3; // m/s^2
  double accelTime = maxSpeed/accel; // s
  double lengthTransient = 0.5*accel*pow(accelTime, 2); // m
  if (iteration == 0) {
    // TODO: Find actual neutral lengths
	  actuator1.currentLength = 1; // m, same length for all actuators when motor is straight
    actuator2.currentLength = 1; // m, same length for all actuators when motor is straight
    actuator3.currentLength = 1; // m, same length for all actuators when motor is straight
  }
  actuator1.displacement = length1 - actuator1.currentLength;
  actuator2.displacement = length2 - actuator2.currentLength;
  actuator3.displacement = length3 - actuator3.currentLength;
	// Calculating drive times for actuators at max power (function of desired length and current length)
  if (length1 >= lengthTransient) {
    //length1 = currentLength1 + lengthTransient*t/accelTime; // m
    actuator1.driveTime = abs((length1 - actuator1.currentLength))*accelTime/lengthTransient; // s
  } else {
    // length1 = currentLength1 + lengthTransient + maxSpeed*(t-accelTime); // m
    actuator1.driveTime = abs((length1 - actuator1.currentLength - lengthTransient))/maxSpeed + accelTime; // s
  }
  if (length2 >= lengthTransient) {
    //length1 = currentLength1 + lengthTransient*t/accelTime; // m
    actuator2.driveTime = abs((length2 - actuator2.currentLength))*accelTime/lengthTransient; // s
  } else {
    // length1 = currentLength1 + lengthTransient + maxSpeed*(t-accelTime); // m
    actuator2.driveTime = abs((length2 - actuator2.currentLength - lengthTransient))/maxSpeed + accelTime; // s
  }
  if (length3 >= lengthTransient) {
    //length1 = currentLength1 + lengthTransient*t/accelTime; // m
    actuator3.driveTime = abs((length3 - actuator3.currentLength))*accelTime/lengthTransient; // s
  } else {
    // length1 = currentLength1 + lengthTransient + maxSpeed*(t-accelTime); // m
    actuator3.driveTime = abs((length3 - actuator3.currentLength - lengthTransient))/maxSpeed + accelTime; // s
  }
  // Assign drive timings and associated actuators from shortest to longest
  actuator actuators[3] = {actuator1, actuator2, actuator3};
  qsort(actuators, 3, sizeof(actuator), compare);
  // Driving actuators
  analogWrite(actuatorDirection((length1-actuator1.currentLength), actuator1Forward, actuator1Backward), 255);
  analogWrite(actuatorDirection((length2-actuator2.currentLength), actuator2Forward, actuator2Backward), 255);
  analogWrite(actuatorDirection((length3-actuator3.currentLength), actuator3Forward, actuator3Backward), 255);
  delay(actuators[1].driveTime);
  analogWrite(actuatorDirection(actuators[1].displacement, actuators[1].forwardPin, actuators[1].backwardPin), 0);
  delay(actuators[2].driveTime);
  analogWrite(actuatorDirection(actuators[2].displacement, actuators[2].forwardPin, actuators[2].backwardPin), 0);
  delay(actuators[3].driveTime);
  analogWrite(actuatorDirection(actuators[3].displacement, actuators[3].forwardPin, actuators[3].backwardPin), 0);
  // Set initial conditions for next iteration
  actuator1.currentLength = length1; // m
  actuator2.currentLength = length2; // m
  actuator3.currentLength = length3; // m
  ++iteration;
}
