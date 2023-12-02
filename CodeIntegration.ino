#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SD.h>

File BenchFile;

Adafruit_BNO055 bno = Adafruit_BNO055(55);
//Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

#define userButtonPin 6
unsigned long userButtonTime = 1000;
const int userButtonDelay = 2000;

#define buzzerPin 37
#define LEDPin_green 41
#define LEDPin_yellow 40
#define LEDPin_red 39
#define LEDPin_blue 38

#define actuator1Forward  0
#define actuator1Backward 1
#define actuator2Forward  2
#define actuator2Backward 3
#define actuator3Forward  4
#define actuator3Backward 5

struct actuator {
	byte forwardPin, backwardPin;
	float currentLength, displacement, driveTime;
	int pwm;
};

void setup() {
  pinMode(userButtonPin, INPUT);
	
  pinMode(buzzerPin, OUTPUT);
	
  pinMode(LEDPin_green, OUTPUT);
  pinMode(LEDPin_yellow, OUTPUT);
  pinMode(LEDPin_red, OUTPUT);
  pinMode(LEDPin_blue, OUTPUT);

  pinMode(actuator1Forward, OUTPUT);
  pinMode(actuator1Backward, OUTPUT);
  pinMode(actuator2Forward, OUTPUT);
  pinMode(actuator2Backward, OUTPUT);
  pinMode(actuator3Forward, OUTPUT);
  pinMode(actuator3Backward, OUTPUT);
}

enum FlightStateType {
	BoardInit_State,
	Idle_State,
	LaunchReady_State,
	Boost_State,
	Coast_State,
	Recovery_State,
	ShutDown_State,
} FlightState;
unsigned long FlightStateTimer;

void loop() {
	// State Machine
	switch (FlightState) {
		case BoardInit_State:
			BoardInit();
			break;

		case Idle_State:
			Idle();
			break;

		case LaunchReady_State:
			LaunchReady();
			break;

		case Boost_State:
			Boost();
			break;

		case Coast_State:
			Coast();
			break;

		case Recovery_State:
			Recovery();
			break;

		case ShutDown_State:
			ShutDown();
			break;
	}
}

void SwitchStateTo(char state) {
	FlightStateTimer = millis();
	FlightState = state;
}

const unsigned long SHORT_CODE = 500;
const unsigned long LONG_CODE = 1000;
const unsigned int CODE_TONE = 262;
void CodeFailure(int failureCode[]) {
	digitalWrite(LEDPin_yellow, HIGH);

	for (int i = 0; i <= 2; i++) {
		int beep = failureCode[i];
		digitalWrite(LEDPin_blue, HIGH);
		tone(buzzerPin, CODE_TONE, beep);
		delay(beep);
		digitalWrite(LEDPin_blue, LOW);
		noTone(buzzerPin);
		delay(beep);
	}
	digitalWrite(LEDPin_blue, LOW);
	noTone(buzzerPin);

	digitalWrite(LEDPin_yellow, LOW);
	digitalWrite(LEDPin_red, HIGH);

	while(true);
}

void WaitForUserButton() {
	// Delay until user button input is found
	digitalWrite(LEDPin_blue, HIGH);
	while (!ReadUserButton()) {
		delay(100);
	}
	digitalWrite(LEDPin_blue, LOW);
	delay(userButtonDelay);
}

unsigned long lastDebounceTime;
int buttonState = LOW;
int lastButtonState = LOW;
bool ReadUserButton() {
	int reading = digitalRead(userButtonPin);

	if (reading != lastButtonState) {
		lastDebounceTime = millis();
	}

	if ((millis() - lastDebounceTime) > userButtonDelay) {
		if (reading != buttonState) {
			buttonState = reading;
		}
	}

	lastButtonState = reading;

	if (buttonState == HIGH && reading == HIGH) {
		return true;
	}
	else {
		return false;
	}
}

void BoardInit() {
	WaitForUserButton();
	{		
		SwitchStateTo(Idle_State);
	}
}

#define SERVO_MAX 50
#define SERVO_MID 90
void Idle() {
	// Setup BNO055
	if (!bno.begin()) {
		int code[] = {SHORT_CODE, LONG_CODE};
		CodeFailure(code);
	}
	
	// Setup SD IO
	/*if (!SD.begin(BUILTIN_SDCARD)) {
		int code[] = {LONG_CODE, LONG_CODE};
		CodeFailure(code);
	}
	SD.remove("arduino.txt");*/

	BenchFile = SD.open("arduino.txt", FILE_WRITE);
	if (!BenchFile) {
		int code[] = {LONG_CODE, SHORT_CODE};
		CodeFailure(code);
	}

	digitalWrite(LEDPin_green, HIGH);
	
	WaitForUserButton();
	{
		digitalWrite(LEDPin_yellow, HIGH);
		
		// Check Servo Range of Motion
		/*servoA.write(SERVO_MID);
		servoB.write(SERVO_MID);
		delay(5000);
		servoA.write(SERVO_MID + SERVO_MAX);
		servoB.write(SERVO_MID + SERVO_MAX);
		delay(5000);
		servoA.write(SERVO_MID - SERVO_MAX);
		servoB.write(SERVO_MID - SERVO_MAX);
		delay(5000);
		servoA.write(SERVO_MID);
		servoB.write(SERVO_MID);
		delay(5000);*/

		digitalWrite(LEDPin_yellow, LOW);
		
		WaitForUserButton();
		{
			// TODO: Calibrate BNO055
			bno.setExtCrystalUse(true);
			
			SwitchStateTo(LaunchReady_State);
		}
	}
}

void LaunchReady() {
	digitalWrite(LEDPin_red, HIGH);
 
	// Acquire acceleration data
	/*imu::Vector<3> linaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
 
	// If high acceleration in z direction (towards nosecone)
	if (linaccel.z > 1) {
		delay(1000);
		digitalWrite(LEDPin_red, LOW);
		delay(1000);
		SwitchStateTo(Boost_State);
	}*/
}

#define BOOST_TIME 4000
void Boost() {
	// Acquire orientation data
	float orient[6];
	unsigned long lastTime;
	float* outputang = (float*)malloc(2*sizeof(float));
	orientation(orient);
	// PID actuator positions
	PIDcontrol(orient, outputang, (millis()-lastTime));
	float lengthInputs[3];// = getLengths(1,1); // inputs for getLengths taken from PIDcontrol.ino
	// Move Servos
	driveActuators(lengthInputs);
	// TODO: Log data to SD IO
	String outputString = String(orient[0]) + "\t" + String(orient[1]) + "\t" + String(orient[2]);
	BenchFile = SD.open("orientation.txt", FILE_WRITE);
	if (BenchFile) {
		Serial.println("Writing to orientation.txt");
		BenchFile.println(outputString);
	} else {
		int code[] = {LONG_CODE, SHORT_CODE};
		CodeFailure(code);
	}
	
	if (millis() - FlightStateTimer > BOOST_TIME) {
		// Deactivate actuators
		analogWrite(actuator1Forward, 0);
		analogWrite(actuator1Backward, 0);
		analogWrite(actuator2Forward, 0);
    		analogWrite(actuator2Backward, 0);
    		analogWrite(actuator3Forward, 0);
    		analogWrite(actuator3Backward, 0);

		SwitchStateTo(Coast_State);
	}
}

#define COAST_TIME 4000
void Coast() {
	// TODO: Acquire and log orientation data
	
	if (millis() - FlightStateTimer > COAST_TIME) {
		SwitchStateTo(Recovery_State);
	}
}

void Recovery() {
	// Stop BNO055
	bno.enterSuspendMode();
	
	// Close SD IO
	BenchFile.close();
	
	// Cycle Lights and Buzzer
	digitalWrite(LEDPin_green, HIGH);
	digitalWrite(LEDPin_yellow, HIGH);
	digitalWrite(LEDPin_red, LOW);
	digitalWrite(LEDPin_blue, LOW);
	tone(buzzerPin, 1000);
	delay(3000);
	digitalWrite(LEDPin_green, LOW);
	digitalWrite(LEDPin_yellow, LOW);
	digitalWrite(LEDPin_red, HIGH);
	digitalWrite(LEDPin_blue, HIGH);
	noTone(buzzerPin);
	
	WaitForUserButton();
	{
		SwitchStateTo(ShutDown_State);
	}
}

void ShutDown() {
	exit(0);
}

void orientation(float orient[6]) {
	// Create quaternion object and get quaternion data from BNO055
	imu::Quaternion quat = bno.getQuat();

	// Create 3-axis vector for angular velocity
	imu::Vector<3> angvel = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

	// Assign conjugate quaternion values to variables
	float qw = quat.w();
	float qx = quat.x();
	float qy = quat.y();
	float qz = quat.z();

	// Get angular rate data from BNO055
	float pitch_rate = angvel.x();
	float yaw_rate = angvel.y();
	float roll_rate = angvel.z();

	// Convert quaternion data to Euler angles
	float pitch, yaw, roll;
	quaternionToEuler(qx,qy,qz,qw,&pitch,&yaw,&roll);

	// Assigning Pitch, Yaw, and Roll to elements of orient[] array
	orient[0] = pitch;
	orient[1] = yaw;
	orient[2] = roll;

	// Assigning Pitch rate, Yaw rate, and Roll rate to elements of orient[] array
	orient[3] = pitch_rate;
	orient[4] = yaw_rate;
	orient[5] = roll_rate;

	// Plotting Pitch, Yaw, and Roll
	plotOrientation(pitch,yaw,roll);
}

void quaternionToEuler(float qx, float qy, float qz, float qw, float* pitch, float* yaw, float* roll) {
	// Convert quaternion to Euler angles (pitch, roll, and yaw) in degrees
	*pitch = atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy)) * 180.0 / M_PI;
	*yaw = asin(2.0 * (qw * qy - qz * qx)) * 180.0 / M_PI; 
	*roll = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)) * 180.0 / M_PI;
}

void plotOrientation(float pitch, float yaw, float roll) {
	// Display angles in the Serial Plotter
	Serial.print("Pitch: ");
	Serial.print(pitch);
	Serial.print("\tYaw: ");
	Serial.println(yaw);
	Serial.print("\tRoll: ");
	Serial.println(roll);
}

// PID Control Parameters for Pitch
#define kp_pitch 0.0
#define ki_pitch 0.0
#define kd_pitch 0.0

// PID Control Parameters for Yaw
#define kp_yaw 0.0
#define ki_yaw 0.0
#define kd_yaw 0.0

// Target Pitch and Yaw Angles
float target_pitch = 0.0;
float target_yaw = 0.0;

// Initializing Integral Error
float int_pitch = 0.0; 
float int_yaw = 0.0;

void PIDcontrol(float orient[6], float* outputang, unsigned long dt) {
	// Assign orientation values to pitch, yaw, pitch_rate, and yaw_rate variables
	float pitch = orient[0];
	float yaw = orient[1];
	float pitch_rate = orient[3];
	float yaw_rate = orient[4];

	// Error calculation
	float pitch_error = target_pitch - pitch;
	float yaw_error = target_yaw - yaw;

	// Proportional error
	float prop_pitch = kp_pitch*pitch_error;
	float prop_yaw = kp_yaw*yaw_error;

	// Integral error
	int_pitch = int_pitch + (ki_pitch*pitch_error*dt);
	int_yaw = int_yaw + (ki_yaw*yaw_error*dt);

	// Derivative error
	float der_pitch = kd_pitch*pitch_rate;
	float der_yaw = kd_yaw*yaw_rate;

	// PID Output
	float output_pitch = prop_pitch + int_pitch - der_pitch;
	float output_yaw = prop_yaw + int_yaw - der_yaw;

	// Updating outputang[] array with PID processed correction angles
	outputang[0] = output_pitch;
	outputang[1] = output_yaw; 
}

float getLengths(float inputPitch, float inputYaw) {
	// TODO
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

  for (int i = 0; i <= 2; ++i) {
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
  for (int i = 0; i <= 1; ++i) {
    actuators[i].pwm = actuators[i].driveTime/actuators[3].driveTime*255;
    actuators[i].driveTime = actuators[i].driveTime / actuators[i].pwm * 255; // ms
  }

  // Sort by drive time again for stopping the actuators at the right time
  qsort(actuators, 3, sizeof(actuator), compareDriveTimes);

  // Driving actuators
  for (int i = 0; i <= 2; ++i) {
    analogWrite(actuatorDirection(actuators[i].displacement, actuators[i].forwardPin, actuators[i].backwardPin), actuators[i].pwm);
  }
  for (int i = 0; i <= 2; ++i) {
    delay(actuators[i].driveTime);
    analogWrite(actuatorDirection(actuators[i].displacement, actuators[i].forwardPin, actuators[i].backwardPin), 0);
  }

  // Set initial conditions for next iteration
  actuators[0].currentLength = lengthInputs[0]; // m
  actuators[1].currentLength = lengthInputs[1]; // m
  actuators[2].currentLength = lengthInputs[2]; // m
  ++iteration;
}
