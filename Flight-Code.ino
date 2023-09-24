#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SD.h>

//#include <Servo.h>
#include "C:\Program Files (x86)\Arduino\libraries\Servo"
//#include "C:\Users\derek\AppData\Local\Arduino15\packages\teensy\hardware\avr\1.58.1\libraries\Servo\Servo.h"

File BenchFile;

Adafruit_BNO055 bno = Adafruit_BNO055(55);
//Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

#define SERVOA_PIN 2
#define SERVOB_PIN 3
Servo servoA;
Servo servoB;

#define userButtonPin 6
unsigned long userButtonTime = 1000;
const int userButtonDelay = 2000;

#define buzzerPin 37
#define LEDPin_green 41
#define LEDPin_yellow 40
#define LEDPin_red 39
#define LEDPin_blue 38

void setup() {
	pinMode(userButtonPin, INPUT);
	
	pinMode(buzzerPin, OUTPUT);
	
	pinMode(LEDPin_green, OUTPUT);
	pinMode(LEDPin_yellow, OUTPUT);
	pinMode(LEDPin_red, OUTPUT);
	pinMode(LEDPin_blue, OUTPUT);
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

void SwitchStateTo(int state) {
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
