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

	// Setup Servos
	servoA.attach(SERVOA_PIN);
	servoB.attach(SERVOB_PIN);
	
	// Setup SD IO
	if (!SD.begin(BUILTIN_SDCARD)) {
		int code[] = {LONG_CODE, LONG_CODE};
		CodeFailure(code);
	}
	SD.remove("arduino.txt");

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
		servoA.write(SERVO_MID);
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
		delay(5000);

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
	if (linaccel.z > 50) {
		delay(1000);
		digitalWrite(LEDPin_red, LOW);
		delay(1000);
		SwitchStateTo(Boost_State);
	}*/
}

#define BOOST_TIME 4000
void Boost() {
	// TODO: Acquire orientation data
	
	// TODO: PID servo positions
	
	// TODO: Move Servos
	
	// TODO: Log data to SD IO
	
	if (millis() - FlightStateTimer > BOOST_TIME) {
		// TODO: Set servos to neutral position

		servoA.detach(SERVOA_PIN);
		servoB.detach(SERVOB_PIN);
		
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
