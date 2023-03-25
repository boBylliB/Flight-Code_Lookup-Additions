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