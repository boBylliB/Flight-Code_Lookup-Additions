This repository contains code for thrust vectoring a rocket motor. We do this to rotate the rocket back onto a certain path if it gets shifted somehow during flight. Currently we thrust vector to keep the rocket oriented upward. The orientation of the rocket thrust is changed by rotating the rocket motor with servos, both of which are attached to a gimbal.

Electronics we use:
Teensy microcontroller: Controls the accelerometer and servos; all data goes through this and is logged/processed.
BNO055: 6-axis Accelerometer/gyroscope, measures acceleration in 3 axes and rotation about 3 axes.
Servos: Turns the gimbal to orient the rocket motor.

Code File Summaries:
FlightStates.ino: Code for each different stage of the flight.
  BoardInit: Waits for a button press before moving to Idle.
  Idle: Sets up the BNO055, servos, and SD card for writing, then waits for a button press to test the servos' range of motion. Another button press calibrates the BNO055 and then switches to the LaunchReady state. If initialization fails for anything, the appropriate error message is displayed.
  LaunchReady: The rocket should be waiting on the launchpad. The program determines that the rocket has launched when upward acceleration exceeds a certain threshold, and then switches to the Boost state.
  Boost: The orientation and velocities from the BNO055 are recorded on the SD card. Error is calculated based on the difference between straight up and the rocket's current orientation, and is processed by PID code to generate a position for the servos to move to based on error. After a certain amount of time hard-coded based on the motor's burn time, switches to the Coast state.
  Coast: Orientation data is logged to the SD card. After a certain amount of time switches to the Recovery state.
  Recovery: The LEDs and buzzer are switched on and off to make the rocket easier to find. When the button is pressed, switches to the ShutDown stage.
  ShutDown: Program execution ends.

Flight-Code.ino: Sets pins and inputs/outputs for the servos, button, buzzer, and LEDs, and combines the different functions from FlightStates into state code, timing how long the program is in each state. If there is a code failure, the buzzer and blue LED flicker based on the error, looping after flashing the blue, yellow, and red LEDs.

GetOrientation.ino: Takes the outputs from the BNO055 as a quaternion and processes them into Euler angles (pitch, yaw, and roll).

PIDcontrol.ino: Calculates the difference between straight up and the rocket's orientation (error), and outputs a pitch and yaw for the servos to orient to for correcting toward moving upward.

UserButton.ino: Code for waiting for a button press (keeps blue LED on until press) and debouncing code.
