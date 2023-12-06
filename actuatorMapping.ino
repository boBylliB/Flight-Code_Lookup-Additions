#include <SD.h>

File dataTable;
struct TableRange {
      int count;
      float min, max, diff;
};
TableRange tablePitchRange, tableYawRange;

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

  // Setup SD IO
  if (!SD.begin(BUILTIN_SDCARD)) {
    int code[] = {LONG_CODE, LONG_CODE};
    CodeFailure(code); // Pulled the code failure function from Flight-Code.ino
  }
  dataTable = SD.open("LengthTable.bin", FILE_WRITE);
  readLengthsFromFile("LengthTable.txt", &dataTable);
  // Explicitly reopening the file as "read" as this file should now remain constant
  fclose(dataTable);
  dataTable = SD.open("LengthTable.bin", FILE_READ);
}

void loop() {
  float lengthInputs[3];
  getLengths(0f, 0f, lengthInputs, &dataTable); // inputs for getLengths taken from PIDcontrol.ino
  driveActuators(lengthInputs);
}

void getLengths(float inputPitch, float inputYaw, float* lengths, File* dataFile) {
	// Using the input pitch and yaw, and the known table ranges, we calculate the position in the file
  // for which the desired lengths should be located
  // Clamp the pitch and yaw between the max and min
  if (inputPitch > tablePitchRange.max) inputPitch = tablePitchRange.max;
  if (inputPitch < tablePitchRange.min) inputPitch = tablePitchRange.min;
  if (inputYaw > tableYawRange.max) inputYaw = tableYawRange.max;
  if (inputYaw < tableYawRange.min) inputYaw = tableYawRange.min;
  // Find the bounding pitch "indices" from the desired pitch input
  int pitchIdx = (int)((inputPitch - tablePitchRange.min) / tablePitchRange.diff);
  // Find the bounding yaw "indices" from the desired yaw input
  int yawIdx = (int)((inputYaw - tableYawRange.min) / tableYawRange.diff);
  // Combine the two and take into account the 3 length values per angle to find the actual file locations
  byte tableData[12];
  byte* dataPtr = tableData;
  dataFile.seek((pitchIdx*tableYawRange.count + yawIdx) * 3);
  dataFile.read(tableData, 6);
  dataFile.seek(((pitchIdx+1)*tableYawRange.count + yawIdx) * 3);
  dataFile.read(tableData+6, 6);
  // Interpolate between the two to get the resulting length output
  // Since the length is a function of both pitch and yaw, we use bilinear interpolation
  // Yaw was arbitrarily chosen to go first, but order shouldn't matter
  float minPitch = pitchIdx*tablePitchRange.diff + tablePitchRange.min;
  float maxPitch = (pitchIdx+1)*tablePitchRange.diff + tablePitchRange.min;
  float minYaw = yawIdx*tableYawRange.diff + tableYawRange.min;
  float maxYaw = (yawIdx+1)*tableYawRange.diff + tableYawRange.min;
  int idx;
  float interp[6];
  for (idx = 0; idx < 3; idx++) {
    interp[idx] = (float)tableData[idx]*(maxYaw-inputYaw)/(tableYawRange.diff) + (float)tableData[idx+3]*(inputYaw-minYaw)/(tableYawRange.diff);
    interp[idx+3] = (float)tableData[idx+6]*(maxYaw-inputYaw)/(tableYawRange.diff) + (float)tableData[idx+9]*(inputYaw-minYaw)/(tableYawRange.diff);
  }
  for (idx = 0; idx < 3; idx++) {
    lengths[idx] = interp[idx]*(maxPitch-inputPitch)/(tablePitchRange.diff) + interp[idx+3]*(inputPitch-minPitch)/(tablePitchRange.diff);
  }
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

void readLengthsFromFile(const char* sourceFileName, File* targetFile) {
  /* This function converts the human-readable MATLAB-generated length table file and converts it into a more
   * compressed machine-readable (but therefore not very human-readable) file, intended to maximize read speed.
   * The human-readable file will have a header, in which the final line will contain a line of equals signs
   * Due to the file generation, simply checking for a line with an equals sign is enough to detect the end of
   * the header
   * The data format of the file is as follows:
   * <sign><pitchAngle %2.3f> <sign><yawAngle %2.3f> <length1 %2d> <length2 %2d> <length3 %2d>
   * To be more efficient with memory usage, these values will be packed by the following format:
   *   angles -> uint16_t's where the decimal place occurs after the second digit (ex: 7.24 -> 07240)
   *   lengths -> bytes, only utilizing 0-50mm
   * This, as a note, this is a tradeoff between relatively low RAM usage and read speed/precision
   * To streamline usage, the function "getLengths" takes this packed format and converts the length into a double
   * This packing format also specifies that angles MUST be less than 65.535 degrees in either direction with 0.001
   * degree precision, and that lengths MUST be less than 256mm with only millimeter precision
  */
  // Open the file
  File sourceFile = SD.open(sourceFileName, FILE_READ);
  if (!sourceFile) {
    int code[] = {LONG_CODE, SHORT_CODE};
    CodeFailure(code);
  }
  // Skip the header
  int rlen = sourceFile.available();
  bool foundEquals = false;
  char ch = ' ';
  for (int count = 0; count < rlen && !(foundEquals && ch == '\n'); count++, ch = sourceFile.read()) {
    if (ch == '=') foundEquals = true;
  }
  // Read in the file statistics to get the data ranges
  String buffer = "";
  rlen = sourceFile.available();
  int currentVar = 0;
  for (int count = 0; count < rlen && currentVar < 6; count++, ch = sourceFile.read()) {
    if (ch == ' ' || ch == '\n') {
      switch (currentVar) {
      case 0:
        tablePitchRange.count = buffer.toInt();
        break;
      case 1:
        tablePitchRange.min = buffer.toFloat();
        break;
      case 2:
        tablePitchRange.max = buffer.toFloat();
        break;
      case 3:
        tableYawRange.count = buffer.toInt();
        break;
      case 4:
        tableYawRange.min = buffer.toFloat();
        break;
      case 5:
        tableYawRange.max = buffer.toFloat();
        break;
      }
      buffer = "";
      currentVar++;
    }
    else
      buffer += ch;
  }
  tablePitchRange.diff = (tablePitchRange.max - tablePitchRange.min) / (tablePitchRange.count - 1);
  tableYawRange.diff = (tableYawRange.max - tableYawRange.min) / (tableYawRange.count - 1);
  // Transfer data to the target file
  buffer = "";
  rlen = sourceFile.available();
  currentVar = 0;
  for (int count = 0; count < rlen; count++, ch = sourceFile.read()) {
    if (ch == ' ' || ch == '\n') {
      if (!(currentVar == 0 || currentVar == 1)) {
        // Length, no conversion is needed
        // We ignore angle, as it should come in correctly from the MATLAB
        targetFile.write((byte)buffer.toInt())
      }
      buffer = "";
      currentVar = (currentVar + 1) % 6;
    }
    else
      buffer += ch;
  }
  // Close the file
  sourceFile.close();
}
