void orientation(float orient[6]) {
	// Create quaternion object and get quaternion data from BNO055
	imu::Quaternion quat = bno.getQuat();

	/* // Get conjugate of quaternion from BNO055
	imu::Quaternion quatconj = quaternionConjugate(quat);
 	*/

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
	orient[1] = pitch;
	orient[2] = yaw;
	orient[3] = roll;

	// Assigning Pitch rate, Yaw rate, and Roll rate to elements of orient[] array
	orient[4] = pitch_rate;
	orient[5] = yaw_rate;
	orient[6] = roll_rate;

	// Plotting Pitch, Yaw, and Roll
	plotOrientation(pitch,yaw,roll);
}

void quaternionToEuler(float qx, float qy, float qz, float qw, float* pitch, float* yaw, float* roll) {
	// Convert quaternion to Euler angles (pitch, roll, and yaw) in degrees
	*pitch = atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy)) * 180.0 / M_PI;
	*yaw = asin(2.0 * (qw * qy - qz * qx)) * 180.0 / M_PI; 
	*roll = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)) * 180.0 / M_PI;
}

/* imu::Quaternion quaternionConjugate(imu::Quaternion quat) {
	imu::Quaternion conj;
	conj.w() = quat.w();
	conj.x() = -quat.x();
	conj.y() = -quat.y();
	conj.z() = -quat.z();
	return conj;  
} */

void plotOrientation(float pitch, float yaw, float roll) {
	// Display angles in the Serial Plotter
	Serial.print("Pitch:");
	Serial.print(pitch);
	Serial.print(",");
	Serial.print("Yaw:");
	Serial.println(yaw);
	Serial.print(",");
	Serial.print("Roll:");
	Serial.println(roll);
}
