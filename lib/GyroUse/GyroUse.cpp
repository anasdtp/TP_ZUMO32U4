/* Class implementation for configuring the Zumo 32U4's gyro, 
calibrating it, and using it to measure how much the robot 
has turned about its Z axis. */

#include <Wire.h>
#include <math.h>
#include "GyroUse.h"

// Constructor
GyroUse::GyroUse(Zumo32U4IMU& imuRef) : 
    imu(imuRef), 
    turnAngle(0), 
    turnRate(0), 
    gyroOffset(0),
    accelX(0), accelY(0), accelZ(0),
    accelOffsetX(0), accelOffsetY(0), accelOffsetZ(0),
    velocityX(0.0f), velocityY(0.0f),
    positionX(0.0f), positionY(0.0f),
    lastUpdate(0) {
}

/* This should be called in setup() to enable and calibrate the
gyro. While calibrating, you should be careful to hold the robot
still.

The digital zero-rate level of the gyro can be as high as
25 degrees per second, and this calibration helps us correct for
that. */
void GyroUse::setup()
{
  // Calibrate the gyro.
  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++)
  {
    // Wait for new data to be available, then read it.
    while(!imu.gyroDataReady()) {}
    imu.readGyro();

    // Add the Z axis reading to the total.
    total += imu.g.z;
  }
  gyroOffset = total / 1024;

  // Calibrate accelerometer
  calibrateAccelerometer();

  // Reset the sensor
  reset();
}

// This should be called to set the starting point for measuring
// a turn.  After calling this, turnAngle will be 0.
void GyroUse::reset()
{
  lastUpdate = micros();
  turnAngle = 0;
  resetPosition();
}

// Read the gyro and update the angle.  This should be called as
// frequently as possible while using the gyro to do turns.
void GyroUse::update()
{
  // Read the measurements from the gyro and accelerometer
  imu.readGyro();
  imu.readAcc();
  
  // Update gyroscope data
  turnRate = imu.g.z - gyroOffset;
  
  // Update accelerometer data (subtract offsets)
  accelX = imu.a.x - accelOffsetX;
  accelY = imu.a.y - accelOffsetY;
  accelZ = imu.a.z - accelOffsetZ;

  // Figure out how much time has passed since the last update (dt)
  uint32_t m = micros();
  uint32_t dt = m - lastUpdate;
  lastUpdate = m;
  
  // Skip integration if dt is too large (indicates first call or long pause)
  if (dt > 100000) { // 100ms
    return;
  }
  
  float dtSeconds = dt / 1000000.0f; // Convert to seconds

  // Update gyroscope angle
  int32_t d = (int32_t)turnRate * dt;
  turnAngle += ((int64_t)d * 3670016) /17578125;
  
  // Get current orientation for coordinate transformation
  float orientation = getOrientationRadians();
  float cosTheta = cos(orientation);
  float sinTheta = sin(orientation);
  
  // Convert accelerometer readings to mm/s²
  float accelXWorld = convertAccelToMmS2(accelX);
  float accelYWorld = convertAccelToMmS2(accelY);
  
  // Transform acceleration from robot frame to world frame
  float accelXGlobal = accelXWorld * cosTheta - accelYWorld * sinTheta;
  float accelYGlobal = accelXWorld * sinTheta + accelYWorld * cosTheta;
  
  // Integrate acceleration to get velocity (mm/s)
  velocityX += accelXGlobal * dtSeconds;
  velocityY += accelYGlobal * dtSeconds;
  
  // Apply velocity damping to reduce drift
  velocityX *= 0.999f;
  velocityY *= 0.999f;
  
  // Integrate velocity to get position (mm)
  positionX += velocityX * dtSeconds;
  positionY += velocityY * dtSeconds;
}

// Reset position and velocity
void GyroUse::resetPosition() {
  velocityX = 0.0f;
  velocityY = 0.0f;
  positionX = 0.0f;
  positionY = 0.0f;
}

// Calibrate accelerometer offsets
void GyroUse::calibrateAccelerometer() {
  int32_t totalX = 0, totalY = 0, totalZ = 0;
  
  for (uint16_t i = 0; i < 1024; i++) {
    while(!imu.accDataReady()) {}
    imu.readAcc();
    
    totalX += imu.a.x;
    totalY += imu.a.y;
    totalZ += imu.a.z;
  }
  
  accelOffsetX = totalX / 1024;
  accelOffsetY = totalY / 1024;
  // Z axis should read approximately +1g when stationary, so we subtract 1g from the offset
  accelOffsetZ = (totalZ / 1024) - 16384; // 16384 = 1g in LSM303 scale
}

// Convert raw accelerometer reading to mm/s²
float GyroUse::convertAccelToMmS2(int16_t rawAccel) const {
  // LSM303 accelerometer: ±2g range, 16-bit resolution
  // 1g ≈ 16384 LSBs, 1g = 9806.65 mm/s²
  return (rawAccel / 16384.0f) * 9806.65f;
}

// Get current orientation in radians
float GyroUse::getOrientationRadians() const {
  // Convert turnAngle to radians
  // turnAngle45 = 0x08000000 represents 45 degrees = π/4 radians
  return (turnAngle / (float)turnAngle45) * (M_PI / 4.0f);
}

// Getter methods for gyroscope
int32_t GyroUse::getTurnAngle() const {
  return turnAngle;
}

int16_t GyroUse::getTurnRate() const {
  return turnRate;
}

// Getter methods for accelerometer (raw values in mg)
int16_t GyroUse::getAccelX() const {
  return accelX;
}

int16_t GyroUse::getAccelY() const {
  return accelY;
}

int16_t GyroUse::getAccelZ() const {
  return accelZ;
}

// Getter methods for velocity (in mm/s)
float GyroUse::getVelocityX() const {
  return velocityX;
}

float GyroUse::getVelocityY() const {
  return velocityY;
}

// Getter methods for position (in mm)
float GyroUse::getPositionX() const {
  return positionX;
}

float GyroUse::getPositionY() const {
  return positionY;
}
