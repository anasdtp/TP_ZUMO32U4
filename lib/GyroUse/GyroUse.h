/* Class for configuring the Zumo 32U4's gyro, 
calibrating it, and using it to measure how much the robot 
has turned about its Z axis. */

#pragma once

#include <Zumo32U4.h>

class GyroUse {
public:
    // Constants for angle calculations
    static const int32_t turnAngle45 = 0x08000000;
    static const int32_t turnAngle90 = turnAngle45 * 2;
    static const int32_t turnAngle1 = (turnAngle45 + 5) / 45;

    // Constructor
    GyroUse(Zumo32U4IMU& imuRef);
    
    // Public methods
    void setup();
    void reset();
    void resetPosition();
    void update();
    
    // Getters for gyroscope
    int32_t getTurnAngle() const;
    int16_t getTurnRate() const;
    
    // Getters for acceleration (in mg - milligravity)
    int16_t getAccelX() const;
    int16_t getAccelY() const;
    int16_t getAccelZ() const;
    
    // Getters for velocity (in mm/s)
    float getVelocityX() const;
    float getVelocityY() const;
    
    // Getters for position (in mm)
    float getPositionX() const;
    float getPositionY() const;
    
    // Get current orientation angle in radians
    float getOrientationRadians() const;

private:
    // Reference to IMU object
    Zumo32U4IMU& imu;
    
    // Gyroscope variables
    int32_t turnAngle;
    int16_t turnRate;
    int16_t gyroOffset;
    
    // Accelerometer variables (raw readings)
    int16_t accelX, accelY, accelZ;
    
    // Accelerometer offsets for calibration
    int16_t accelOffsetX, accelOffsetY, accelOffsetZ;
    
    // Velocity in mm/s (integrated from acceleration)
    float velocityX, velocityY;
    
    // Position in mm (integrated from velocity)
    float positionX, positionY;
    
    // Timing
    uint32_t lastUpdate;
    
    // Helper methods
    void calibrateAccelerometer();
    float convertAccelToMmS2(int16_t rawAccel) const;
};
