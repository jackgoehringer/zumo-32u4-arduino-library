// Copyright Pololu Corporation.  For more information, see http://www.pololu.com/
#pragma once

#include <Arduino.h>
#include <Zumo32U4IMU.h>
#include "Config.h"

/*
 * AngleEstimator.h - Complementary filter for angle estimation
 *
 * Fuses gyroscope and accelerometer data to estimate the robot's pitch angle.
 * - Gyro provides high-frequency angle updates (integrates angular velocity)
 * - Accelerometer provides low-frequency absolute reference (corrects drift)
 *
 * The complementary filter blends these two sources:
 *   angle = (1 - alpha) * gyroAngle + alpha * accelAngle
 *
 * Where alpha is dynamically adjusted based on accelerometer reliability.
 */

class AngleEstimator
{
public:
    AngleEstimator()
        : angle(0), angleRate(0), gyroOffset(0), gyroOffsetZ(0),
          lastUpdateTime(0), accelAngle(0)
    {
    }

    // Initialize with IMU reference and perform gyro calibration
    // Call this after imu.init() and imu.configureForBalancing()
    void begin(Zumo32U4IMU* imuPtr)
    {
        imu = imuPtr;
        calibrateGyro();
        lastUpdateTime = micros();
        angle = 0;
        angleRate = 0;
    }

    // Calibrate gyro offset by averaging samples while stationary
    // Calibrates both Y-axis (pitch) and Z-axis (yaw) offsets
    void calibrateGyro()
    {
        gyroOffset = 0;
        gyroOffsetZ = 0;

        for (uint16_t i = 0; i < GYRO_CALIBRATION_SAMPLES; i++)
        {
            while (!imu->gyroDataReady()) {}
            imu->readGyro();
            gyroOffset += imu->g.y;
            gyroOffsetZ += imu->g.z;
        }

        gyroOffset /= GYRO_CALIBRATION_SAMPLES;
        gyroOffsetZ /= GYRO_CALIBRATION_SAMPLES;
    }

    // Update angle estimate using gyro integration
    // Call this as fast as possible (every time gyro data is ready)
    // Returns true if new data was processed
    bool updateGyro()
    {
        if (!imu->gyroDataReady())
        {
            return false;
        }

        uint32_t now = micros();
        uint32_t dt = now - lastUpdateTime;
        lastUpdateTime = now;

        imu->readGyro();

        // Calculate angular velocity in degrees per second
        // Gyro sensitivity is 0.07 dps per digit for 2000 dps full scale
        // Positive gyro Y reading = robot tilting forward = positive angle change
        // (matches Pololu's official Balancing example sign convention)
        angleRate = ((float)imu->g.y - gyroOffset) * GYRO_SENSITIVITY;

        // Integrate to get angle change
        // dt is in microseconds, convert to seconds
        angle += angleRate * dt / 1000000.0f;

        return true;
    }

    // Correct angle estimate using accelerometer
    // Call this at a lower rate (e.g., 50 Hz)
    void correctWithAccel()
    {
        imu->readAcc();

        // Calculate angle from accelerometer using atan2
        // When robot is vertical: z points up, x points forward
        // angle = -atan2(z, -x) gives pitch angle
        // (matches Pololu's official Balancing example sign convention)
        accelAngle = -atan2(imu->a.z, -imu->a.x) * 180.0f / M_PI;

        // Calculate magnitude of acceleration vector in units of g
        // For 2g full scale: 4096 counts per g
        // For 8g full scale: 4096 counts per g (LSM6DS33 at this setting)
        float ax = (float)imu->a.x / 4096.0f;
        float ay = (float)imu->a.y / 4096.0f;
        float az = (float)imu->a.z / 4096.0f;
        float mag = sqrt(ax * ax + ay * ay + az * az);

        // Calculate weight for accelerometer correction
        // Trust accelerometer less when magnitude differs from 1g
        // (indicates robot is accelerating, not just gravity)
        float weight = 1.0f - ACCEL_TRUST_FACTOR * fabs(1.0f - mag);
        weight = constrain(weight, 0.0f, 1.0f);
        weight *= ACCEL_WEIGHT_MAX;

        // Apply complementary filter correction
        angle = weight * accelAngle + (1.0f - weight) * angle;
    }

    // Optional: Continuous gyro drift compensation
    // Call this when robot is relatively still
    void updateDriftCompensation(float velocity)
    {
#if ENABLE_GYRO_DRIFT_COMPENSATION
        // Only update offset when robot is quiet
        if (fabs(angleRate) < GYRO_DRIFT_RATE_THRESHOLD &&
            fabs(velocity) < GYRO_DRIFT_VELOCITY_THRESHOLD)
        {
            // Slowly adapt gyro offset
            gyroOffset = (1.0f - GYRO_DRIFT_ALPHA) * gyroOffset +
                         GYRO_DRIFT_ALPHA * (float)imu->g.y;
        }
#endif
    }

    // Get current angle estimate (degrees)
    float getAngle() const
    {
        return angle;
    }

    // Get current angular velocity (degrees per second)
    float getAngleRate() const
    {
        return angleRate;
    }

    // Get Z-axis gyro offset (for heading control)
    float getGyroOffsetZ() const
    {
        return gyroOffsetZ;
    }

    // Update Z-axis gyro drift compensation
    // Call when robot is relatively still (in middle loop)
    void updateZDriftCompensation(int16_t rawGyroZ, float headingRate, float velocity)
    {
#if ENABLE_GYRO_Z_DRIFT_COMPENSATION
        // Only update offset when robot is quiet
        if (fabs(headingRate) < GYRO_Z_DRIFT_RATE_THRESHOLD &&
            fabs(velocity) < GYRO_DRIFT_VELOCITY_THRESHOLD)
        {
            // Slowly adapt gyro Z offset
            gyroOffsetZ = (1.0f - GYRO_DRIFT_ALPHA) * gyroOffsetZ +
                           GYRO_DRIFT_ALPHA * (float)rawGyroZ;
        }
#endif
    }

    // Reset angle estimator state
    void reset()
    {
        angle = 0;
        angleRate = 0;
        lastUpdateTime = micros();
    }

private:
    Zumo32U4IMU* imu;
    float angle;          // Current angle estimate (degrees)
    float angleRate;      // Current angular velocity (degrees/s)
    float gyroOffset;     // Gyro Y-axis bias (pitch)
    float gyroOffsetZ;    // Gyro Z-axis bias (yaw/heading)
    uint32_t lastUpdateTime;
    float accelAngle;     // Last accelerometer-derived angle
};
