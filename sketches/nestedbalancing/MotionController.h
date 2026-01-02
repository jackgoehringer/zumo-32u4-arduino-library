// Copyright Pololu Corporation.  For more information, see http://www.pololu.com/
#pragma once

#include <Arduino.h>
#include <Zumo32U4Encoders.h>
#include "Config.h"
#include "PIDController.h"

/*
 * MotionController.h - Outer loop controllers for velocity and position
 *
 * This class manages:
 * - Encoder-based position and velocity tracking
 * - Velocity control loop (outputs target angle offset)
 * - Position control loop (outputs target velocity)
 * - Heading/turn control using gyro Z-axis
 *
 * Control hierarchy:
 *   Position Loop -> Velocity Loop -> Angle Loop (in BalanceController)
 */

class MotionController
{
public:
    MotionController()
        : position(0), velocity(0),
          targetPosition(0), targetVelocity(0),
          targetHeading(0), heading(0), headingRate(0),
          lastEncoderLeft(0), lastEncoderRight(0),
          lastVelocityUpdateTime(0), lastPositionUpdateTime(0),
          positionControlEnabled(true), velocityControlEnabled(true),
          headingControlEnabled(false)
    {
        // Configure velocity PID
        velocityPID.setGains(VELOCITY_KP, VELOCITY_KI, VELOCITY_KD);
        velocityPID.setOutputLimits(VELOCITY_OUTPUT_MIN, VELOCITY_OUTPUT_MAX);
        velocityPID.setIntegralLimits(VELOCITY_INTEGRAL_MIN, VELOCITY_INTEGRAL_MAX);

        // Configure position PID
        positionPID.setGains(POSITION_KP, POSITION_KI, POSITION_KD);
        positionPID.setOutputLimits(POSITION_OUTPUT_MIN, POSITION_OUTPUT_MAX);
        positionPID.setIntegralLimits(POSITION_INTEGRAL_MIN, POSITION_INTEGRAL_MAX);
    }

    // Initialize with encoder reference
    void begin(Zumo32U4Encoders* encodersPtr)
    {
        encoders = encodersPtr;
        reset();
    }

    // Reset all state (call when starting to balance)
    void reset()
    {
        position = 0;
        velocity = 0;
        targetPosition = 0;
        targetVelocity = 0;
        targetHeading = 0;
        heading = 0;

        // Reset encoder tracking
        lastEncoderLeft = encoders->getCountsLeft();
        lastEncoderRight = encoders->getCountsRight();

        lastVelocityUpdateTime = micros();
        lastPositionUpdateTime = micros();

        velocityPID.reset();
        positionPID.reset();
    }

    // Update encoder readings and compute velocity
    // Call this at MIDDLE_LOOP rate (~50 Hz)
    void updateEncoders()
    {
        uint32_t now = micros();
        uint32_t dt = now - lastVelocityUpdateTime;
        lastVelocityUpdateTime = now;

        // Read encoder counts
        int16_t leftCount = encoders->getCountsLeft();
        int16_t rightCount = encoders->getCountsRight();

        // Compute delta (handles 16-bit wraparound correctly)
        int16_t deltaLeft = leftCount - lastEncoderLeft;
        int16_t deltaRight = rightCount - lastEncoderRight;

        lastEncoderLeft = leftCount;
        lastEncoderRight = rightCount;

        // Average of both wheels for forward motion
        int16_t deltaAvg = (deltaLeft + deltaRight) / 2;

        // Update position (32-bit to avoid overflow)
        position += deltaAvg;

        // Compute velocity in counts per second
        if (dt > 0)
        {
            velocity = (float)deltaAvg * 1000000.0f / (float)dt;
        }
    }

    // Update heading from gyro Z-axis
    // gyroZ: raw gyro Z reading
    // gyroOffsetZ: calibrated Z offset
    // dt: time step in seconds
    void updateHeading(int16_t gyroZ, float gyroOffsetZ, float dt)
    {
        headingRate = ((float)gyroZ - gyroOffsetZ) * GYRO_SENSITIVITY;
        heading += headingRate * dt;
    }

    // Compute velocity loop output (target angle offset)
    // Call this at MIDDLE_LOOP rate
    float computeVelocityLoop(float dt)
    {
        if (!velocityControlEnabled)
        {
            return 0;
        }

        // Velocity PID: error = targetVelocity - actualVelocity
        // Output: angle offset to add to BALANCE_ANGLE
        //
        // Sign convention for this robot:
        // - Forward motion: velocity +, angle - (tilted back)
        // - To slow from forward: need angle offset + (lean forward to catch up)
        // - PID output for positive error is positive
        // - But we have negative error (0 - positive vel), so PID gives negative
        // - We need positive, so NEGATE the output
        return -velocityPID.compute(targetVelocity, velocity, dt);
    }

    // Compute position loop output (target velocity)
    // Call this at OUTER_LOOP rate
    void computePositionLoop(float dt)
    {
        if (!positionControlEnabled)
        {
            return;
        }

        // Position PID: error = targetPosition - actualPosition
        // Output: target velocity for velocity loop
        //
        // If robot is ahead of target (positive position, target=0):
        // - error is negative → negative velocity command
        // - We want robot to move backward (negative velocity) to return
        // - So sign is correct here, no negation needed
        float velocityCommand = positionPID.compute(targetPosition, (float)position, dt);
        targetVelocity = velocityCommand;
    }

    // Compute turn output (differential motor speed)
    // Returns value to add to left motor and subtract from right motor
    float computeTurnOutput()
    {
        if (!headingControlEnabled)
        {
            return 0;
        }

        float headingError = targetHeading - heading;

        // Simple PD control for turning
        float turnOutput = TURN_KP * headingError - TURN_KD * headingRate;

        return constrain(turnOutput, -MAX_TURN_SPEED, MAX_TURN_SPEED);
    }

    // === Command Interface ===

    // Set target position (absolute, in encoder counts)
    void setTargetPosition(int32_t pos)
    {
        targetPosition = pos;
        positionControlEnabled = true;
    }

    // Move relative to current position (in encoder counts)
    void moveRelative(int32_t delta)
    {
        targetPosition = position + delta;
        positionControlEnabled = true;
    }

    // Move relative distance in millimeters
    void moveRelativeMM(float distanceMM)
    {
        int32_t counts = (int32_t)(distanceMM * COUNTS_PER_MM);
        moveRelative(counts);
    }

    // Set target velocity directly (disables position control)
    void setTargetVelocity(float vel)
    {
        targetVelocity = constrain(vel, -MAX_COMMAND_VELOCITY, MAX_COMMAND_VELOCITY);
        positionControlEnabled = false;
        velocityControlEnabled = true;
    }

    // Set target velocity in mm/s
    void setTargetVelocityMM(float mmPerSec)
    {
        setTargetVelocity(mmPerSec * COUNTS_PER_MM);
    }

    // Stop and hold current position
    void stop()
    {
        targetPosition = position;
        targetVelocity = 0;
        positionControlEnabled = true;
        velocityControlEnabled = true;
    }

    // Turn to absolute heading (degrees)
    void setTargetHeading(float degrees)
    {
        targetHeading = degrees;
        headingControlEnabled = true;
    }

    // Turn relative to current heading (degrees)
    void turnRelative(float degrees)
    {
        targetHeading = heading + degrees;
        headingControlEnabled = true;
    }

    // Disable heading control (for manual turning)
    void disableHeadingControl()
    {
        headingControlEnabled = false;
    }

    // === Getters ===

    int32_t getPosition() const { return position; }
    float getVelocity() const { return velocity; }
    float getTargetVelocity() const { return targetVelocity; }
    int32_t getTargetPosition() const { return targetPosition; }
    float getHeading() const { return heading; }
    float getHeadingRate() const { return headingRate; }
    float getTargetHeading() const { return targetHeading; }

    // Get position in millimeters
    float getPositionMM() const { return (float)position / COUNTS_PER_MM; }

    // Get velocity in mm/s
    float getVelocityMM() const { return velocity / COUNTS_PER_MM; }

    // Check if at target position (within tolerance)
    bool atTargetPosition(int32_t tolerance = 50) const
    {
        return abs(position - targetPosition) < tolerance;
    }

    // Check if at target heading (within tolerance)
    bool atTargetHeading(float toleranceDeg = 2.0f) const
    {
        return fabs(heading - targetHeading) < toleranceDeg;
    }

    // === Configuration ===

    void enablePositionControl(bool enable) { positionControlEnabled = enable; }
    void enableVelocityControl(bool enable) { velocityControlEnabled = enable; }

    // Access to PID controllers for tuning
    PIDController& getVelocityPID() { return velocityPID; }
    PIDController& getPositionPID() { return positionPID; }

private:
    Zumo32U4Encoders* encoders;

    // State
    int32_t position;         // Current position (encoder counts)
    float velocity;           // Current velocity (counts/second)
    int32_t targetPosition;   // Target position (encoder counts)
    float targetVelocity;     // Target velocity (counts/second)

    // Heading state
    float targetHeading;      // Target heading (degrees)
    float heading;            // Current heading (degrees)
    float headingRate;        // Current turn rate (degrees/second)

    // Encoder tracking
    int16_t lastEncoderLeft;
    int16_t lastEncoderRight;

    // Timing
    uint32_t lastVelocityUpdateTime;
    uint32_t lastPositionUpdateTime;

    // Control enable flags
    bool positionControlEnabled;
    bool velocityControlEnabled;
    bool headingControlEnabled;

    // PID controllers
    PIDController velocityPID;
    PIDController positionPID;
};
