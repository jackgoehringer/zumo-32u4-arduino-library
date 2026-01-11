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
          arcModeEnabled(false), arcTargetAngle(0), arcCompletedAngle(0),
          arcTargetLengthMM(0), arcCompletedLengthMM(0), arcTurnRadiusMM(0),
          arcTightness(0), arcSpeedMMps(0),
          lastEncoderLeft(0), lastEncoderRight(0), encoderDiffAccum(0),
          lastVelocityUpdateTime(0),
          positionControlEnabled(true), velocityControlEnabled(true),
          headingControlEnabled(true)  // heading enabled by default
    {
        // Configure velocity PID
        velocityPID.setGains(VELOCITY_KP, VELOCITY_KI, VELOCITY_KD);
        velocityPID.setOutputLimits(VELOCITY_OUTPUT_MIN, VELOCITY_OUTPUT_MAX);
        velocityPID.setIntegralLimits(VELOCITY_INTEGRAL_MIN, VELOCITY_INTEGRAL_MAX);

        // Configure position PID
        positionPID.setGains(POSITION_KP, POSITION_KI, POSITION_KD);
        positionPID.setOutputLimits(POSITION_OUTPUT_MIN, POSITION_OUTPUT_MAX);
        positionPID.setIntegralLimits(POSITION_INTEGRAL_MIN, POSITION_INTEGRAL_MAX);

        // Configure heading PID
        headingPID.setGains(TURN_KP, TURN_KI, TURN_KD);
        headingPID.setOutputLimits(TURN_OUTPUT_MIN, TURN_OUTPUT_MAX);
        headingPID.setIntegralLimits(TURN_INTEGRAL_MIN, TURN_INTEGRAL_MAX);
        headingPID.setDerivativeOnMeasurement(true);  // Use gyro rate directly
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

        // Arc state
        arcModeEnabled = false;
        arcTargetAngle = 0;
        arcCompletedAngle = 0;
        arcTargetLengthMM = 0;
        arcCompletedLengthMM = 0;
        arcTurnRadiusMM = 0;
        arcTightness = 0;
        arcSpeedMMps = 0;

        // Reset encoder tracking
        lastEncoderLeft = encoders->getCountsLeft();
        lastEncoderRight = encoders->getCountsRight();
        encoderDiffAccum = 0;

        lastVelocityUpdateTime = micros();

        // Reset control flags - heading enabled by default
        positionControlEnabled = true;
        velocityControlEnabled = true;
        headingControlEnabled = true;

        velocityPID.reset();
        positionPID.reset();
        headingPID.reset();
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

        // Track encoder difference for heading correction
        // Positive difference means left wheel moved more = veering right
        // This accumulates over time to detect sustained drift
        encoderDiffAccum += (deltaLeft - deltaRight);

        // Track arc distance if in arc mode
        if (arcModeEnabled)
        {
            arcCompletedLengthMM += (float)deltaAvg / COUNTS_PER_MM;
        }

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
    // Note: Negated so counterclockwise rotation = positive heading change
    // (standard math convention, matches how turnOutput is applied to motors)
    void updateHeading(int16_t gyroZ, float gyroOffsetZ, float dt)
    {
        headingRate = -((float)gyroZ - gyroOffsetZ) * GYRO_SENSITIVITY;
        heading += headingRate * dt;
    }

    // Start an arc turn (tightness 0..1, speed mm/s, angle degrees)
    void startArcTurn(float tightness, float speedMMps, float angleDeg)
    {
        arcModeEnabled = true;
        arcTightness = constrain(tightness, 0.0f, 1.0f);
        arcSpeedMMps = speedMMps;
        arcTargetAngle = angleDeg;
        arcCompletedAngle = 0;
        arcCompletedLengthMM = 0;

        // Compute turn radius
        if (arcTightness <= 0.0f)
        {
            arcTurnRadiusMM = 1e9f;  // effectively straight
        }
        else
        {
            arcTurnRadiusMM = HALF_TRACK_MM * (1.0f + arcTightness) / arcTightness;
        }

        // Target arc length (if tightness > 0)
        arcTargetLengthMM = (float)fabs(angleDeg) * (float)M_PI / 180.0f * arcTurnRadiusMM;

        // Disable position control during arc; keep velocity + heading
        positionControlEnabled = false;
        velocityControlEnabled = true;
        headingControlEnabled = true;

        // Set heading target to final desired heading
        targetHeading = heading + angleDeg;
        headingPID.reset(heading);

        // Reset velocity PID to ensure clean startup (prevents integral windup issues)
        velocityPID.reset();

        // Reset encoder diff and set target velocity
        encoderDiffAccum = 0;
        float targetVelCounts = speedMMps * COUNTS_PER_MM;
        targetVelocity = constrain(targetVelCounts, -MAX_COMMAND_VELOCITY, MAX_COMMAND_VELOCITY);
    }

    // Update arc progress using gyro angle delta (degrees) and distance delta (mm)
    void updateArcProgress(float deltaAngleDeg, float deltaLengthMM)
    {
        if (!arcModeEnabled)
        {
            return;
        }

        arcCompletedAngle += deltaAngleDeg;
        arcCompletedLengthMM += deltaLengthMM;
    }

    // Check if arc turn is complete
    bool isArcComplete() const
    {
        if (!arcModeEnabled)
        {
            return false;
        }

        bool angleDone = fabs(arcCompletedAngle) >= (fabs(arcTargetAngle) - ARC_ANGLE_TOLERANCE_DEG);
        bool lengthDone = fabs(arcCompletedLengthMM) >= (arcTargetLengthMM - ARC_LENGTH_TOLERANCE_MM);
        return angleDone || lengthDone;
    }

    bool isArcModeEnabled() const { return arcModeEnabled; }
    float getArcSpeedMMps() const { return arcSpeedMMps; }

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
        // Empirically determined sign convention:
        // - Positive angleOffset → robot leans forward → accelerates forward
        // - Negative angleOffset → robot leans backward → decelerates/reverses
        //
        // To SLOW DOWN from forward motion (positive velocity):
        // - We need negative angleOffset (lean backward)
        // - PID gives: error = 0 - (+velocity) = negative → negative output
        // - Negative output directly gives negative angleOffset (correct!)
        // - Therefore NO negation needed
        return velocityPID.compute(targetVelocity, velocity, dt);
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

    // Normalize angle to [-180, 180] range
    float normalizeAngle(float angle)
    {
        while (angle > 180.0f) angle -= 360.0f;
        while (angle < -180.0f) angle += 360.0f;
        return angle;
    }

    // Compute turn output (differential motor speed)
    // dt: time step in seconds (for integral term)
    // Returns value to subtract from left motor and add to right motor
    // 
    // Uses HYBRID heading control:
    // 1. Gyro-based: PID on heading error (corrects absolute heading drift)
    // 2. Encoder-based: P on encoder difference (corrects wheel speed mismatch)
    float computeTurnOutput(float dt)
    {
        if (!headingControlEnabled)
        {
            return 0;
        }

        // === Gyro-based correction (absolute heading) ===
        // Normalize heading error to [-180, 180] so robot takes shortest path
        float headingError = normalizeAngle(targetHeading - heading);
        
        // Compute effective target for PID (current heading + normalized error)
        float effectiveTarget = heading + headingError;
        
        // Gyro-based PID output
        float gyroCorrection = headingPID.compute(effectiveTarget, heading, dt);

        // === Encoder-based correction (wheel speed mismatch) ===
        // Positive encoderDiffAccum means left wheel moved more = robot veered right
        // To correct, we need positive turnOutput (right motor faster = turn left)
        // So the sign is correct: positive diff -> positive correction
        float encoderCorrection = ENCODER_HEADING_KP * (float)encoderDiffAccum;
        encoderCorrection = constrain(encoderCorrection, -ENCODER_HEADING_MAX, ENCODER_HEADING_MAX);

        // === Combine both corrections ===
        float totalOutput = gyroCorrection + encoderCorrection;
        
        return constrain(totalOutput, TURN_OUTPUT_MIN, TURN_OUTPUT_MAX);
    }

    // === Command Interface ===

    // Move relative distance in millimeters
    void moveRelativeMM(float distanceMM)
    {
        int32_t counts = (int32_t)(distanceMM * COUNTS_PER_MM);
        targetPosition = position + counts;
        positionControlEnabled = true;
    }

    // Stop and hold current position
    void stop()
    {
        targetPosition = position;
        targetVelocity = 0;
        positionControlEnabled = true;
        velocityControlEnabled = true;
        arcModeEnabled = false;
        arcTargetAngle = 0;
        arcCompletedAngle = 0;
        arcTargetLengthMM = 0;
        arcCompletedLengthMM = 0;
        // Hold current heading
        targetHeading = heading;
        headingControlEnabled = true;
    }

    // Lock in current heading as target (call at start of move commands)
    void holdCurrentHeading()
    {
        targetHeading = heading;
        headingControlEnabled = true;
        headingPID.reset(heading);  // Reset with current measurement to avoid derivative spike
        encoderDiffAccum = 0;       // Reset encoder difference tracking
    }

    // === Getters ===

    int32_t getPosition() const { return position; }
    float getVelocity() const { return velocity; }
    float getTargetVelocity() const { return targetVelocity; }
    float getHeading() const { return heading; }
    float getHeadingRate() const { return headingRate; }
    float getTargetHeading() const { return targetHeading; }
    
    // Get normalized heading error (for debugging)
    // Positive = need to turn CCW, Negative = need to turn CW
    float getHeadingError() const
    {
        float error = targetHeading - heading;
        while (error > 180.0f) error -= 360.0f;
        while (error < -180.0f) error += 360.0f;
        return error;
    }

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
    // Uses normalized error so 359° and 1° are considered close
    bool atTargetHeading(float toleranceDeg = HEADING_TOLERANCE_DEG) const
    {
        float error = targetHeading - heading;
        // Normalize to [-180, 180]
        while (error > 180.0f) error -= 360.0f;
        while (error < -180.0f) error += 360.0f;
        return fabs(error) < toleranceDeg;
    }

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

    // Arc turn state
    bool arcModeEnabled;
    float arcTargetAngle;         // Total angle to turn (degrees)
    float arcCompletedAngle;      // Angle turned so far (degrees)
    float arcTargetLengthMM;      // Target arc length to travel (mm)
    float arcCompletedLengthMM;   // Arc length traveled so far (mm)
    float arcTurnRadiusMM;        // Current turn radius (mm)
    float arcTightness;           // 0-1 tightness factor
    float arcSpeedMMps;           // Forward speed during arc (mm/s)

    // Encoder tracking
    int16_t lastEncoderLeft;
    int16_t lastEncoderRight;
    int32_t encoderDiffAccum;     // Accumulated left-right encoder difference

    // Timing
    uint32_t lastVelocityUpdateTime;

    // Control enable flags
    bool positionControlEnabled;
    bool velocityControlEnabled;
    bool headingControlEnabled;

    // PID controllers
    PIDController velocityPID;
    PIDController positionPID;
    PIDController headingPID;
};
