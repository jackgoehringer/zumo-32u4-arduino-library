// Copyright Pololu Corporation.  For more information, see http://www.pololu.com/
#pragma once

#include <Arduino.h>
#include <Zumo32U4Encoders.h>
#include "Config.h"
#include "PIDController.h"
#include "MotionProfile.h"

/*
 * MotionController.h - Outer loop controllers for velocity and position
 *
 * This class manages:
 * - Encoder-based position and velocity tracking
 * - Trapezoidal motion profiles for smooth move/turn execution
 * - Velocity control loop with feedforward (outputs target angle offset)
 * - Position tracking loop (outputs target velocity)
 * - Heading/turn control with feedforward and friction compensation
 *
 * Control hierarchy (with profiles):
 *   Motion Profile -> Position Tracking + Feedforward -> Velocity Loop -> Angle Loop
 *   Turn Profile -> Heading PID + Rate Feedforward + Friction FF -> Motor Differential
 *
 * The motion profiles plan smooth acceleration/cruise/deceleration trajectories
 * so the PID controllers only need to correct small tracking errors, not drive
 * the entire motion from scratch. Friction feedforward provides minimum effort
 * to overcome Coulomb friction of the tracks.
 */

class MotionController
{
public:
    MotionController()
        : position(0), velocity(0),
          targetPosition(0), targetVelocity(0),
          targetHeading(0), targetHeadingEnc(0), heading(0), headingEnc(0), headingRate(0),
          arcModeEnabled(false), arcTargetAngle(0), arcCompletedAngle(0),
          arcTargetLengthMM(0), arcCompletedLengthMM(0), arcTurnRadiusMM(0),
          arcTightness(0), arcSpeedMMps(0),
          lastEncoderLeft(0), lastEncoderRight(0), lastDeltaLeft(0), lastDeltaRight(0), encoderDiffAccum(0),
          lastVelocityUpdateTime(0),
          turnCommandActive(false), headingSettleCounter(0), lastTurnOutput(0),
          positionControlEnabled(true), velocityControlEnabled(true),
          headingControlEnabled(true),
          moveProfileActive(false), turnProfileActive(false), turnSettling(false),
          moveStartPosition(0), turnStartHeading(0), moveSettleCounter(0),
          moveFrictionFF(0)
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
        // Use derivative on error (not measurement) so the D term doesn't
        // fight the feedforward during profile tracking. With a smoothly
        // moving setpoint, d(error)/dt is near zero during good tracking
        // and provides damping only when the robot deviates from the plan.
        headingPID.setGains(TURN_KP, TURN_KI, TURN_KD);
        headingPID.setOutputLimits(TURN_OUTPUT_MIN, TURN_OUTPUT_MAX);
        headingPID.setIntegralLimits(TURN_INTEGRAL_MIN, TURN_INTEGRAL_MAX);
        headingPID.setDerivativeOnMeasurement(false);
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
        targetHeadingEnc = 0;
        heading = 0;
        headingEnc = 0;
        turnCommandActive = false;
        headingSettleCounter = 0;
        lastTurnOutput = 0;

        // Arc state
        arcModeEnabled = false;
        arcTargetAngle = 0;
        arcCompletedAngle = 0;
        arcTargetLengthMM = 0;
        arcCompletedLengthMM = 0;
        arcTurnRadiusMM = 0;
        arcTightness = 0;
        arcSpeedMMps = 0;

        // Profile state
        moveProfile.stop();
        turnProfile.stop();
        moveProfileActive = false;
        turnProfileActive = false;
        turnSettling = false;
        moveStartPosition = 0;
        turnStartHeading = 0;
        moveSettleCounter = 0;
        moveFrictionFF = 0;

        // Reset encoder tracking
        lastEncoderLeft = encoders->getCountsLeft();
        lastEncoderRight = encoders->getCountsRight();
        lastDeltaLeft = 0;
        lastDeltaRight = 0;
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
        lastDeltaLeft = deltaLeft;
        lastDeltaRight = deltaRight;

        // Average of both wheels for forward motion
        int16_t deltaAvg = (deltaLeft + deltaRight) / 2;

        // Update position (32-bit to avoid overflow)
        position += deltaAvg;

        // Track encoder difference for heading correction
        // Positive difference means left wheel moved more = veering right
        // This accumulates over time to detect sustained drift
        encoderDiffAccum += (deltaLeft - deltaRight);

        // Encoder-based heading estimate (degrees)
        // deltaTheta = (deltaRight - deltaLeft) / trackWidth
        float deltaLeftMM = (float)deltaLeft / COUNTS_PER_MM;
        float deltaRightMM = (float)deltaRight / COUNTS_PER_MM;
        float deltaThetaRad = (deltaRightMM - deltaLeftMM) / WHEEL_TRACK_MM;
        headingEnc += deltaThetaRad * 180.0f / (float)M_PI;

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
        turnCommandActive = false;
        headingSettleCounter = 0;
        lastTurnOutput = 0;

        // Stop any active profiles
        moveProfile.stop();
        turnProfile.stop();
        moveProfileActive = false;
        turnProfileActive = false;
        turnSettling = false;
        moveFrictionFF = 0;

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

        // Disable position and velocity control during arc
        // Arc turns use feedforward (ARC_BASE_EFFORT) instead of velocity PID
        // to avoid conflicts between the two control methods
        positionControlEnabled = false;
        velocityControlEnabled = false;
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
    float getArcTargetAngle() const { return arcTargetAngle; }
    float getArcTurnRadiusMM() const { return arcTurnRadiusMM; }

    // Compute arc wheel speed differential based on geometry
    // Returns the speed ratio for inner and outer wheels
    // innerRatio: multiplier for inner wheel speed (< 1.0)
    // outerRatio: multiplier for outer wheel speed (> 1.0)
    // For CCW turn (positive angle): left is inner, right is outer
    // For CW turn (negative angle): right is inner, left is outer
    void getArcWheelRatios(float& leftRatio, float& rightRatio) const
    {
        if (!arcModeEnabled || arcTurnRadiusMM <= 0)
        {
            leftRatio = 1.0f;
            rightRatio = 1.0f;
            return;
        }

        // Compute inner/outer wheel ratios based on turn radius and track width
        // Inner wheel travels shorter arc: radius - halfTrack
        // Outer wheel travels longer arc: radius + halfTrack
        float innerRatio = (arcTurnRadiusMM - HALF_TRACK_MM) / arcTurnRadiusMM;
        float outerRatio = (arcTurnRadiusMM + HALF_TRACK_MM) / arcTurnRadiusMM;

        // Clamp inner ratio to prevent negative speeds (very tight turns)
        innerRatio = max(0.1f, innerRatio);

        // Assign based on turn direction
        if (arcTargetAngle > 0)  // CCW turn: left is inner, right is outer
        {
            leftRatio = innerRatio;
            rightRatio = outerRatio;
        }
        else  // CW turn: right is inner, left is outer
        {
            leftRatio = outerRatio;
            rightRatio = innerRatio;
        }
    }

    // Compute base wheel speeds for arc turn (in motor speed units, not counts/s)
    // This provides a feedforward drive signal based on the commanded arc speed,
    // independent of the balance correction.
    void getArcBaseWheelSpeeds(float& leftBase, float& rightBase) const
    {
        if (!arcModeEnabled || arcTurnRadiusMM <= 0)
        {
            leftBase = 0;
            rightBase = 0;
            return;
        }

        // Convert target velocity (counts/s) to a motor "effort" estimate
        float baseEffort = (targetVelocity / MAX_COMMAND_VELOCITY) * ARC_BASE_EFFORT;

        // Get the wheel ratios
        float leftRatio, rightRatio;
        getArcWheelRatios(leftRatio, rightRatio);

        // Apply ratios to base effort
        leftBase = baseEffort * leftRatio;
        rightBase = baseEffort * rightRatio;
    }

    // Compute velocity loop output (target angle offset)
    // Call this at MIDDLE_LOOP rate
    //
    // When a move profile is active, this method:
    //   1. Updates the profile to get desired {position, velocity, acceleration}
    //   2. Computes a position tracking correction (profile pos vs actual pos)
    //   3. Applies feedforward (velocity + acceleration) + PID feedback
    //
    // When no profile is active, falls back to PID on targetVelocity (set by outer loop)
    float computeVelocityLoop(float dt)
    {
        if (!velocityControlEnabled)
        {
            return 0;
        }

        float effectiveTargetVel;
        float profileAccel = 0;

        if (moveProfileActive)
        {
            if (!moveProfile.isFinished())
            {
                // === Profile running: track the planned trajectory ===
                moveProfile.update(dt);

                // Position tracking: compare profile's planned position to actual
                float profilePos = (float)moveStartPosition + moveProfile.getPosition();
                float posError = profilePos - (float)position;

                // Velocity reference = profile velocity + small correction from position error
                float velCorrection = POSITION_KP * posError;
                effectiveTargetVel = moveProfile.getVelocity() + velCorrection;
                effectiveTargetVel = constrain(effectiveTargetVel,
                                               POSITION_OUTPUT_MIN, POSITION_OUTPUT_MAX);

                profileAccel = moveProfile.getAcceleration();

                // No motor-level friction FF during profile (feedforward handles it)
                moveFrictionFF = 0;
            }
            else
            {
                // === Profile done: settling phase ===
                // Use higher position gain to overcome cascaded attenuation
                float posError = (float)targetPosition - (float)position;
                effectiveTargetVel = POSITION_KP_SETTLE * posError;
                effectiveTargetVel = constrain(effectiveTargetVel,
                                               POSITION_OUTPUT_MIN, POSITION_OUTPUT_MAX);

                // Motor-level friction FF to push through last few mm
                if (fabs(posError) > (float)(MOVE_SETTLE_TOLERANCE / 2))
                {
                    moveFrictionFF = (posError > 0) ? FRICTION_FF_MOVE_SETTLE
                                                     : -FRICTION_FF_MOVE_SETTLE;
                }
                else
                {
                    moveFrictionFF = 0;
                }
            }
        }
        else
        {
            // === No profile: use targetVelocity from outer loop position PID ===
            effectiveTargetVel = targetVelocity;
            moveFrictionFF = 0;
        }

        // Feedforward: proactive effort based on planned motion
        float velFF = VELOCITY_FF_KV * effectiveTargetVel;
        float accelFF = VELOCITY_FF_KA * profileAccel;

        // Feedback: PID corrects tracking errors
        float velFB = velocityPID.compute(effectiveTargetVel, velocity, dt);

        // Combined output (clamped to safe angle offset range)
        float output = velFF + accelFF + velFB;
        output = constrain(output, -ANGLE_OFFSET_MAX, ANGLE_OFFSET_MAX);

        return output;
    }

    // Compute position loop output (target velocity)
    // Call this at OUTER_LOOP rate
    // Only active when no move profile is running (for position holding)
    void computePositionLoop(float dt)
    {
        if (!positionControlEnabled)
        {
            return;
        }

        // When a move profile is active, the velocity loop handles everything
        if (moveProfileActive)
        {
            return;
        }

        // Standard position PID for holding position
        float velocityCommand = positionPID.compute((float)targetPosition, (float)position, dt);

        // During turn commands, ignore small position corrections to avoid fighting yaw
        if (turnCommandActive && fabs(velocityCommand) < TURN_POSITION_VELOCITY_DEADBAND)
        {
            velocityCommand = 0;
        }
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
    // dt: time step in seconds
    // Returns value to subtract from left motor and add to right motor
    //
    // When a turn profile is active:
    //   - Tracks the profile's heading reference with PID
    //   - Adds rate feedforward for the desired rotation speed
    //   - Adds friction feedforward to overcome track friction
    //
    // When the turn profile finishes (settling):
    //   - PID tracks the final heading target
    //   - Settling friction FF provides minimum differential for error > deadband
    //
    // When no turn command is active (heading hold):
    //   - Gyro-based PID + encoder-based correction maintains heading
    //
    // NOTE: During arc mode, this returns 0 because arc turns use geometry-based
    // wheel speed ratios instead of PID-based turn correction.
    float computeTurnOutput(float dt)
    {
        if (!headingControlEnabled)
        {
            return 0;
        }

        // During arc mode, don't use PID-based turn output
        // Arc turns are handled by geometry-based wheel ratios in the inner loop
        if (arcModeEnabled)
        {
            // Still update the heading PID to track progress, but don't use its output
            float headingError = normalizeAngle(targetHeading - heading);
            float effectiveTarget = heading + headingError;
            headingPID.compute(effectiveTarget, heading, dt);
            return 0;
        }

        float totalOutput;

        if (turnProfileActive)
        {
            if (!turnProfile.isFinished())
            {
                // === Turn profile running: track with feedforward ===
                turnProfile.update(dt);

                // Profile reference heading (encoder-based for consistency during turns)
                float profileHeading = turnStartHeading + turnProfile.getPosition();
                float headingError = normalizeAngle(profileHeading - headingEnc);
                float effectiveTarget = headingEnc + headingError;

                // PID feedback (tracks moving profile reference)
                float pidOutput = headingPID.compute(effectiveTarget, headingEnc, dt);

                // Rate feedforward: provides the baseline differential for desired turn speed
                float rateFF = TURN_FF_KV * turnProfile.getVelocity();

                // Friction feedforward: overcomes static friction of tracks
                float frictionFF = 0;
                if (fabs(turnProfile.getVelocity()) > 1.0f)
                {
                    frictionFF = (turnProfile.getVelocity() > 0)
                                 ? FRICTION_FF_TURN : -FRICTION_FF_TURN;
                }

                totalOutput = pidOutput + rateFF + frictionFF;
            }
            else
            {
                // === Turn profile done: settling phase ===
                turnSettling = true;

                float headingError = normalizeAngle(targetHeadingEnc - headingEnc);
                float effectiveTarget = headingEnc + headingError;

                // PID feedback (tracks fixed final heading)
                float pidOutput = headingPID.compute(effectiveTarget, headingEnc, dt);

                // Settling friction feedforward: minimum differential to close the gap
                float frictionFF = 0;
                if (fabs(headingError) > TURN_SETTLE_DEADBAND_DEG)
                {
                    frictionFF = (headingError > 0) ? FRICTION_FF_TURN_SETTLE
                                                     : -FRICTION_FF_TURN_SETTLE;
                }

                totalOutput = pidOutput + frictionFF;
            }

            // Decay encoder diff accumulator during turns (avoid buildup)
            encoderDiffAccum = (int32_t)((float)encoderDiffAccum * 0.7f);
        }
        else
        {
            // === Normal heading hold (no turn command) ===
            // Uses gyro-based heading with encoder correction

            float headingError = normalizeAngle(targetHeading - heading);
            float effectiveTarget = heading + headingError;

            // Gyro-based PID
            float gyroCorrection = headingPID.compute(effectiveTarget, heading, dt);

            // Encoder-based correction (detects wheel speed mismatch)
            float encoderCorrection = ENCODER_HEADING_KP * (float)encoderDiffAccum;
            encoderCorrection = constrain(encoderCorrection,
                                          -ENCODER_HEADING_MAX, ENCODER_HEADING_MAX);

            totalOutput = gyroCorrection + encoderCorrection;
        }

        // Slew-rate limiting: prevents sudden yaw acceleration
        if (dt > 0)
        {
            float maxDelta = TURN_OUTPUT_SLEW_RATE * dt;
            float delta = totalOutput - lastTurnOutput;
            if (delta > maxDelta) { delta = maxDelta; }
            else if (delta < -maxDelta) { delta = -maxDelta; }
            totalOutput = lastTurnOutput + delta;
            lastTurnOutput = totalOutput;
        }

        return constrain(totalOutput, TURN_OUTPUT_MIN, TURN_OUTPUT_MAX);
    }

    // === Command Interface ===

    // Move relative distance in millimeters (uses trapezoidal profile)
    void moveRelativeMM(float distanceMM)
    {
        int32_t counts = (int32_t)(distanceMM * COUNTS_PER_MM);
        targetPosition = position + counts;
        moveStartPosition = position;

        // Start the trapezoidal move profile (in encoder counts)
        moveProfile.start((float)counts, MOVE_PROFILE_MAX_VELOCITY, MOVE_PROFILE_MAX_ACCEL);
        moveProfileActive = true;
        moveSettleCounter = 0;
        moveFrictionFF = 0;

        positionControlEnabled = true;
        velocityControlEnabled = true;
        turnCommandActive = false;
        turnProfile.stop();
        turnProfileActive = false;
        turnSettling = false;
        headingSettleCounter = 0;
        lastTurnOutput = 0;

        // Reset velocity PID for clean profile tracking
        velocityPID.reset();
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

        // Stop all profiles
        moveProfile.stop();
        turnProfile.stop();
        moveProfileActive = false;
        turnProfileActive = false;
        turnSettling = false;
        moveSettleCounter = 0;
        moveFrictionFF = 0;

        // Hold current heading
        targetHeading = heading;
        targetHeadingEnc = headingEnc;
        headingControlEnabled = true;
        turnCommandActive = false;
        headingSettleCounter = 0;
        lastTurnOutput = 0;
    }

    // Turn by a relative angle (degrees, positive = CCW, negative = CW)
    // Uses trapezoidal heading profile for smooth acceleration/deceleration
    void turnRelative(float angleDeg)
    {
        targetHeading = heading + angleDeg;
        targetHeadingEnc = headingEnc + angleDeg;
        turnStartHeading = headingEnc;

        // Start the trapezoidal turn profile (in degrees)
        turnProfile.start(angleDeg, TURN_PROFILE_MAX_VELOCITY, TURN_PROFILE_MAX_ACCEL);
        turnProfileActive = true;
        turnSettling = false;

        headingControlEnabled = true;
        headingPID.reset(headingEnc);
        encoderDiffAccum = 0;
        turnCommandActive = true;
        headingSettleCounter = 0;
        lastTurnOutput = 0;

        // Stop any active move profile
        moveProfile.stop();
        moveProfileActive = false;
        moveFrictionFF = 0;

        // Keep position control active so robot doesn't drift while turning
        positionControlEnabled = true;
        velocityControlEnabled = true;
        targetPosition = position;  // Hold current position
        targetVelocity = 0;
    }

    // Lock in current heading as target (call at start of move commands)
    void holdCurrentHeading()
    {
        targetHeading = heading;
        targetHeadingEnc = headingEnc;
        headingControlEnabled = true;
        headingPID.reset(heading);
        encoderDiffAccum = 0;
        turnCommandActive = false;
        turnProfile.stop();
        turnProfileActive = false;
        turnSettling = false;
        headingSettleCounter = 0;
        lastTurnOutput = 0;
    }

    // Check if a move command is complete (profile finished + settled)
    bool isMoveComplete()
    {
        if (!moveProfileActive)
        {
            return false;
        }

        // Profile must be finished first
        if (!moveProfile.isFinished())
        {
            return false;
        }

        // Check settling: position within tolerance and velocity low
        bool positionClose = abs(position - targetPosition) < MOVE_SETTLE_TOLERANCE;
        bool velocityLow = fabs(velocity) < MOVE_SETTLE_VELOCITY;

        if (positionClose && velocityLow)
        {
            moveSettleCounter++;
        }
        else
        {
            moveSettleCounter = 0;
        }

        return moveSettleCounter >= MOVE_SETTLE_COUNT;
    }

    // Check if a turn command is complete (profile finished + settled)
    bool isTurnComplete()
    {
        if (!turnProfileActive)
        {
            return false;
        }

        // Profile must be finished first
        if (!turnProfile.isFinished())
        {
            return false;
        }

        float error = normalizeAngle(targetHeadingEnc - headingEnc);
        if (fabs(error) < HEADING_TOLERANCE_DEG && fabs(headingRate) < HEADING_RATE_TOLERANCE_DPS)
        {
            if (headingSettleCounter < HEADING_SETTLE_COUNT)
            {
                headingSettleCounter++;
            }
        }
        else
        {
            headingSettleCounter = 0;
        }

        return headingSettleCounter >= HEADING_SETTLE_COUNT;
    }

    bool isTurnCommandActive() const { return turnCommandActive; }
    int16_t getLastDeltaLeft() const { return lastDeltaLeft; }
    int16_t getLastDeltaRight() const { return lastDeltaRight; }

    // === Profile State Getters ===

    bool isMoveProfileActive() const { return moveProfileActive; }
    bool isTurnProfileActive() const { return turnProfileActive; }
    bool isTurnSettling() const { return turnSettling; }

    // Get motor-level friction feedforward for move settling
    // Apply this to both motors in the .ino inner loop
    float getMoveFrictionFF() const { return moveFrictionFF; }

    // === Standard Getters ===

    int32_t getPosition() const { return position; }
    float getVelocity() const { return velocity; }
    float getTargetVelocity() const { return targetVelocity; }
    float getHeading() const { return heading; }
    float getHeadingEnc() const { return headingEnc; }
    float getHeadingRate() const { return headingRate; }
    float getTargetHeading() const { return targetHeading; }

    // Get normalized heading error (for debugging)
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

    // Check if at target position (within tolerance) - legacy, use isMoveComplete() for commands
    bool atTargetPosition(int32_t tolerance = 50) const
    {
        return abs(position - targetPosition) < tolerance;
    }

    // Check if at target heading (within tolerance)
    bool atTargetHeading(float toleranceDeg = HEADING_TOLERANCE_DEG) const
    {
        float error = targetHeading - heading;
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
    float targetHeadingEnc;   // Target heading from encoder yaw (degrees)
    float heading;            // Current heading (degrees)
    float headingEnc;         // Encoder-based heading (degrees)
    float headingRate;        // Current turn rate (degrees/second)

    // Arc turn state
    bool arcModeEnabled;
    float arcTargetAngle;
    float arcCompletedAngle;
    float arcTargetLengthMM;
    float arcCompletedLengthMM;
    float arcTurnRadiusMM;
    float arcTightness;
    float arcSpeedMMps;

    // Motion profiles
    TrapezoidalProfile moveProfile;
    TrapezoidalProfile turnProfile;
    bool moveProfileActive;
    bool turnProfileActive;
    bool turnSettling;
    int32_t moveStartPosition;    // Position when move profile started
    float turnStartHeading;       // Encoder heading when turn profile started
    uint8_t moveSettleCounter;    // Consecutive settled cycles for move
    float moveFrictionFF;         // Current move settling friction feedforward

    // Encoder tracking
    int16_t lastEncoderLeft;
    int16_t lastEncoderRight;
    int16_t lastDeltaLeft;
    int16_t lastDeltaRight;
    int32_t encoderDiffAccum;     // Accumulated left-right encoder difference

    // Timing
    uint32_t lastVelocityUpdateTime;

    // Control enable flags
    bool positionControlEnabled;
    bool velocityControlEnabled;
    bool headingControlEnabled;

    // Turn completion and shaping state
    bool turnCommandActive;
    uint8_t headingSettleCounter;
    float lastTurnOutput;

    // PID controllers
    PIDController velocityPID;
    PIDController positionPID;
    PIDController headingPID;
};
