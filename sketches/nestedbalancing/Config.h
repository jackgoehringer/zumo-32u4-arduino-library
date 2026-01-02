// Copyright Pololu Corporation.  For more information, see http://www.pololu.com/
#pragma once

/*
 * Config.h - Tunable parameters for cascaded PID balancing
 *
 * This file contains all configuration constants that may need tuning
 * for your specific Zumo 32U4 robot.
 */

// =============================================================================
// HARDWARE CONFIGURATION
// =============================================================================

// Set to true for OLED version, false for LCD version
// The different display weights affect the balance point
#define ZUMO_OLED_VERSION false

// Encoder counts per millimeter of travel
// Zumo 32U4: 12 CPR motor encoder * 75.81:1 gearbox / (32mm wheel * PI)
// = 909.7 counts per revolution / 100.5mm circumference = 9.05 counts/mm
#define COUNTS_PER_MM 9.549f

// Gyro sensitivity: 0.07 dps per digit (from L3GD20H / LSM6DS33 datasheet)
#define GYRO_SENSITIVITY 0.07f

// =============================================================================
// LOOP TIMING (in microseconds)
// =============================================================================

// Inner loop (angle control) - runs as fast as possible with gyro polling
// Target: ~200 Hz (5000 us), actual rate depends on gyro data ready
#define INNER_LOOP_PERIOD_US 5000

// Middle loop (velocity control) - runs at encoder update rate
// Target: 50 Hz (20000 us)
#define MIDDLE_LOOP_PERIOD_US 20000

// Outer loop (position control) - runs slower for smooth tracking
// Target: 20 Hz (50000 us)
#define OUTER_LOOP_PERIOD_US 50000

// Display update period (don't update too often, it's slow)
// Target: 10 Hz (100000 us)
#define DISPLAY_UPDATE_PERIOD_US 100000

// =============================================================================
// ANGLE ESTIMATOR (Complementary Filter)
// =============================================================================

// Base balance angle in degrees
// TUNE THIS FIRST! Manually balance robot and read angle from display.
// Set this to the angle shown when robot is balanced.
// OLED version is heavier on top, needs more forward lean

#define BALANCE_ANGLE 0.88f      // STEP 1: Find this value first!

// Complementary filter accelerometer weight
// Higher = trusts accelerometer more (reduces gyro drift)
// Lower = trusts gyro more (better dynamic response)
// Range: 0.0 to 1.0, typical: 0.01 to 0.05
#define ACCEL_WEIGHT_MAX 0.02f

// Accelerometer trust threshold
// Reduce accelerometer weight when magnitude differs from 1g
#define ACCEL_TRUST_FACTOR 5.0f

// =============================================================================
// INNER LOOP - ANGLE PID CONTROLLER
// =============================================================================

// These gains control how aggressively the robot responds to angle errors
// Higher Kp = faster response but may oscillate
// Higher Kd = more damping, reduces oscillation
// Higher Ki = eliminates steady-state error but may cause windup
//
// TUNING ORDER: Kp first (alone), then Kd, then Ki last
// Start with Ki=0, tune Kp until slight oscillation, back off 40%
// Add Kd to dampen oscillation, then add small Ki if needed

#define ANGLE_KP 65.0f       // STEP 2: Start at 20, increase until oscillation
#define ANGLE_KI 0.0f       // STEP 4: Usually not needed, add last if robot leans
#define ANGLE_KD 0.9f       // STEP 3: Add after Kp tuned, damps oscillation

// Integral windup limits (in degrees * seconds)
#define ANGLE_INTEGRAL_MIN -40.0f
#define ANGLE_INTEGRAL_MAX 40.0f

// Output limits (motor speed units, -400 to 400)
#define ANGLE_OUTPUT_MIN -400.0f
#define ANGLE_OUTPUT_MAX 400.0f

// =============================================================================
// MIDDLE LOOP - VELOCITY PID CONTROLLER
// =============================================================================

// These gains control how the robot maintains target velocity
// Output is an angle offset added to BALANCE_ANGLE
//
// TUNING: Only tune AFTER inner angle loop is stable!
// Start with all zeros, then add Kp until drift stops
// Add Ki only if slow drift persists

// Velocity in encoder counts per second
#define VELOCITY_KP 0.002f    // STEP 5: Start at 0.0005, increase until drift stops
#define VELOCITY_KI 0.0001f    // STEP 6: Add if slow drift persists (start 0.0001)
#define VELOCITY_KD 0.0f    // Usually not needed

// Integral windup limits (in (counts/s) * seconds)
#define VELOCITY_INTEGRAL_MIN -1000.0f
#define VELOCITY_INTEGRAL_MAX 1000.0f

// Maximum angle offset from velocity controller (degrees)
// Limits how far the robot will tilt to achieve target velocity
#define VELOCITY_OUTPUT_MIN -15.0f
#define VELOCITY_OUTPUT_MAX 15.0f

// =============================================================================
// OUTER LOOP - POSITION PID CONTROLLER
// =============================================================================

// These gains control how the robot moves to target position
// Output is a target velocity in counts per second
//
// TUNING: Only tune AFTER velocity loop is stable!
// Usually only Kp is needed, Ki/Kd often cause problems

#define POSITION_KP -1.0f    // STEP 7: Start at 0.3, increase for faster return
#define POSITION_KI 0.0f    // Usually leave at 0
#define POSITION_KD 0.0f    // Usually leave at 0

// Integral windup limits
#define POSITION_INTEGRAL_MIN -500.0f
#define POSITION_INTEGRAL_MAX 500.0f

// Maximum velocity from position controller (counts per second)
// ~500 counts/s = ~55 mm/s
#define POSITION_OUTPUT_MIN -250.0f
#define POSITION_OUTPUT_MAX 250.0f

// =============================================================================
// TURN CONTROLLER (Heading/Yaw)
// =============================================================================

// PD controller for turning while balancing
// Output is differential motor speed (added to left, subtracted from right)

#define TURN_KP 3.0f
#define TURN_KD 0.05f

// Maximum turn rate in degrees per second
#define MAX_TURN_RATE 180.0f

// Maximum differential motor speed for turning
#define MAX_TURN_SPEED 100.0f

// =============================================================================
// SAFETY LIMITS
// =============================================================================

// If angle exceeds this, robot is falling - stop motors
#define FALL_ANGLE_THRESHOLD 45.0f

// Angle at which we consider the robot recovered and ready to balance
#define RECOVERY_ANGLE_THRESHOLD 5.0f

// Maximum motor speed (hardware limit is 400)
#define MAX_MOTOR_SPEED 400

// =============================================================================
// MOTION COMMAND LIMITS
// =============================================================================

// Maximum velocity that can be commanded (counts per second)
// ~1000 counts/s = ~110 mm/s
#define MAX_COMMAND_VELOCITY 1000.0f

// Maximum position command per move (counts)
// ~9050 counts = 1 meter
#define MAX_COMMAND_DISTANCE 9050.0f

// Default velocity for position moves (counts per second)
#define DEFAULT_MOVE_VELOCITY 100.0f

// =============================================================================
// GYRO CALIBRATION
// =============================================================================

// Number of samples for initial gyro calibration
#define GYRO_CALIBRATION_SAMPLES 1024

// Enable continuous gyro drift compensation
#define ENABLE_GYRO_DRIFT_COMPENSATION true

// Threshold for detecting "quiet" periods for drift compensation
#define GYRO_DRIFT_RATE_THRESHOLD 1.0f      // degrees per second
#define GYRO_DRIFT_VELOCITY_THRESHOLD 10.0f // counts per second

// Drift compensation filter coefficient (smaller = slower adaptation)
#define GYRO_DRIFT_ALPHA 0.001f
