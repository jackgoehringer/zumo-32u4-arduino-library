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

// Encoder counts per millimeter of travel
// Zumo 32U4: 12 CPR motor encoder * 75.81:1 gearbox / (32mm wheel * PI)
// = 909.7 counts per revolution / 100.5mm circumference = 9.05 counts/mm
#define COUNTS_PER_MM 9.549f

// Gyro sensitivity: 0.07 dps per digit (from L3GD20H / LSM6DS33 datasheet)
#define GYRO_SENSITIVITY 0.07f

// =============================================================================
// LOOP TIMING (in microseconds)
// =============================================================================

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

#define BALANCE_ANGLE 1.4f      // STEP 1: Find this value first!

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

#define ANGLE_KP 45.0f       // STEP 2: Start at 20, increase until oscillation
#define ANGLE_KI 0.00f       // STEP 4: Usually not needed, add last if robot leans
#define ANGLE_KD 0.9f       // STEP 3: Add after Kp tuned damps oscillation

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
#define VELOCITY_KP 0.0035f    // STEP 5: Start at 0.0005, increase until drift stops
#define VELOCITY_KI 0.00015f    // STEP 6: Add if slow drift persists (start 0.0001)
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

#define POSITION_KP 1.31f     // STEP 7: Start at 0.3, increase for faster return
#define POSITION_KI 0.0f    // Usually leave at 0
#define POSITION_KD 0.0f    // Usually leave at 0

// Integral windup limits
#define POSITION_INTEGRAL_MIN -500.0f
#define POSITION_INTEGRAL_MAX 500.0f

// Maximum velocity from position controller (counts per second)
// ~500 counts/s = ~55 mm/s
#define POSITION_OUTPUT_MIN -2000.0f
#define POSITION_OUTPUT_MAX 2000.0f

// =============================================================================
// MOTION PROFILES (Trapezoidal Velocity Profiles)
// =============================================================================
// Instead of step-changing the setpoint and relying on PID to chase it,
// motion profiles plan smooth acceleration -> cruise -> deceleration
// trajectories. This ensures the controller always has enough authority
// to execute the motion, and prevents the "last mile" stall problem.

// Move profile constraints (in encoder counts)
// Max velocity: ~157 mm/s at 1500 counts/s
// Max accel: moderate ramp, reaches max velocity in 0.5s
#define MOVE_PROFILE_MAX_VELOCITY    1500.0f   // counts/s
#define MOVE_PROFILE_MAX_ACCEL       3000.0f   // counts/s^2

// Turn profile constraints (in degrees)
// Max turn rate: 120 deg/s (a 90-degree turn takes ~1s)
// Max turn accel: reaches max rate in 0.4s
#define TURN_PROFILE_MAX_VELOCITY    120.0f    // deg/s
#define TURN_PROFILE_MAX_ACCEL       300.0f    // deg/s^2

// =============================================================================
// FEEDFORWARD GAINS
// =============================================================================
// Feedforward terms provide proactive control effort based on the planned
// motion, rather than waiting for error to accumulate. This dramatically
// reduces the burden on the PID feedback controllers.

// Velocity feedforward: angle offset per unit desired velocity
// Represents how much lean is needed to sustain a given forward speed
// (compensates for rolling friction of tracks)
#define VELOCITY_FF_KV 0.003f     // degrees per (counts/s)

// Acceleration feedforward: angle offset per unit desired acceleration
// Represents how much extra lean is needed to change speed
#define VELOCITY_FF_KA 0.0003f    // degrees per (counts/s^2)

// Turn rate feedforward: turn output per unit desired heading rate
// Provides the differential motor speed needed to rotate at a given rate
#define TURN_FF_KV 0.5f           // motor units per (deg/s)

// Maximum total angle offset (feedforward + PID combined)
// Wider than the PID-only limit to allow feedforward headroom
#define ANGLE_OFFSET_MAX 20.0f    // degrees

// =============================================================================
// FRICTION FEEDFORWARD
// =============================================================================
// Constant motor effort applied in the direction of intended motion to
// overcome Coulomb (static) friction of the Zumo's tracks. Applied at
// the motor level, not through the angle cascade.

// Turn friction FF: differential motor units during active turn profile
// Ensures the wheels have enough differential to overcome track friction
#define FRICTION_FF_TURN 40.0f

// Turn settling friction FF: smaller differential during post-profile settling
// Applied when heading error exceeds deadband after profile completes
#define FRICTION_FF_TURN_SETTLE 35.0f

// Move settling friction FF: motor output during post-profile position settling
// Applied to both motors in the direction of position error
// Keep small to avoid destabilizing the balance controller (~30 PWM = ~0.7 deg equivalent)
#define FRICTION_FF_MOVE_SETTLE 30.0f

// =============================================================================
// COMMAND SETTLING AND COMPLETION
// =============================================================================
// After a motion profile finishes, the robot enters a settling phase where
// the PID controllers correct any residual error. The command is complete
// when the error is within tolerance for several consecutive cycles.

// Move settling
#define MOVE_SETTLE_TOLERANCE      50        // encoder counts (~5.2mm)
#define MOVE_SETTLE_VELOCITY       30.0f     // counts/s max velocity for "settled"
#define MOVE_SETTLE_COUNT          3         // consecutive 50Hz cycles

// Turn settling deadband: below this heading error, friction FF is removed
#define TURN_SETTLE_DEADBAND_DEG   2.0f

// Higher position gain during settling (overcomes cascaded gain attenuation)
// Used after move profile completes to drive the last few mm more aggressively
#define POSITION_KP_SETTLE 4.0f

// Safety timeout for any command (milliseconds)
// If a command doesn't complete within this time, force-complete it
#define COMMAND_TIMEOUT_MS 10000

// =============================================================================
// TURN CONTROLLER (Heading/Yaw)
// =============================================================================

// PID controller for heading stabilization while balancing
// Output is differential motor speed (subtracted from left, added to right)

// Tuning guide:
// - KP: Controls how aggressively robot turns toward target. Too high = overshoot.
// - KD: Critical for damping! Opposes rotational velocity to prevent overshoot.
//       With profiles providing the reference trajectory, KD damps deviations.
// - KI: Only needed if there's steady-state error. Start at 0, add if needed.
#define TURN_KP 1.5f         // Proportional gain for heading error (degrees)
#define TURN_KI 0.0f         // Integral gain for steady-state correction (start at 0)
#define TURN_KD 0.08f        // Derivative gain (damping from heading rate change)

// Integral windup limits for heading controller (degrees * seconds)
#define TURN_INTEGRAL_MIN -50.0f
#define TURN_INTEGRAL_MAX 50.0f

// Output limits (differential motor speed)
// Must be symmetric to allow both CCW (positive) and CW (negative) turns
#define TURN_OUTPUT_MIN -400.0f
#define TURN_OUTPUT_MAX 400.0f

// Maximum differential motor speed for turning
#define MAX_TURN_SPEED 400.0f

// Wheel track width (center-to-center distance between wheels, in mm)
#define WHEEL_TRACK_MM 85.0f
#define HALF_TRACK_MM (WHEEL_TRACK_MM / 2.0f)

// Acceptable heading error (degrees) - used for turn completion tolerance
#define HEADING_TOLERANCE_DEG 5.0f

// Turn completion requires low heading error AND low turn rate for several cycles
// This prevents early completion while still rotating
#define HEADING_RATE_TOLERANCE_DPS 9.0f
#define HEADING_SETTLE_COUNT 5

// Slew-rate limit for turn output (motor units per second)
// Reduces sudden yaw acceleration for smoother, more accurate turns
#define TURN_OUTPUT_SLEW_RATE 1200.0f

// Reduce position-hold influence during turns (counts per second)
// Small velocities inside this band are ignored during turn commands
#define TURN_POSITION_VELOCITY_DEADBAND 180.0f

// =============================================================================
// DEPRECATED - Turn output shaping (replaced by motion profiles)
// =============================================================================
// These constants are no longer used. The trapezoidal turn profile and
// friction feedforward replace all of this ad-hoc logic.
//
// #define TURN_SLOWDOWN_ANGLE_DEG 45.0f
// #define TURN_MIN_OUTPUT 80.0f
// #define TURN_MIN_OUTPUT_ANGLE_DEG 18.0f
// #define TURN_WHEEL_MIN_OUTPUT 60.0f
// #define TURN_STALL_DELTA_COUNTS 6
// #define TURN_STALL_SPEED_THRESHOLD 130.0f
// #define TURN_STALL_BOOST 35.0f

// =============================================================================
// DEPRECATED - Per-motor minimum output (replaced by friction feedforward)
// =============================================================================
// The old MIN_MOTOR_OUTPUT approach keyed off the sign of motorOutput,
// which oscillates during balance, making the enforcement unreliable.
// Friction feedforward replaces this with direction based on the planned
// motion trajectory.
//
// #define MIN_MOTOR_OUTPUT 70

// =============================================================================
// ENCODER-BASED HEADING CORRECTION
// =============================================================================

// Hybrid heading control uses both gyro and encoder difference to maintain heading
// Encoder difference detects wheel speed mismatch (friction, motor differences)
// This correction is always active when heading control is enabled (non-turn mode)

// Proportional gain for encoder difference correction
// Units: motor speed per encoder count difference
// Higher = more aggressive correction for wheel speed mismatch
#define ENCODER_HEADING_KP 1.0f

// Maximum correction from encoder difference (prevents overcorrection)
#define ENCODER_HEADING_MAX 50.0f

// =============================================================================
// ARC TURN PARAMETERS
// =============================================================================

// Arc completion tolerances
#define ARC_ANGLE_TOLERANCE_DEG 3.0f      // heading tolerance for arc completion
#define ARC_LENGTH_TOLERANCE_MM 3.0f      // arc length tolerance in mm

// Arc feedforward base motor effort
// This provides forward drive during arc turns, independent of balance correction.
// Without this, when the robot is balanced (motorOutput near zero), the arc stops.
// Higher values = stronger forward drive during arcs (may affect balance stability)
// Lower values = weaker drive, may stall on friction
// The actual motor speed is: (targetVelocity / MAX_COMMAND_VELOCITY) * ARC_BASE_EFFORT
// At 75 mm/s with default MAX_COMMAND_VELOCITY of 1000, this gives ~150 * 0.716 = ~107
#define ARC_BASE_EFFORT 180.0f

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

// =============================================================================
// Z-AXIS GYRO DRIFT COMPENSATION
// =============================================================================

#define ENABLE_GYRO_Z_DRIFT_COMPENSATION true
#define GYRO_Z_DRIFT_RATE_THRESHOLD 2.0f      // degrees per second

// =============================================================================
// SERIAL DEBUG OUTPUT
// =============================================================================

// Enable serial debug output (set to false for production, saves memory)
#define ENABLE_SERIAL_DEBUG true

// Serial baud rate
#define SERIAL_BAUD_RATE 115200

// Debug output rate (how often to print, in microseconds)
// 50000 = 20 Hz, good balance between detail and readability
#define SERIAL_DEBUG_PERIOD_US 50000

// Output format: CSV for easy plotting/analysis
// Columns: time,angle,targetAngle,angleRate,motorOut,leftSpd,rightSpd,
//          velocity,targetVelocity,angleOff,heading,targetHead,turnOut,cmdBusy
