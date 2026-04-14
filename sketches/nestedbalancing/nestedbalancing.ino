/* Nested Balancing - Cascaded PID balancing algorithm for Zumo 32U4
 *
 * This sketch implements a robust balancing algorithm with three control loops:
 *   - Inner Loop (fastest): Angle PID controller
 *   - Middle Loop: Velocity PID controller with feedforward
 *   - Outer Loop (slowest): Position PID controller
 *
 * Features:
 *   - Complementary filter for angle estimation (gyro + accelerometer fusion)
 *   - Encoder-based position and velocity tracking
 *   - Trapezoidal motion profiles for smooth move/turn execution
 *   - Velocity and acceleration feedforward for proactive control
 *   - Friction feedforward to overcome Coulomb friction of tracks
 *   - Fall detection and recovery
 *
 * Control Hierarchy (with profiles):
 *   Motion Profile -> Position Tracking + Feedforward -> Velocity Loop -> Angle Offset
 *   (Balance Angle + Angle Offset) -> Angle Loop -> Motor PWM
 *   Turn Profile -> Heading PID + Rate Feedforward + Friction FF -> Motor Differential
 *
 * Hardware: Zumo 32U4 with LCD or OLED display
 *
 * Tuning Instructions:
 *   1. Start with all loops disabled except inner angle loop
 *   2. Tune angle loop Kp until robot oscillates, then back off
 *   3. Add Kd to dampen oscillations
 *   4. Add small Ki to eliminate steady-state error
 *   5. Enable velocity loop, tune Kp to reduce drift
 *   6. Enable position loop, tune Kp for position holding
 *   7. Tune profile constraints (max velocity, max acceleration)
 *   8. Tune feedforward gains (KV, KA) to minimize PID effort
 */

#include <Wire.h>
#include <Zumo32U4.h>

#include "Config.h"
#include "PIDController.h"
#include "AngleEstimator.h"
#include "MotionProfile.h"
#include "MotionController.h"
#include "CommandHandler.h"

// Hardware objects
Zumo32U4LCD display;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;
Zumo32U4Motors motors;
Zumo32U4Buzzer buzzer;
Zumo32U4IMU imu;
Zumo32U4Encoders encoders;

// Control system components
AngleEstimator angleEstimator;
MotionController motionController;
CommandHandler commandHandler;

// Control state
enum class RobotState : uint8_t
{
    WAITING,        // Waiting for user to calibrate
    CALIBRATING,    // Calibrating gyro
    CALIBRATED,     // Calibrated, waiting to start balancing
    BALANCING,      // Active balancing
    FALLEN,         // Robot has fallen
    STOPPED         // Motors stopped
};

RobotState robotState = RobotState::WAITING;

// Timing variables
uint32_t lastMiddleLoopTime = 0;
uint32_t lastOuterLoopTime = 0;
uint32_t lastDisplayTime = 0;
uint32_t lastSerialDebugTime = 0;

// Current control outputs
float targetAngle = BALANCE_ANGLE;
float angleOffset = 0;      // From velocity loop (includes feedforward)
float motorOutput = 0;      // From angle loop
float turnOutput = 0;       // From heading control (includes friction FF)
float angleIntegral = 0;    // Inner loop integral term (must persist across runInnerLoop calls)

// Motor speeds (saved for debug output)
float debugLeftSpeed = 0;
float debugRightSpeed = 0;

// Demo mode state
bool demoMode = false;
uint8_t demoStep = 0;
uint32_t demoStartTime = 0;

// Forward declarations
void runInnerLoop();
void runMiddleLoop();
void runOuterLoop();
void updateDisplay();
void handleButtons();
void runDemoSequence();
void calibrateGyro();
void startBalancing();
void stopBalancing();
void handleFall();
void serialDebugOutput();

void setup()
{
    Wire.begin();

#if ENABLE_SERIAL_DEBUG
    Serial.begin(SERIAL_BAUD_RATE);
    // Print CSV header
    Serial.println(F("time_ms,angle,targetAngle,angleRate,motorOut,leftSpd,rightSpd,vel,targetVel,angleOff,heading,targetHead,turnOut,busy"));
#endif

    // Initialize display
    display.clear();
    display.print(F("Init..."));

    // Initialize IMU
    imu.init();
    imu.enableDefault();
    imu.configureForBalancing();

    // Initialize motion controller
    motionController.begin(&encoders);

    // Initialize command handler
    commandHandler.begin(&motionController);

    // Show instructions
    display.clear();
    display.print(F("A:Calibrt"));
    display.gotoXY(0, 1);
    display.print(F("Lay flat"));

    robotState = RobotState::WAITING;
}

void loop()
{
    uint32_t now = micros();

    switch (robotState)
    {
    case RobotState::WAITING:
        handleButtons();
        break;

    case RobotState::CALIBRATING:
        // Calibration is blocking, handled in calibrateGyro()
        break;

    case RobotState::CALIBRATED:
        // Waiting for user to stand robot up and press A to start
        handleButtons();
        break;

    case RobotState::BALANCING:
        // Inner loop: Run as fast as possible
        runInnerLoop();

        // Middle loop: Velocity control at ~50 Hz
        if (now - lastMiddleLoopTime >= MIDDLE_LOOP_PERIOD_US)
        {
            lastMiddleLoopTime = now;
            runMiddleLoop();
        }

        // Outer loop: Position control at ~20 Hz
        if (now - lastOuterLoopTime >= OUTER_LOOP_PERIOD_US)
        {
            lastOuterLoopTime = now;
            runOuterLoop();
        }

        // Display update at ~10 Hz
        if (now - lastDisplayTime >= DISPLAY_UPDATE_PERIOD_US)
        {
            lastDisplayTime = now;
            updateDisplay();
            handleButtons();
        }

#if ENABLE_SERIAL_DEBUG
        // Serial debug output at configured rate
        if (now - lastSerialDebugTime >= SERIAL_DEBUG_PERIOD_US)
        {
            lastSerialDebugTime = now;
            serialDebugOutput();
        }
#endif

        // Run demo sequence if active
        if (demoMode)
        {
            runDemoSequence();
        }

        // Check for fall
        if (fabs(angleEstimator.getAngle()) > FALL_ANGLE_THRESHOLD)
        {
            handleFall();
        }
        break;

    case RobotState::FALLEN:
        motors.setSpeeds(0, 0);

        // Check for recovery (robot stood back up)
        angleEstimator.updateGyro();
        angleEstimator.correctWithAccel();

        if (fabs(angleEstimator.getAngle()) < RECOVERY_ANGLE_THRESHOLD)
        {
            // Robot is upright again, restart balancing
            buzzer.playNote(NOTE_C(5), 100, 15);
            delay(200);
            startBalancing();
        }

        handleButtons();
        break;

    case RobotState::STOPPED:
        motors.setSpeeds(0, 0);
        handleButtons();
        break;
    }
}

// Inner loop: Angle control (runs as fast as possible)
void runInnerLoop()
{
    // Update angle estimate from gyro
    if (!angleEstimator.updateGyro())
    {
        return;  // No new gyro data
    }

    float currentAngle = angleEstimator.getAngle();
    float angleRate = angleEstimator.getAngleRate();

    // Compute target angle (balance point + offset from velocity loop)
    targetAngle = BALANCE_ANGLE + angleOffset;

    // Compute motor output using PID
    // Error = angle - target (matches official Balancing example)
    // Positive error = tilted too far forward = need positive motor speed to catch up
    float error = currentAngle - targetAngle;
    float dError = angleRate;  // Derivative of error (same sign as angle rate)

    // Manual PID computation for better control
    // angleIntegral is a global variable so it can be reset in startBalancing()
    angleIntegral += error;
    angleIntegral = constrain(angleIntegral, ANGLE_INTEGRAL_MIN, ANGLE_INTEGRAL_MAX);

    motorOutput = ANGLE_KP * error + ANGLE_KI * angleIntegral + ANGLE_KD * dError;
    motorOutput = constrain(motorOutput, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);

    // Compute left/right motor speeds
    float leftSpeed, rightSpeed;

    if (motionController.isArcModeEnabled())
    {
        // ARC TURN MODE: Combine feedforward arc drive with balance correction
        float leftBase, rightBase;
        motionController.getArcBaseWheelSpeeds(leftBase, rightBase);

        // Add balance correction (motorOutput) to both wheels equally
        leftSpeed = leftBase + motorOutput;
        rightSpeed = rightBase + motorOutput;
    }
    else
    {
        // NORMAL MODE: Balance correction + heading differential
        // turnOutput already includes turn friction feedforward (from MotionController)
        leftSpeed = motorOutput - turnOutput;
        rightSpeed = motorOutput + turnOutput;

        // Apply move settling friction feedforward (motor-level)
        // This small constant effort in the direction of position error helps
        // overcome track friction during the last few mm of a move command.
        // It's computed by MotionController based on profile state and position error.
        float moveFrictionFF = motionController.getMoveFrictionFF();
        if (moveFrictionFF != 0)
        {
            leftSpeed += moveFrictionFF;
            rightSpeed += moveFrictionFF;
        }
    }

    // Final constraining to hardware limits
    leftSpeed = constrain(leftSpeed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    rightSpeed = constrain(rightSpeed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);

    // Save for debug output
    debugLeftSpeed = leftSpeed;
    debugRightSpeed = rightSpeed;

    motors.setSpeeds((int16_t)leftSpeed, (int16_t)rightSpeed);
}

// Middle loop: Velocity control (~50 Hz)
void runMiddleLoop()
{
    // Correct angle with accelerometer
    angleEstimator.correctWithAccel();

    // Update encoder readings and compute velocity
    motionController.updateEncoders();

    // Optional: Update gyro drift compensation when robot is still
    angleEstimator.updateDriftCompensation(motionController.getVelocity());

    float dt = MIDDLE_LOOP_PERIOD_US / 1000000.0f;

    // Update heading from gyro Z-axis (using calibrated offset)
    // Do this before computeVelocityLoop/computeTurnOutput so they have fresh data
    imu.readGyro();
    motionController.updateHeading(imu.g.z, angleEstimator.getGyroOffsetZ(), dt);

    // Compute velocity loop output (includes profile update and feedforward)
    angleOffset = motionController.computeVelocityLoop(dt);

    // Update arc progress if active
    if (motionController.isArcModeEnabled())
    {
        float deltaAngle = motionController.getHeadingRate() * dt;
        float deltaLength = motionController.getVelocityMM() * dt;
        motionController.updateArcProgress(deltaAngle, deltaLength);
    }

    // Update Z-axis drift compensation
    angleEstimator.updateZDriftCompensation(
        imu.g.z,
        motionController.getHeadingRate(),
        motionController.getVelocity()
    );

    // Compute turn output (includes turn profile update, feedforward, friction FF)
    turnOutput = motionController.computeTurnOutput(dt);

    // Update command handler (checks completion)
    commandHandler.update();
}

// Outer loop: Position control (~20 Hz)
void runOuterLoop()
{
    float dt = OUTER_LOOP_PERIOD_US / 1000000.0f;
    // Only runs when no move profile is active (for position holding)
    motionController.computePositionLoop(dt);
}

// Update LCD display
void updateDisplay()
{
    display.clear();

    float angle = angleEstimator.getAngle();

    if (demoMode)
    {
        // Demo mode: Show step number and velocity
        // Line 1: Step number and velocity
        display.print(F("S"));
        display.print(demoStep);
        display.print(F(" V"));
        display.print((int)motionController.getVelocity());

        // Line 2: Heading and target heading
        display.gotoXY(0, 1);
        display.print(F("H"));
        display.print((int)motionController.getHeading());
        display.print(F(">"));
        display.print((int)motionController.getTargetHeading());
    }
    else
    {
        // Normal mode: Angle and velocity
        // Line 1: Angle and velocity
        display.print(angle, 1);  // 1 decimal place
        display.print(F(" "));
        display.print((int)motionController.getVelocity());

        // Line 2: Position and angle offset
        display.gotoXY(0, 1);
        display.print((int)motionController.getPositionMM());
        display.print(F(" "));
        display.print(angleOffset, 1);
    }
}

// Handle button presses
void handleButtons()
{
    if (buttonA.getSingleDebouncedRelease())
    {
        if (robotState == RobotState::WAITING)
        {
            // First press: calibrate gyro while robot is laying flat
            calibrateGyro();
        }
        else if (robotState == RobotState::CALIBRATED)
        {
            // Second press: start balancing
            startBalancing();
        }
        else if (robotState == RobotState::STOPPED ||
                 robotState == RobotState::FALLEN)
        {
            // After stopped/fallen: recalibrate first
            calibrateGyro();
        }
        else if (robotState == RobotState::BALANCING)
        {
            stopBalancing();
        }
    }

    if (buttonB.getSingleDebouncedRelease()) // SWAPPED TO DEBUG MODE, CHANGE TO B TO DEMO
    {
        if (robotState == RobotState::CALIBRATED)
        {
            // Start demo mode after calibration
            demoMode = true;
            startBalancing();
        }
        else if (robotState == RobotState::BALANCING)
        {
            // Toggle demo mode
            demoMode = !demoMode;
            if (demoMode)
            {
                demoStep = 0;
                demoStartTime = millis();
                commandHandler.clear();
            }
        }
    }

    if (buttonC.getSingleDebouncedRelease())
    {
        if (robotState == RobotState::BALANCING && demoMode)
        {
            // In demo mode: test move forward 100mm
            commandHandler.move(100);
        }
    }
}

// Calibrate gyro (robot should be laying flat, not balancing)
void calibrateGyro()
{
    robotState = RobotState::CALIBRATING;

    // Show calibration message
    display.clear();
    display.print(F("Gyro cal"));
    display.gotoXY(0, 1);
    display.print(F("Hold still"));
    ledYellow(1);

    // Calibrate gyro
    angleEstimator.begin(&imu);

    ledYellow(0);

    // Play calibration complete sound
    buzzer.playNote(NOTE_C(5), 100, 15);
    delay(100);

    // Show ready message
    display.clear();
    display.print(F("A:Balance"));
    display.gotoXY(0, 1);
    display.print(F("B:Demo"));

    robotState = RobotState::CALIBRATED;
}

// Start balancing (called after calibration)
void startBalancing()
{
    // Reset controllers
    motionController.reset();
    commandHandler.clear();

    // Reset angle to 0 since robot is now in balance position
    angleEstimator.reset();

    // Reset all control state variables to prevent accumulation between runs
    targetAngle = BALANCE_ANGLE;
    angleOffset = 0;
    motorOutput = 0;
    turnOutput = 0;
    angleIntegral = 0;  // Critical: reset inner loop integral

    // Initialize timing
    lastMiddleLoopTime = micros();
    lastOuterLoopTime = micros();
    lastDisplayTime = micros();

    if (demoMode)
    {
        demoStep = 0;
        demoStartTime = millis();
    }

    // Play start sound
    buzzer.playNote(NOTE_E(5), 100, 15);
    delay(100);

    display.clear();
    display.print(F("Balance!"));

    robotState = RobotState::BALANCING;
}

// Stop balancing
void stopBalancing()
{
    motors.setSpeeds(0, 0);
    robotState = RobotState::STOPPED;
    demoMode = false;

    // Reset control state to prevent accumulation
    angleIntegral = 0;
    angleOffset = 0;
    motorOutput = 0;
    turnOutput = 0;

    display.clear();
    display.print(F("Stopped"));
    display.gotoXY(0, 1);
    display.print(F("A:Recal"));

    buzzer.playNote(NOTE_C(4), 200, 15);
}

// Handle robot falling
void handleFall()
{
    motors.setSpeeds(0, 0);
    robotState = RobotState::FALLEN;
    demoMode = false;
    commandHandler.clear();

    // Reset control state to prevent accumulation
    angleIntegral = 0;
    angleOffset = 0;
    motorOutput = 0;
    turnOutput = 0;

    display.clear();
    display.print(F("Fallen!"));
    display.gotoXY(0, 1);
    display.print(F("Stand up"));

    // Play falling sound
    buzzer.playNote(NOTE_A(3), 300, 15);
}

// Run demo sequence: Simple square pattern using new turn() command
void runDemoSequence()
{
    uint32_t elapsed = millis() - demoStartTime;

    // Wait for current command to complete before issuing next
    if (!commandHandler.isIdle())
    {
        return;
    }

    switch (demoStep)
    {
    case 0:
        // Initial stabilization period
        if (elapsed > 1000)
        {
            buzzer.playNote(NOTE_C(5), 100, 15);
            demoStep++;
        }
        break;

    // Simple square pattern: move, turn 90, repeat 4 times
    case 1:
        commandHandler.move(-150);
        demoStep++;
        break;
    case 2:
        commandHandler.wait(100);
        demoStep++;
        break;
    case 3:
        commandHandler.turn(90);  // Turn 90 degrees CCW
        demoStep++;
        break;
    case 4:
        commandHandler.wait(100);
        demoStep++;
        break;

    case 5:
        commandHandler.move(-150);
        demoStep++;
        break;
    case 6:
        commandHandler.wait(100);
        demoStep++;
        break;
    case 7:
        commandHandler.turn(90);
        demoStep++;
        break;
    case 8:
        commandHandler.wait(100);
        demoStep++;
        break;

    case 9:
        commandHandler.move(-150);
        demoStep++;
        break;
    case 10:
        commandHandler.wait(100);
        demoStep++;
        break;
    case 11:
        commandHandler.turn(90);
        demoStep++;
        break;
    case 12:
        commandHandler.wait(100);
        demoStep++;
        break;

    case 13:
        commandHandler.move(-150);
        demoStep++;
        break;
    case 14:
        commandHandler.wait(100);
        demoStep++;
        break;
    case 15:
        commandHandler.turn(90);
        demoStep++;
        break;
    case 16:
        commandHandler.wait(200);  // Longer pause before repeating
        demoStep++;
        break;

    case 17:
        // Restart sequence
        demoStep = 1;
        break;
    }
}

#if ENABLE_SERIAL_DEBUG
// Serial debug output - CSV format for easy plotting
// Outputs key control values at SERIAL_DEBUG_PERIOD_US rate
void serialDebugOutput()
{
    // Format: time_ms,angle,targetAngle,angleRate,motorOut,leftSpd,rightSpd,
    //         vel,targetVel,angleOff,heading,targetHead,turnOut,busy

    Serial.print(millis());
    Serial.print(',');
    Serial.print(angleEstimator.getAngle(), 2);
    Serial.print(',');
    Serial.print(targetAngle, 2);
    Serial.print(',');
    Serial.print(angleEstimator.getAngleRate(), 2);
    Serial.print(',');
    Serial.print(motorOutput, 1);
    Serial.print(',');
    Serial.print(debugLeftSpeed, 1);
    Serial.print(',');
    Serial.print(debugRightSpeed, 1);
    Serial.print(',');
    Serial.print(motionController.getVelocity(), 1);
    Serial.print(',');
    Serial.print(motionController.getTargetVelocity(), 1);
    Serial.print(',');
    Serial.print(angleOffset, 2);
    Serial.print(',');
    Serial.print(motionController.getHeading(), 1);
    Serial.print(',');
    Serial.print(motionController.getTargetHeading(), 1);
    Serial.print(',');
    Serial.print(turnOutput, 1);
    Serial.print(',');
    Serial.println(commandHandler.isBusy() ? 1 : 0);
}
#endif
