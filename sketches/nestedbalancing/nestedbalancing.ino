/* Nested Balancing - Cascaded PID balancing algorithm for Zumo 32U4
 *
 * This sketch implements a robust balancing algorithm with three control loops:
 *   - Inner Loop (fastest): Angle PID controller
 *   - Middle Loop: Velocity PID controller
 *   - Outer Loop (slowest): Position PID controller
 *
 * Features:
 *   - Complementary filter for angle estimation (gyro + accelerometer fusion)
 *   - Encoder-based position and velocity tracking
 *   - Movement command interface (move forward/backward, turn)
 *   - Fall detection and recovery
 *
 * Control Hierarchy:
 *   Position Command -> Position Loop -> Velocity Setpoint
 *   Velocity Setpoint -> Velocity Loop -> Angle Offset
 *   (Balance Angle + Angle Offset) -> Angle Loop -> Motor PWM
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
 */

#include <Wire.h>
#include <Zumo32U4.h>

#include "Config.h"
#include "PIDController.h"
#include "AngleEstimator.h"
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
PIDController anglePID;

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

// Current control outputs
float targetAngle = BALANCE_ANGLE;
float angleOffset = 0;      // From velocity loop
float motorOutput = 0;      // From angle loop
float turnOutput = 0;       // From heading control

// Demo mode state
bool demoMode = false;
uint8_t demoStep = 0;
uint32_t demoStartTime = 0;

// Debug display mode (cycle through with button C while balancing)
// 0 = normal, 1 = heading debug, 2 = motor debug
uint8_t debugDisplayMode = 0;

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

void setup()
{
    Wire.begin();

    // Initialize display
    display.clear();
    display.print(F("Init..."));

    // Initialize IMU
    imu.init();
    imu.enableDefault();
    imu.configureForBalancing();

    // Initialize angle PID controller
    anglePID.setGains(ANGLE_KP, ANGLE_KI, ANGLE_KD);
    anglePID.setOutputLimits(ANGLE_OUTPUT_MIN, ANGLE_OUTPUT_MAX);
    anglePID.setIntegralLimits(ANGLE_INTEGRAL_MIN, ANGLE_INTEGRAL_MAX);
    anglePID.setDerivativeOnMeasurement(true);  // Prevents derivative kick

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

    // Track mode transitions to reset integral
    static float integral = 0;
    
    // ARC TURN MODE: Use full cascaded control including velocity loop
    // The MotionController keeps velocity control enabled during arc turns,
    // so we must apply angleOffset to maintain the commanded forward speed
    
    // Compute target angle (balance point + offset from velocity loop)
    targetAngle = BALANCE_ANGLE + angleOffset;

    // Compute motor output using PID
    // Error = angle - target (matches official Balancing example)
    // Positive error = tilted too far forward = need positive motor speed to catch up
    float error = currentAngle - targetAngle;
    float dError = angleRate;  // Derivative of error (same sign as angle rate)

    // Manual PID computation for better control
    integral += error;
    integral = constrain(integral, ANGLE_INTEGRAL_MIN, ANGLE_INTEGRAL_MAX);

    motorOutput = ANGLE_KP * error + ANGLE_KI * integral + ANGLE_KD * dError;
    motorOutput = constrain(motorOutput, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);

    // Apply minimum motor output during active commands to overcome static friction
    // This helps ensure the robot has enough power to start moving from rest
    if (MIN_MOTOR_OUTPUT > 0 && commandHandler.isBusy())
    {
        // Apply minimum threshold while preserving sign
        if (motorOutput > 0 && motorOutput < MIN_MOTOR_OUTPUT)
        {
            motorOutput = MIN_MOTOR_OUTPUT;
        }
        else if (motorOutput < 0 && motorOutput > -MIN_MOTOR_OUTPUT)
        {
            motorOutput = -MIN_MOTOR_OUTPUT;
        }
    }

    // Apply motor output with turn differential
    int16_t leftSpeed = constrain((int16_t)(motorOutput - turnOutput), -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    int16_t rightSpeed = constrain((int16_t)(motorOutput + turnOutput), -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);

    motors.setSpeeds(leftSpeed, rightSpeed);
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

    angleOffset = motionController.computeVelocityLoop(dt);

    // Update heading from gyro Z-axis (using calibrated offset)
    imu.readGyro();
    motionController.updateHeading(imu.g.z, angleEstimator.getGyroOffsetZ(), dt);

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

    // Compute turn output (with dt for proper integral term)
    turnOutput = motionController.computeTurnOutput(dt);

    // Update command handler
    commandHandler.update();
}

// Outer loop: Position control (~20 Hz)
void runOuterLoop()
{
    float dt = OUTER_LOOP_PERIOD_US / 1000000.0f;
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
    else if (debugDisplayMode == 1)
    {
        // Debug mode 1: Heading diagnostics
        // Line 1: H=heading, E=normalized error
        display.print(F("H"));
        display.print((int)motionController.getHeading());
        display.print(F(" E"));
        display.print((int)motionController.getHeadingError());

        // Line 2: O=turnOutput, R=headingRate
        display.gotoXY(0, 1);
        display.print(F("O"));
        display.print((int)turnOutput);
        display.print(F(" R"));
        display.print((int)motionController.getHeadingRate());
    }
    else if (debugDisplayMode == 2)
    {
        // Debug mode 2: Motor diagnostics
        // Line 1: L=left motor, R=right motor
        int16_t leftSpeed = constrain((int16_t)(motorOutput - turnOutput), -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
        int16_t rightSpeed = constrain((int16_t)(motorOutput + turnOutput), -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
        display.print(F("L"));
        display.print(leftSpeed);
        display.print(F(" R"));
        display.print(rightSpeed);

        // Line 2: M=motorOutput, T=turnOutput
        display.gotoXY(0, 1);
        display.print(F("M"));
        display.print((int)motorOutput);
        display.print(F(" T"));
        display.print((int)turnOutput);
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
        if (robotState == RobotState::BALANCING)
        {
            if (!demoMode)
            {
                // Cycle debug display mode: 0 -> 1 -> 2 -> 0
                // Also trigger a test rotation when entering mode 1
                debugDisplayMode = (debugDisplayMode + 1) % 3;
                
                if (debugDisplayMode == 1)
                {
                    // Entering heading debug mode - start a 90 degree arc turn test
                    commandHandler.turnArc(0.5, 50, 90);
                    buzzer.playNote(NOTE_E(5), 50, 15);
                }
                else if (debugDisplayMode == 0)
                {
                    // Back to normal mode
                    commandHandler.clear();
                }
            }
            else
            {
                // In demo mode: test move forward 100mm
                commandHandler.move(100);
            }
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
    anglePID.reset();
    motionController.reset();
    commandHandler.clear();

    // Reset angle to 0 since robot is now in balance position
    angleEstimator.reset();

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

    display.clear();
    display.print(F("Fallen!"));
    display.gotoXY(0, 1);
    display.print(F("Stand up"));

    // Play falling sound
    buzzer.playNote(NOTE_A(3), 300, 15);
}

// Run demo sequence: Move forward 300mm, rotate for 500ms, repeat
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

    case 1:
        commandHandler.move(-200);
        demoStep++;
        break;
    case 2:
        commandHandler.wait(100);
        demoStep++;
        break;
    case 3:
        commandHandler.turnArc(0.5, 50, 90);
        demoStep++;
        break;
    case 4:
        commandHandler.wait(100);
        demoStep++;
        break;
    case 5:
        commandHandler.move(-200);
        demoStep++;
        break;
    case 6:
        commandHandler.wait(100);
        demoStep++;
        break;
    case 7:
        commandHandler.turnArc(0.5, 50, 90);
        demoStep++;
        break;
    case 8:
        commandHandler.wait(100);
        demoStep++;
        break;
    case 9:
        commandHandler.move(-200);
        demoStep++;
        break;
    case 10:
        commandHandler.wait(100);
        demoStep++;
        break;
    case 11:
        commandHandler.turnArc(0.5, 50, 90);
        demoStep++;
        break;
    case 12:
        commandHandler.wait(100);
        demoStep++;
        break;
    case 13:
        commandHandler.move(-200);
        demoStep++;
        break;
    case 14:
        commandHandler.wait(100);
        demoStep++;
        break;
    case 15:
        commandHandler.turnArc(0.5, 50, 90);
        demoStep++;
        break;
    case 16:
        commandHandler.wait(100);
        demoStep++;
        break;

    case 17:
        // Restart sequence
        demoStep = 1;
        break;
    }
}
