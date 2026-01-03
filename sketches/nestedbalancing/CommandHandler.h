// Copyright Pololu Corporation.  For more information, see http://www.pololu.com/
#pragma once

#include <Arduino.h>
#include "Config.h"
#include "MotionController.h"

/*
 * CommandHandler.h - Simple movement command interface for demos
 *
 * Provides a minimal API for issuing movement commands while balancing:
 * - move(distance): Forward/backward movement in mm
 * - rotate(angle): In-place rotation in degrees
 * - rotateFor(duration): Timed rotation (positive=CCW, negative=CW)
 * - arc(distance, radius): Inscribe a circular arc
 * - wait(duration): Pause for specified milliseconds
 *
 * Single-command execution model: one command at a time, check isIdle()
 * before issuing new commands.
 */

// Command types
enum class CommandType : uint8_t
{
    NONE,       // No command / idle
    MOVE,       // Straight line movement (forward/backward)
    ROTATE,     // In-place rotation (CW/CCW)
    ROTATE_FOR, // Timed rotation at constant rate
    ARC,        // Circular arc movement
    WAIT        // Timed pause
};

// Single command structure
struct Command
{
    CommandType type;
    float param1;       // distance (mm) for MOVE/ARC, angle (deg) for ROTATE
    float param2;       // radius (mm) for ARC only
    uint32_t startTime; // When command started (millis)
    uint32_t duration;  // For WAIT command (ms)
};

class CommandHandler
{
public:
    CommandHandler()
        : motionController(nullptr),
          busy(false)
    {
        currentCommand.type = CommandType::NONE;
    }

    // Initialize with motion controller reference
    void begin(MotionController* mc)
    {
        motionController = mc;
        clear();
    }

    // Clear current command and stop
    void clear()
    {
        currentCommand.type = CommandType::NONE;
        busy = false;

        if (motionController)
        {
            motionController->stop();
        }
    }

    // === Motion Commands ===

    // Move forward (positive) or backward (negative) by distance in mm
    bool move(float distanceMM)
    {
        if (busy || !motionController)
        {
            return false;
        }

        currentCommand.type = CommandType::MOVE;
        currentCommand.param1 = distanceMM;
        currentCommand.param2 = 0;
        currentCommand.startTime = millis();
        currentCommand.duration = 0;

        // Lock in current heading before starting move to ensure straight-line motion
        motionController->holdCurrentHeading();
        motionController->moveRelativeMM(distanceMM);
        busy = true;
        return true;
    }

    // Rotate in place: positive = counterclockwise, negative = clockwise
    bool rotate(float angleDeg)
    {
        if (busy || !motionController)
        {
            return false;
        }

        currentCommand.type = CommandType::ROTATE;
        currentCommand.param1 = angleDeg;
        currentCommand.param2 = 0;
        currentCommand.startTime = millis();
        currentCommand.duration = 0;

        motionController->turnRelative(angleDeg);
        busy = true;
        return true;
    }

    // Rotate for a specified duration
    // durationMs: positive = counterclockwise, negative = clockwise
    // Rotates at MAX_TURN_RATE degrees per second
    bool rotateFor(int32_t durationMs)
    {
        if (busy || !motionController)
        {
            return false;
        }

        currentCommand.type = CommandType::ROTATE_FOR;
        currentCommand.param1 = 0;
        currentCommand.param2 = 0;
        currentCommand.startTime = millis();
        currentCommand.duration = abs(durationMs);

        // Set turn rate: positive duration = CCW (positive rate)
        // negative duration = CW (negative rate)
        float turnRate = (durationMs >= 0) ? MAX_TURN_RATE : -MAX_TURN_RATE;
        motionController->setTurnRate(turnRate);
        busy = true;
        return true;
    }

    // Move in a circular arc
    // distanceMM: arc length to travel (positive = forward, negative = backward)
    // radiusMM: turn radius (positive = curve left, negative = curve right)
    bool arc(float distanceMM, float radiusMM)
    {
        if (busy || !motionController)
        {
            return false;
        }

        // Avoid division by zero
        if (fabs(radiusMM) < 1.0f)
        {
            return false;
        }

        currentCommand.type = CommandType::ARC;
        currentCommand.param1 = distanceMM;
        currentCommand.param2 = radiusMM;
        currentCommand.startTime = millis();
        currentCommand.duration = 0;

        // Calculate the angle swept during the arc
        // angle (radians) = arc_length / radius
        // Convert to degrees: angle_deg = (distance / radius) * (180 / PI)
        float angleDeg = (distanceMM / radiusMM) * (180.0f / PI);

        // Start coordinated arc motion
        motionController->startArc(distanceMM, angleDeg);
        busy = true;
        return true;
    }

    // Wait for specified duration in milliseconds
    bool wait(uint32_t durationMs)
    {
        if (busy || !motionController)
        {
            return false;
        }

        currentCommand.type = CommandType::WAIT;
        currentCommand.param1 = 0;
        currentCommand.param2 = 0;
        currentCommand.startTime = millis();
        currentCommand.duration = durationMs;

        // Ensure robot holds position during wait
        motionController->stop();
        busy = true;
        return true;
    }

    // === Update (call in main loop) ===

    void update()
    {
        if (!motionController || !busy)
        {
            return;
        }

        bool complete = false;

        switch (currentCommand.type)
        {
        case CommandType::MOVE:
            complete = motionController->atTargetPosition();
            break;

        case CommandType::ROTATE:
            complete = motionController->atTargetHeading();
            break;

        case CommandType::ROTATE_FOR:
            complete = (millis() - currentCommand.startTime) >= currentCommand.duration;
            if (complete)
            {
                motionController->stopTurning();
            }
            break;

        case CommandType::ARC:
            complete = motionController->atTargetPosition() &&
                       motionController->atTargetHeading();
            break;

        case CommandType::WAIT:
            complete = (millis() - currentCommand.startTime) >= currentCommand.duration;
            break;

        case CommandType::NONE:
        default:
            complete = true;
            break;
        }

        if (complete)
        {
            currentCommand.type = CommandType::NONE;
            busy = false;
            motionController->stop();
        }
    }

    // === Status ===

    bool isIdle() const { return !busy; }
    bool isBusy() const { return busy; }
    CommandType getCurrentCommandType() const { return currentCommand.type; }

    // Get progress info for display
    float getTargetDistance() const { return currentCommand.param1; }
    float getTargetRadius() const { return currentCommand.param2; }

private:
    MotionController* motionController;
    Command currentCommand;
    bool busy;
};
