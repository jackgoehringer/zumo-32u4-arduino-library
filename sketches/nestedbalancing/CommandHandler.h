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
    ARC,        // Arc turn with tightness/speed/angle
    WAIT        // Timed pause
};

// Single command structure
struct Command
{
    CommandType type;
    float param1;       // For MOVE: distance (mm); For ARC: tightness (0-1)
    float param2;       // For MOVE: unused;      For ARC: speed (mm/s)
    float param3;       // For ARC: angle (degrees)
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
        currentCommand.param3 = 0;
        currentCommand.startTime = millis();
        currentCommand.duration = 0;

        // Lock in current heading before starting move to ensure straight-line motion
        motionController->holdCurrentHeading();
        motionController->moveRelativeMM(distanceMM);
        busy = true;
        return true;
    }

    // Turn along an arc (tightness 0-1, speed mm/s, angle degrees)
    // tightness: 0 = straight, 1 = minimum radius (inner wheel stopped)
    // speedMMperSec: forward speed during turn (positive=forward)
    // angleDeg: heading change (positive=CCW, negative=CW)
    bool turnArc(float tightness, float speedMMperSec, float angleDeg)
    {
        if (busy || !motionController)
        {
            return false;
        }

        // If speed is zero or angle is zero, nothing to do
        if (speedMMperSec == 0.0f || angleDeg == 0.0f)
        {
            return false;
        }

        currentCommand.type = CommandType::ARC;
        currentCommand.param1 = tightness;
        currentCommand.param2 = speedMMperSec;
        currentCommand.param3 = angleDeg;
        currentCommand.startTime = millis();
        currentCommand.duration = 0;

        motionController->startArcTurn(tightness, speedMMperSec, angleDeg);
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
        currentCommand.param3 = 0;
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

        case CommandType::ARC:
            complete = motionController->isArcComplete();
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
    float getCurrentArcTightness() const { return currentCommand.param1; }
    float getCurrentArcSpeed() const { return currentCommand.param2; }
    float getCurrentArcAngle() const { return currentCommand.param3; }

private:
    MotionController* motionController;
    Command currentCommand;
    bool busy;
    bool turningInPlace;  // True during explicit rotate() commands
};
