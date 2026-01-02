// Copyright Pololu Corporation.  For more information, see http://www.pololu.com/
#pragma once

#include <Arduino.h>
#include "Config.h"
#include "MotionController.h"

/*
 * CommandHandler.h - High-level movement command interface
 *
 * Provides a simple API for issuing movement commands while balancing:
 * - Relative position moves (forward/backward N millimeters)
 * - Velocity commands (move at N mm/s for duration)
 * - Turn commands (rotate N degrees)
 * - Compound commands (move and turn simultaneously)
 *
 * Commands are queued and executed sequentially.
 */

// Command types
enum class CommandType : uint8_t
{
    NONE,
    MOVE_RELATIVE,      // Move relative distance
    SET_VELOCITY,       // Set velocity for duration
    TURN_RELATIVE,      // Turn relative angle
    MOVE_AND_TURN,      // Compound move + turn
    STOP,               // Stop and hold position
    WAIT                // Wait for duration
};

// Command state
enum class CommandState : uint8_t
{
    IDLE,               // No command active
    EXECUTING,          // Command in progress
    COMPLETED,          // Command finished successfully
    FAILED              // Command failed (e.g., robot fell)
};

// Single command structure
struct Command
{
    CommandType type;
    float param1;       // Distance (mm) or velocity (mm/s) or angle (deg)
    float param2;       // Secondary param: velocity for moves, duration for velocity cmd
    uint32_t startTime; // When command started (millis)
    uint32_t duration;  // Max duration (ms), 0 = no limit
};

// Command queue size
#define COMMAND_QUEUE_SIZE 8

class CommandHandler
{
public:
    CommandHandler()
        : motionController(nullptr),
          state(CommandState::IDLE),
          queueHead(0), queueTail(0), queueCount(0)
    {
        currentCommand.type = CommandType::NONE;
    }

    // Initialize with motion controller reference
    void begin(MotionController* mc)
    {
        motionController = mc;
        clear();
    }

    // Clear all commands and stop
    void clear()
    {
        queueHead = 0;
        queueTail = 0;
        queueCount = 0;
        currentCommand.type = CommandType::NONE;
        state = CommandState::IDLE;

        if (motionController)
        {
            motionController->stop();
        }
    }

    // === Immediate Commands (interrupt current command) ===

    // Stop immediately and hold position
    void stopNow()
    {
        clear();
        if (motionController)
        {
            motionController->stop();
        }
        state = CommandState::IDLE;
    }

    // === Queued Commands ===

    // Move forward/backward by distance (mm), at specified velocity (mm/s)
    // Negative distance = backward
    bool moveRelative(float distanceMM, float velocityMM = DEFAULT_MOVE_VELOCITY)
    {
        Command cmd;
        cmd.type = CommandType::MOVE_RELATIVE;
        cmd.param1 = constrain(distanceMM, -MAX_COMMAND_DISTANCE / COUNTS_PER_MM,
                               MAX_COMMAND_DISTANCE / COUNTS_PER_MM);
        cmd.param2 = fabs(velocityMM);
        cmd.duration = 0;  // No timeout, complete when at position
        return enqueue(cmd);
    }

    // Move forward (convenience)
    bool moveForward(float distanceMM, float velocityMM = DEFAULT_MOVE_VELOCITY)
    {
        return moveRelative(fabs(distanceMM), velocityMM);
    }

    // Move backward (convenience)
    bool moveBackward(float distanceMM, float velocityMM = DEFAULT_MOVE_VELOCITY)
    {
        return moveRelative(-fabs(distanceMM), velocityMM);
    }

    // Set velocity for specified duration (ms)
    // velocity in mm/s, duration in ms (0 = indefinite until next command)
    bool setVelocity(float velocityMM, uint32_t durationMs = 0)
    {
        Command cmd;
        cmd.type = CommandType::SET_VELOCITY;
        cmd.param1 = constrain(velocityMM, -MAX_COMMAND_VELOCITY / COUNTS_PER_MM,
                               MAX_COMMAND_VELOCITY / COUNTS_PER_MM);
        cmd.param2 = 0;
        cmd.duration = durationMs;
        return enqueue(cmd);
    }

    // Turn relative angle (degrees, positive = counterclockwise)
    bool turnRelative(float angleDeg)
    {
        Command cmd;
        cmd.type = CommandType::TURN_RELATIVE;
        cmd.param1 = angleDeg;
        cmd.param2 = 0;
        cmd.duration = 0;  // Complete when at heading
        return enqueue(cmd);
    }

    // Turn left (convenience)
    bool turnLeft(float angleDeg)
    {
        return turnRelative(fabs(angleDeg));
    }

    // Turn right (convenience)
    bool turnRight(float angleDeg)
    {
        return turnRelative(-fabs(angleDeg));
    }

    // Compound move and turn simultaneously
    bool moveAndTurn(float distanceMM, float angleDeg, float velocityMM = DEFAULT_MOVE_VELOCITY)
    {
        Command cmd;
        cmd.type = CommandType::MOVE_AND_TURN;
        cmd.param1 = distanceMM;
        cmd.param2 = angleDeg;
        cmd.duration = 0;
        return enqueue(cmd);
    }

    // Wait for duration (ms)
    bool wait(uint32_t durationMs)
    {
        Command cmd;
        cmd.type = CommandType::WAIT;
        cmd.param1 = 0;
        cmd.param2 = 0;
        cmd.duration = durationMs;
        return enqueue(cmd);
    }

    // Add stop command to queue
    bool queueStop()
    {
        Command cmd;
        cmd.type = CommandType::STOP;
        cmd.param1 = 0;
        cmd.param2 = 0;
        cmd.duration = 0;
        return enqueue(cmd);
    }

    // === Update (call in main loop) ===

    void update()
    {
        if (!motionController)
        {
            return;
        }

        // If no active command, try to start next one
        if (state == CommandState::IDLE || state == CommandState::COMPLETED)
        {
            if (!dequeue())
            {
                return;  // Queue empty
            }
            startCurrentCommand();
        }

        // Check if current command is complete
        if (state == CommandState::EXECUTING)
        {
            checkCommandComplete();
        }
    }

    // === Status ===

    CommandState getState() const { return state; }
    bool isBusy() const { return state == CommandState::EXECUTING; }
    bool isIdle() const { return state == CommandState::IDLE && queueCount == 0; }
    uint8_t getQueueCount() const { return queueCount; }
    CommandType getCurrentCommandType() const { return currentCommand.type; }

private:
    MotionController* motionController;
    CommandState state;

    // Current command being executed
    Command currentCommand;

    // Command queue (circular buffer)
    Command queue[COMMAND_QUEUE_SIZE];
    uint8_t queueHead;
    uint8_t queueTail;
    uint8_t queueCount;

    // Add command to queue
    bool enqueue(const Command& cmd)
    {
        if (queueCount >= COMMAND_QUEUE_SIZE)
        {
            return false;  // Queue full
        }

        queue[queueTail] = cmd;
        queueTail = (queueTail + 1) % COMMAND_QUEUE_SIZE;
        queueCount++;
        return true;
    }

    // Get next command from queue
    bool dequeue()
    {
        if (queueCount == 0)
        {
            return false;  // Queue empty
        }

        currentCommand = queue[queueHead];
        queueHead = (queueHead + 1) % COMMAND_QUEUE_SIZE;
        queueCount--;
        return true;
    }

    // Start executing current command
    void startCurrentCommand()
    {
        currentCommand.startTime = millis();
        state = CommandState::EXECUTING;

        switch (currentCommand.type)
        {
        case CommandType::MOVE_RELATIVE:
            motionController->moveRelativeMM(currentCommand.param1);
            // TODO: Set velocity limit from param2
            break;

        case CommandType::SET_VELOCITY:
            motionController->setTargetVelocityMM(currentCommand.param1);
            break;

        case CommandType::TURN_RELATIVE:
            motionController->turnRelative(currentCommand.param1);
            break;

        case CommandType::MOVE_AND_TURN:
            motionController->moveRelativeMM(currentCommand.param1);
            motionController->turnRelative(currentCommand.param2);
            break;

        case CommandType::STOP:
            motionController->stop();
            state = CommandState::COMPLETED;
            break;

        case CommandType::WAIT:
            // Nothing to do, just wait
            break;

        case CommandType::NONE:
        default:
            state = CommandState::IDLE;
            break;
        }
    }

    // Check if current command is complete
    void checkCommandComplete()
    {
        uint32_t elapsed = millis() - currentCommand.startTime;

        // Check duration timeout
        if (currentCommand.duration > 0 && elapsed >= currentCommand.duration)
        {
            completeCommand();
            return;
        }

        switch (currentCommand.type)
        {
        case CommandType::MOVE_RELATIVE:
            // Complete when at target position
            if (motionController->atTargetPosition())
            {
                completeCommand();
            }
            break;

        case CommandType::SET_VELOCITY:
            // Complete when duration expires (checked above)
            // If no duration, stays active until next command
            break;

        case CommandType::TURN_RELATIVE:
            // Complete when at target heading
            if (motionController->atTargetHeading())
            {
                completeCommand();
            }
            break;

        case CommandType::MOVE_AND_TURN:
            // Complete when both position and heading are reached
            if (motionController->atTargetPosition() &&
                motionController->atTargetHeading())
            {
                completeCommand();
            }
            break;

        case CommandType::WAIT:
            // Complete when duration expires (checked above)
            if (currentCommand.duration == 0)
            {
                completeCommand();  // No duration = immediate complete
            }
            break;

        default:
            break;
        }
    }

    void completeCommand()
    {
        state = CommandState::COMPLETED;
        currentCommand.type = CommandType::NONE;

        // Immediately try next command
        if (dequeue())
        {
            startCurrentCommand();
        }
        else
        {
            state = CommandState::IDLE;
            motionController->stop();  // Hold position when queue empty
        }
    }
};
