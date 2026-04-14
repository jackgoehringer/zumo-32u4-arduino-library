// Copyright Pololu Corporation.  For more information, see http://www.pololu.com/
#pragma once

#include <Arduino.h>

/*
 * MotionProfile.h - Trapezoidal velocity profile generator
 *
 * Generates time-parameterized motion trajectories with three phases:
 *   1. Acceleration: velocity ramps up linearly
 *   2. Cruise: velocity held at maximum
 *   3. Deceleration: velocity ramps down linearly to zero
 *
 * For short distances where max velocity can't be reached, the profile
 * becomes triangular (accelerate then immediately decelerate).
 *
 * Usage:
 *   profile.start(distance, maxVelocity, maxAcceleration);
 *   // In control loop:
 *   profile.update(dt);
 *   float vel = profile.getVelocity();   // signed
 *   float acc = profile.getAcceleration(); // signed
 *   float pos = profile.getPosition();   // signed, relative to start
 *
 * The profile handles both positive and negative distances. All getters
 * return signed values incorporating the direction of travel.
 */

class TrapezoidalProfile
{
public:
    TrapezoidalProfile()
        : active(false), finished(true), direction(1.0f),
          totalDist(0), vMax(0), accelMag(0),
          tAccel(0), tCruise(0), tDecel(0), tTotal(0),
          elapsed(0), currentPos(0), currentVel(0), currentAccel(0)
    {
    }

    // Start a new profile
    // distance: signed distance to travel (encoder counts or degrees)
    // maxVelocity: maximum speed (positive, in counts/s or deg/s)
    // maxAcceleration: acceleration magnitude (positive, in counts/s^2 or deg/s^2)
    void start(float distance, float maxVelocity, float maxAcceleration)
    {
        // Handle degenerate cases
        if (fabs(distance) < 0.001f || maxVelocity <= 0 || maxAcceleration <= 0)
        {
            active = false;
            finished = true;
            currentVel = 0;
            currentAccel = 0;
            currentPos = 0;
            totalDist = 0;
            return;
        }

        direction = (distance >= 0) ? 1.0f : -1.0f;
        totalDist = fabs(distance);
        vMax = maxVelocity;
        accelMag = maxAcceleration;

        // Compute phase durations
        tAccel = vMax / accelMag;
        float dAccel = 0.5f * accelMag * tAccel * tAccel;

        if (2.0f * dAccel >= totalDist)
        {
            // Triangular profile: distance too short to reach max velocity
            // Solve: distance = 2 * (0.5 * accel * t^2) => t = sqrt(distance / accel)
            tAccel = sqrtf(totalDist / accelMag);
            tCruise = 0;
            tDecel = tAccel;
            vMax = accelMag * tAccel;  // Actual peak velocity (less than requested max)
        }
        else
        {
            // Full trapezoidal profile
            float dCruise = totalDist - 2.0f * dAccel;
            tCruise = dCruise / vMax;
            tDecel = tAccel;
        }

        tTotal = tAccel + tCruise + tDecel;
        elapsed = 0;
        currentPos = 0;
        currentVel = 0;
        currentAccel = 0;
        active = true;
        finished = false;
    }

    // Advance the profile by dt seconds
    // Call this at the control loop rate (e.g., 50 Hz)
    void update(float dt)
    {
        if (!active || finished)
        {
            currentVel = 0;
            currentAccel = 0;
            return;
        }

        elapsed += dt;

        if (elapsed >= tTotal)
        {
            // Profile complete: clamp to final state
            elapsed = tTotal;
            currentPos = totalDist;
            currentVel = 0;
            currentAccel = 0;
            finished = true;
            return;
        }

        float dAccelPhase = 0.5f * accelMag * tAccel * tAccel;

        if (elapsed <= tAccel)
        {
            // Phase 1: Acceleration
            currentAccel = accelMag;
            currentVel = accelMag * elapsed;
            currentPos = 0.5f * accelMag * elapsed * elapsed;
        }
        else if (elapsed <= tAccel + tCruise)
        {
            // Phase 2: Cruise (constant velocity)
            float tInCruise = elapsed - tAccel;
            currentAccel = 0;
            currentVel = vMax;
            currentPos = dAccelPhase + vMax * tInCruise;
        }
        else
        {
            // Phase 3: Deceleration
            float tInDecel = elapsed - tAccel - tCruise;
            float dCruisePhase = vMax * tCruise;
            currentAccel = -accelMag;
            currentVel = vMax - accelMag * tInDecel;
            currentPos = dAccelPhase + dCruisePhase
                         + vMax * tInDecel
                         - 0.5f * accelMag * tInDecel * tInDecel;

            // Clamp velocity to zero (numerical safety)
            if (currentVel < 0) { currentVel = 0; }
        }
    }

    // === Getters (all return signed values incorporating direction) ===

    // Current position relative to start (counts or degrees)
    float getPosition() const { return direction * currentPos; }

    // Current velocity (counts/s or deg/s)
    float getVelocity() const { return direction * currentVel; }

    // Current acceleration (counts/s^2 or deg/s^2)
    float getAcceleration() const { return direction * currentAccel; }

    // Final target position (total signed distance)
    float getFinalPosition() const { return direction * totalDist; }

    // Profile state
    bool isActive() const { return active; }
    bool isFinished() const { return finished; }
    float getElapsed() const { return elapsed; }
    float getTotalTime() const { return tTotal; }
    float getPeakVelocity() const { return direction * vMax; }

    // Force-stop the profile and reset state
    void stop()
    {
        active = false;
        finished = true;
        currentPos = 0;
        currentVel = 0;
        currentAccel = 0;
        totalDist = 0;
    }

private:
    bool active;
    bool finished;
    float direction;       // +1.0 or -1.0
    float totalDist;       // Absolute distance
    float vMax;            // Peak velocity (positive, may be less than requested if triangular)
    float accelMag;        // Acceleration magnitude (positive)

    // Phase durations (seconds)
    float tAccel;
    float tCruise;
    float tDecel;
    float tTotal;

    // Current state (unsigned magnitudes, direction applied in getters)
    float elapsed;
    float currentPos;
    float currentVel;
    float currentAccel;
};
