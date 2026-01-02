// Copyright Pololu Corporation.  For more information, see http://www.pololu.com/
#pragma once

#include <Arduino.h>

/*
 * PIDController.h - Reusable PID controller with anti-windup
 *
 * Features:
 * - Configurable P, I, D gains
 * - Integral windup protection with configurable limits
 * - Output saturation limits
 * - Derivative on measurement (optional) to avoid derivative kick
 * - Reset function for reinitialization
 */

class PIDController
{
public:
    PIDController()
        : kp(0), ki(0), kd(0),
          integral(0), lastError(0), lastMeasurement(0),
          outMin(-400), outMax(400),
          integralMin(-100), integralMax(100),
          derivativeOnMeasurement(false)
    {
    }

    PIDController(float kp, float ki, float kd)
        : kp(kp), ki(ki), kd(kd),
          integral(0), lastError(0), lastMeasurement(0),
          outMin(-400), outMax(400),
          integralMin(-100), integralMax(100),
          derivativeOnMeasurement(false)
    {
    }

    // Set PID gains
    void setGains(float p, float i, float d)
    {
        kp = p;
        ki = i;
        kd = d;
    }

    // Set output limits
    void setOutputLimits(float min, float max)
    {
        outMin = min;
        outMax = max;
    }

    // Set integral windup limits
    void setIntegralLimits(float min, float max)
    {
        integralMin = min;
        integralMax = max;
    }

    // Enable derivative on measurement instead of derivative on error
    // This prevents "derivative kick" when setpoint changes suddenly
    void setDerivativeOnMeasurement(bool enable)
    {
        derivativeOnMeasurement = enable;
    }

    // Compute PID output
    // setpoint: desired value
    // measurement: current measured value
    // dt: time step in seconds
    // Returns: control output (clamped to output limits)
    float compute(float setpoint, float measurement, float dt)
    {
        if (dt <= 0)
        {
            return 0;
        }

        float error = setpoint - measurement;

        // Proportional term
        float pTerm = kp * error;

        // Integral term with anti-windup
        integral += error * dt;
        integral = constrain(integral, integralMin, integralMax);
        float iTerm = ki * integral;

        // Derivative term
        float dTerm;
        if (derivativeOnMeasurement)
        {
            // Derivative on measurement (prevents derivative kick)
            float dMeasurement = (measurement - lastMeasurement) / dt;
            dTerm = -kd * dMeasurement;
        }
        else
        {
            // Derivative on error
            float dError = (error - lastError) / dt;
            dTerm = kd * dError;
        }

        // Store for next iteration
        lastError = error;
        lastMeasurement = measurement;

        // Sum and clamp output
        float output = pTerm + iTerm + dTerm;
        output = constrain(output, outMin, outMax);

        return output;
    }

    // Simplified compute without dt (for use in fixed-rate loops)
    // Assumes dt = 1, gains should be pre-scaled accordingly
    float compute(float setpoint, float measurement)
    {
        float error = setpoint - measurement;

        float pTerm = kp * error;

        integral += error;
        integral = constrain(integral, integralMin, integralMax);
        float iTerm = ki * integral;

        float dTerm;
        if (derivativeOnMeasurement)
        {
            dTerm = -kd * (measurement - lastMeasurement);
        }
        else
        {
            dTerm = kd * (error - lastError);
        }

        lastError = error;
        lastMeasurement = measurement;

        float output = pTerm + iTerm + dTerm;
        return constrain(output, outMin, outMax);
    }

    // Reset controller state (call when re-enabling control)
    void reset()
    {
        integral = 0;
        lastError = 0;
        lastMeasurement = 0;
    }

    // Reset with initial measurement (prevents derivative spike on startup)
    void reset(float initialMeasurement)
    {
        integral = 0;
        lastError = 0;
        lastMeasurement = initialMeasurement;
    }

    // Get current integral value (useful for debugging)
    float getIntegral() const
    {
        return integral;
    }

    // Get last error (useful for debugging)
    float getLastError() const
    {
        return lastError;
    }

    // Manually set integral (useful for bumpless transfer)
    void setIntegral(float value)
    {
        integral = constrain(value, integralMin, integralMax);
    }

private:
    float kp, ki, kd;
    float integral;
    float lastError;
    float lastMeasurement;
    float outMin, outMax;
    float integralMin, integralMax;
    bool derivativeOnMeasurement;
};
