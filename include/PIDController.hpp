#pragma once

#include <math.h>

namespace mtrn3100
{

    class PIDController
    {
    public:
        float prev_signal = 0;
        PIDController(float kp = 1, float ki = 1, float kd = 1, float lower_limit = 0, float basePWM = 0, float upper_limit = 255,
                      float deadband = 1)
            : kp(kp), ki(ki), kd(kd), lower_limit(lower_limit), basePWM(basePWM), upper_limit(upper_limit), deadband(deadband) {}

        float compute(float setpoint, float input)
        {
            // Calculate the change in time since the last computation step.
            const uint32_t curr_time = micros();
            const float dt = static_cast<float>(curr_time - prev_time) / 1e6;
            prev_time = curr_time;

            // Calculate error.
            float error = setpoint - input;

            // Check if output needs to be zero'd if setpoint was reached.
            if (fabs(error) < deadband)
            {
                prev_error = 0;
                prev_input = input;
                return 0;
            }

            // Calculate accumulation of error with clamping and saturation limits.
            float conditional_integral = 0;
            if (!isSaturated(input, prev_input, 0.1) && isSameSign(error, prev_input))
            {
                integral = prev_integral + error * dt; // COMPLETE THIS LINE.
                conditional_integral = integral;
            }

            // Calculate change in error over time.
            float derivative = (error - prev_error) / dt;
            ; // COMPLETE THIS LINE.

            // Calculate PID.
            float signal = kp * error + ki * integral + kd * derivative; // COMPLETE THIS LINE.
            signal = limit(signal, lower_limit, upper_limit);
            /*
            if ((signal < basePWM) && (signal > 0) && (signal > lower_limit))
            {
                signal = basePWM;
            }
            else if ((signal > basePWM) && (signal < 0) && (signal < lower_limit))
            {
                signal = -basePWM;
            }
            */
            // Save.
            prev_integral = conditional_integral;
            prev_error = error;
            prev_input = input;
            prev_signal = signal;
            return signal;
        }

        void tune(float p, float i, float d)
        {
            kp = p;
            ki = i;
            kd = d;
        }

        void setLimits(float lower, float upper)
        {
            lower_limit = lower;
            upper_limit = upper;
        }

        void setDeadband(float limit) { deadband = limit; }

        void reset()
        {
            integral = 0;
            prev_integral = 0;
            prev_time = micros();
        }

    private:
        static bool isSaturated(float val1, float val2, float threshold) { return fabs(val1 - val2) < threshold; }

        static bool isSameSign(float val1, float val2) { return val1 > 0 && val2 > 0; }

        static float limit(float val, float lower, float upper)
        {
            bool sign = val < 0;
            float absval = fabs(val);
            if (absval < 0.001)
                return 0.00;
            if (absval < lower)
                return sign ? -lower : lower;
            if (absval > upper)
                return sign ? -upper : upper;
            return val;
        }

        float kp;
        float ki;
        float kd;
        float lower_limit;
        float basePWM;
        float upper_limit;
        float deadband;
        float integral = 0;
        float prev_integral = 0;
        float prev_error = 0;
        float prev_input = 0;
        uint32_t prev_time = micros();
    };

} // namespace mtrn3100
