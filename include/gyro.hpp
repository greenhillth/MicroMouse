#pragma once

#include <ITG3200.h>
#include <Wire.h>

namespace mtrn3100
{

    class GYRO
    {
    public:
        GYRO(bool adrValue = 0) : adrValue(adrValue) {}

        void begin()
        {

            Serial.println("Initializing I2C devices...");
            gyro.init(ITG3200_ADDR_AD0_LOW);
            gyro.zeroCalibrate(2500, 2);
            Serial.println("Testing device connections...");
            Serial.println(gyro.isRawDataReady() ? "ITG3200 connection successful" : "ITG3200 connection failed");

            Serial.println(F("Device connected!"));
        }

        bool dataReady() { return gyro.isRawDataReady(); }

        void reset()
        {
            wr = 0, wp = 0, wy = 0;
            r = 0, p = 0, y = 0;
            prevTime = micros();
        }

        float read()
        {
            // Compute change in time since last measurement.
            const uint32_t currTime = micros();
            float dt = static_cast<float>(currTime - prevTime) / 1e6;
            if (dt > 0.01)
            {
                dt = 0.01;
            }
            prevTime = currTime;

            // Compute roll-pitch-yaw.
            gyro.readGyro(&gr, &gp, &gy);
            wr = (gr - goffsetR);
            wp = (gp - goffsetP);
            wy = (gy - goffsetY);

            if (abs(wr) > 0.5)
            {
                r = r + (wr * dt / 2);
            }
            if (abs(wp) > 0.5)
            {
                p = p + (wp * dt / 2);
            }
            y = y + (wy * dt);
          return y;
        }

        // Angles
        float roll() { return r; }
        float pitch() { return p; }
        float yaw() { return y; }

    private:
        //    ICM_20948_I2C icm;  // Instantiate IMU object from Sparkfun.
        //    icm_20948_DMP_data_t data;

        const bool adrValue;

        uint32_t prevTime = 0;

        // Angular.
        float goffsetR, goffsetP, goffsetY;
        float gr, gp, gy;
        float wr = 0, wp = 0, wy = 0;
        float r = 0, p = 0, y = 0;
        float alpha = .91;
        ITG3200 gyro = ITG3200();
    };

} // namespace mtrn3100
