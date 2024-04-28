#pragma once

#include <Arduino.h>

#include "Encoder.hpp"
#include "Types.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "gyro.hpp"

constexpr int LEFT{0};
constexpr int RIGHT{1};

constexpr int FORWARD{0};
constexpr int BACKWARD{1};

struct driveAssembly
{
    mtrn3100::Motor &motor;
    mtrn3100::PIDController &pidLin;
    mtrn3100::PIDController &pidRot;
    mtrn3100::Encoder &encoder;

    driveAssembly();
};
class differentialDrive
{
public:
    differentialDrive(coords initPose);
    bool calibrate(mtrn3100::GYRO &imu);
    bool move(coords dest);
    bool move(coords dest1...);

private:
    coords currentPos;
    driveAssembly left;
    driveAssembly right;
    mtrn3100::PIDController compPID;

    float basePWM[2][2];
    movePlan movement;
    float targetRotation;
    targets encoderTargets;

    mtrn3100::Tuple<float, float> calculateSignal(float current[2], float setpoint[2]);
    void setPWM(mtrn3100::Tuple<float, float>);

    bool rotate(float degrees);
    bool linearMovement(float lDist);
};

/*
how movement works:
1st loop only:
 - get desired end position (be it coords or distance)
 - calculate difference
 - get signal from diff
rest of loops:
 - apply signal


*/