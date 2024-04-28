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

    driveAssembly(mtrn3100::Motor &motor, mtrn3100::Encoder &encoder,
                  mtrn3100::PIDController &pidLin, mtrn3100::PIDController &pidRot)
        : motor(motor), encoder(encoder), pidLin(pidLin), pidRot(pidRot){};
};
class differentialDrive
{
public:
    differentialDrive(coords initPose, driveAssembly &left, driveAssembly &right);
    bool calibrate(mtrn3100::GYRO &imu);
    bool move(int cycle, coords dest);
    bool move(int cycle, coords dest1...);
    bool move(int cycle, float dist);
    float leftEncoderPos() { return left.encoder.position; };
    float rightEncoderPos() { return right.encoder.position; };

private:
    coords currentPos;
    driveAssembly left;
    driveAssembly right;
    mtrn3100::PIDController compPID;

    float basePWM[2][2];
    movePlan movement;
    float targetRotation;
    targets encoderTargets;

    mtrn3100::Tuple<float, float> calculateSignal(int cycle, float current[2], float setpoint[2]);
    void setPWM(mtrn3100::Tuple<float, float>);

    bool linearMovement(int cycle);
    bool rotate(int cycle, float degrees);
    float ramp_up(int cycle, double length);
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