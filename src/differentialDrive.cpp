#include "../include/differentialDrive.hpp"

// constructor
differentialDrive::differentialDrive(coords initPose, driveAssembly &left, driveAssembly &right) : currentPos(initPose), left(left), right(right)
{
    basePWM[2][2] = {};
}
bool differentialDrive::calibrate(mtrn3100::GYRO &imu)
{
    float init[2] = {left.encoder.position, right.encoder.position};
    for (int offset = 0; left.encoder.position == init[LEFT]; offset += 0.05)
    {
        left.motor.setPWM(offset);
        basePWM[FORWARD][LEFT] = offset;
    }
    for (int offset = 0; right.encoder.position == init[RIGHT]; offset += 0.05)
    {
        right.motor.setPWM(offset);
        basePWM[FORWARD][RIGHT] = offset;
    }

    for (int offset = 0; left.encoder.position != init[LEFT]; offset -= 0.05)
    {
        left.motor.setPWM(offset);
        basePWM[BACKWARD][LEFT] = offset;
    }
    for (int offset = 0; right.encoder.position != init[RIGHT]; offset -= 0.05)
    {
        right.motor.setPWM(offset);
        basePWM[BACKWARD][RIGHT] = offset;
    }
}

// move to position, return true when complete
bool differentialDrive::move(int cycle, coords dest)
{
    if (dest == currentPos)
    {
        return true;
    }
    switch (movement.currentStep)
    {
    case NOPLAN:
        this->movement = movePlan(currentPos, dest);
        movement.currentStep = INITR;
    case INITR:
        if (!rotate(cycle, movement.initR))
        {
            break;
        }
        else
        {
            movement.currentStep = MOVE;
        }
    case MOVE:
        if (cycle == 0)
        {
            encoderTargets = targets(movement.lineardist, left.encoder.position, right.encoder.position);
        }
        if (!linearMovement(cycle))
        {
            break;
        }
        else
        {
            movement.currentStep = FINALR;
        }
    case FINALR:
        if (!rotate(cycle, movement.finalR))
        {
            break;
        }
        else
        {
            movement.currentStep = COMPLETE;
        }
    case COMPLETE:
        return true;
        break;
    default:
        return false;
    }

    polar diff = dest - currentPos;

    // currentPos = currentPos.update(); // TODO - implement EKF update step here

    return false;
}

// move to waypoints, return true when all moves complete
bool differentialDrive::move(int cycle, coords dest1...)
{
}
bool differentialDrive::move(int cycle, float linearDist)
{
    if (cycle == 0)
    {
        Serial.println("Targets set");
        this->encoderTargets = targets(linearDist, left.encoder.position, right.encoder.position);
    }
    return (linearMovement(linearDist));
}

bool differentialDrive::rotate(int cycle, float degrees)
{
    return true;
}

bool differentialDrive::linearMovement(int cycle)
{
    // Set encoder targets if not set already
    float currentL = left.encoder.position;
    float currentR = right.encoder.position;

    // Calculate signal
    mtrn3100::Tuple<float, float> signal;
    float sp[2] = {encoderTargets.left, encoderTargets.right};
    float curr[2] = {currentL, currentR};

    signal = calculateSignal(cycle, curr, sp);
    setPWM(signal);
    // return true if movement complete and clear targets
    if (mtrn3100::get<LEFT>(signal) == 0 && mtrn3100::get<RIGHT>(signal) == 0)
    {
        this->encoderTargets = targets();
        return true;
    }
    return false;
}

mtrn3100::Tuple<float, float> differentialDrive::calculateSignal(int cycle, float current[2], float setpoint[2])
{
    float lSig = left.pidLin.compute(setpoint[LEFT], current[LEFT]);
    float rSig = right.pidLin.compute(setpoint[RIGHT], current[RIGHT]);

    if (lSig == 0 && rSig == 0)
    {
        return {0, 0};
    }

    bool negative = (lSig < 0);

    // calculate left-right compensation due to motor bias
    float compensation = compPID.compute((encoderTargets.initL - current[LEFT]), (encoderTargets.initR - current[RIGHT]));
    // if compensation positive (i.e left stronger than right)
    if (compensation < 0)
    {
        (negative) ? lSig += compensation : lSig -= compensation;
    }
    else if (compensation > 0)
    {
        (negative) ? rSig += compensation : rSig -= compensation;
    }

    return {lSig * ramp_up(cycle, 60), rSig * ramp_up(cycle, 60)};
}

/**
 * @brief Sets left and right motor PWM values.
 * @param signal <float, float> Tuple containing left and right signals.
 * @return void.
 */

void differentialDrive::setPWM(mtrn3100::Tuple<float, float> signal)
{
    left.motor.setPWM(mtrn3100::get<LEFT>(signal));
    right.motor.setPWM(mtrn3100::get<RIGHT>(signal));
}

float ramp_up(int cycle, double length)
{
    if (cycle >= length)
    {
        return 1;
    }
    // input validation
    else if (cycle < 0)
    {
        Serial.println("critical error in ramp_up function");
        return 0;
    }

    return (((1 / length) * cycle) * ((1 / length) * cycle));
}