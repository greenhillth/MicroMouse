#pragma once

#include <Arduino.h>

struct driveAssembly {
mtrn3100::Motor* motor;
mtrn3100::PID* pidLin;
mtrn3100::PID* pidRot;
mtrn3100::Encoder* encoder;
}

class differentialDrive {
public
    differentialDrive();
    bool move();

private
    driveAssembly left;
    driveAssembly right;

}

