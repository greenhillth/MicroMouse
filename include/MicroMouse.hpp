#pragma once

#include <Arduino.h>
#include <VL6180X.h>
#include <VL53L0X.h>
#include <Wire.h>

#include "ITG3200.h"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "EKF.hpp"
#include "Tuple.hpp"
#include "gyro.hpp"
#include "LinkedList.hpp"
#include "Encoder.hpp"
#include "Graph.hpp"
#include "mazeSolver.hpp"

typedef mtrn3100::Tuple<mtrn3100::Motor *, mtrn3100::Encoder *, mtrn3100::PIDController, mtrn3100::PIDController> MotorAssembly;
typedef mtrn3100::Tuple<VL6180X *, VL6180X *, VL53L0X *> lidarObj;

class MicroMouse;
// forward declaration

struct Command
{
    int processID;
    uint16_t processLifespan;
    float mfloatData;
    bool completed;

    void (MicroMouse::*func_ptr)();
    Command(int id, float floatData = 0);
    Command();
    void print();
};

struct Coords
{
    // heading in degrees
    double heading;
    // x, y in mm
    float x;
    float y;

    // experimental IMU based coords
    float headingGyro;
    float xAccel;
    float yAccel;

    Coords(float x, float y, double heading, float hG = -1, float xA = -1, float yA = -1);
    Coords(int column, int row, int heading);
    Coords();
    bool defined();
    float headingCos();
    float headingSin();
    mtrn3100::Tuple<int, int, double> pose();
    Coords update(mtrn3100::GYRO &gyro);
};

class MicroMouse
{
private:
    MotorAssembly &leftDrive;
    MotorAssembly &rightDrive;
    mtrn3100::Tuple<VL6180X *, VL6180X *, VL53L0X *> &lidars;
    mtrn3100::GYRO *gyro;
    // mtrn3100::EKF &ekf;

    // Command registry: Stores commands as linked list
    mtrn3100::LinkedList<Command> CommandRegistry;
    Command mCurrentCommand;
    String motionPlan;
    String mDataBuffer;
    String mSendBuffer;
    float encoderTarget[2];
    mazeInfo mMazeData;
    uint16_t lidarReadings[3][3];
    Coords globalCoords;
    int solveMode;
    // heading, x, y

public:
    void startLidars();
    void startBluetooth();

    // debugging function for lidars
    void printLidars();
    // constructor
    MicroMouse(MotorAssembly &leftDrive, MotorAssembly &rightDrive, lidarObj &Lidars, mtrn3100::GYRO *Gyro);

    float yawCorrection(int ID);

    // getters
    uint16_t lidarReading(int direction);
    float leftEncoderPos();
    float rightEncoderPos();
    float leftPIDsignal();
    float rightPIDsignal();
    float leftTurnPIDsignal();
    float rightTurnPIDsignal();

    void reset();

    mtrn3100::Tuple<float, float> calculatePWM(int cycle);
    mtrn3100::Tuple<float, float> calculateRotationalPWM(int cycle);

    void setLinearTarget(float distance);
    float lidarCorrection(int id);

    int writeBTtransmission();

    void setMotorPWM(mtrn3100::Tuple<float, float> signal);
    // commands
    void init();
    void idle();
    void await_bt_command();
    void define_maze();
    void move();
    void rotate();
    void transmit_bt();
    void calibrate();
    void solve_maze();
    void explore_maze();

    void run();
    void debugMenu();
    void ASCIItoMotionPlan();

private:
    bool lidarDiagnostics();
    bool imuDiagnostics();
    bool encoderDiagnostics();
    bool motorDiagnostics();
};

bool withinThreshold(float const distance);
float ramp_up(int cycle, double length);
mtrn3100::Tuple<float, float> linear(float x1, float x2 = 0, bool equalDist = true);
mtrn3100::Tuple<float, float> rotational(float h1);
mtrn3100::Tuple<float, float> scale(float leftSignal, float rightSignal);

void printLeftRightArgs(float leftArg, float rightArg, String title);