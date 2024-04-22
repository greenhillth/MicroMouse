#pragma once

#include "../include/MicroMouse.hpp"

// constexprs

constexpr int MOTOR{0};
constexpr int ENCODER{1};
constexpr int PID1{2};
constexpr int PID2{3};

constexpr int LEFT{0};
constexpr int RIGHT{1};
constexpr int FRONT{2};

typedef enum pID
{
    INIT = -1,
    IDLE,
    AWAIT_BT_COMMAND,
    DEFINE_MAZE,
    MOVE,
    ROTATE,
    TRANSMIT_BT,
    CALIBRATE,
    SOLVE_MAZE,
    EXPLORE_MAZE
};

constexpr int MANUAL{0};
constexpr int AUTO{2};

constexpr int X{0};
constexpr int Y{1};
constexpr int HEADING{2};

constexpr double INIT_OFFSET_X{250 / 2};
constexpr double INIT_OFFSET_Y{250 / 2};

constexpr int PLAN_MAX_LENGTH{30};

constexpr int TRANSLATE{1};

constexpr float leftCompensation{0};
constexpr float lidarScalar{0.15};
constexpr float yawScalar{0.2};
constexpr float basePWM{40};

mtrn3100::Tuple<float, float> zero(0, 0);

constexpr float R = 47.55 / 2; // mm.
constexpr float L = 145.0 / 2; // mm.

Command::Command(int id, float floatData) : processID(id), mfloatData(floatData), processLifespan(0), completed(false)
{
    switch (processID)
    {
    case INIT:
        func_ptr = &MicroMouse::init;
        break;
    case IDLE:
        func_ptr = &MicroMouse::idle;
        break;
    case AWAIT_BT_COMMAND:
        func_ptr = &MicroMouse::await_bt_command;
        break;
    case DEFINE_MAZE:
        func_ptr = &MicroMouse::define_maze;
        break;
    case MOVE:
        func_ptr = &MicroMouse::move;
        break;
    case ROTATE:
        func_ptr = &MicroMouse::rotate;
        break;
    case TRANSMIT_BT:
        func_ptr = &MicroMouse::transmit_bt;
        break;
    case CALIBRATE:
        func_ptr = &MicroMouse::calibrate;
        break;
    case SOLVE_MAZE:
        func_ptr = &MicroMouse::solve_maze;
        break;
    case EXPLORE_MAZE:
        func_ptr = &MicroMouse::explore_maze;
        break;
    }
}

Command::Command() : Command(INIT){};

Coords::Coords(float x, float y, double heading, float hG, float xA, float yA)
    : x(x), y(y), heading(heading), headingGyro(hG), xAccel(xA), yAccel(yA){};
Coords::Coords() : Coords(-1, -1, -1){};

bool Coords::defined() { return {(x > -1) && (y > -1) && (heading > -1)}; }

float Coords::headingCos() { return static_cast<float>(cos(heading * (M_PI / 180))); }
float Coords::headingSin() { return static_cast<float>(sin(heading * (M_PI / 180))); }

// Returns pose of current global co-ords in ROW(y), COLUMN(x), HEADING form
mtrn3100::Tuple<int, int, double> Coords::pose()
{
    /*
        if (defined())
        {
          WHY THE FUCK IS % NOT WORKING
            auto column = (static_cast<double>(x) % 250.0);
            auto row = (static_cast<double>(y) % 250.0);
        }
        return {static_cast<int>(row), static_cast<int>(column), heading};
    */

    return {0, 0, 0};
}

// Create coord object given pose
Coords::Coords(int row, int column, int cianHeading) : x(column * 250 + INIT_OFFSET_X), y(row * 250 + INIT_OFFSET_Y),
                                                       headingGyro(INIT), xAccel(INIT), yAccel(INIT)
{
    switch (cianHeading)
    {
    case 0: // NORTH
        heading = 90.0;
        break;
    case 1: // EAST
        heading = 0.0;
        break;
    case 2: // SOUTH
        heading = 270.0;
        break;
    case 3: // WEST
        heading = 180;
        break;
    }
}

// TODO - implement
Coords Coords::update(mtrn3100::GYRO &gyro)
{
    // insert IMU functions
    return Coords();
}

/**
 * @brief Constructor which initialises a robot instance.
 */
MicroMouse::MicroMouse(MotorAssembly &leftDrive, MotorAssembly &rightDrive, lidarObj &lidars, mtrn3100::GYRO *gyro)
    : leftDrive(leftDrive), rightDrive(rightDrive), lidars(lidars), gyro(gyro), encoderTarget({0.0, 0.0}),
      mCurrentCommand(Command()), globalCoords(Coords()), solveMode(INIT)
{
    mDataBuffer = String();
    mSendBuffer = String();
    motionPlan = String();
    CommandRegistry.push_back(Command(IDLE));
}

/**
 * @brief Getters which retrive encoder positions.
 * @return float - encoder position.
 */
float MicroMouse::leftEncoderPos() { return mtrn3100::get<ENCODER>(leftDrive)->position; }
float MicroMouse::rightEncoderPos() { return mtrn3100::get<ENCODER>(rightDrive)->position; }

// TODO - fix this function - using switch-cases because ::get requires STATIC member fml
/**
 * @brief Getter that retrives lidar
 * @param direction Lidar index (LEFT, RIGHT, FRONT)
 * @return uint16_t - lidar reading (mm).
 */
uint16_t MicroMouse::lidarReading(int direction)
{
    switch (direction)
    {
    case LEFT:
    {
        return mtrn3100::get<LEFT>(lidars)->readRangeSingleMillimeters();
    }
    case RIGHT:
    {
        return mtrn3100::get<RIGHT>(lidars)->readRangeSingleMillimeters();
    }
    case FRONT:
    {
        auto dist = mtrn3100::get<FRONT>(lidars)->readRangeSingleMillimeters();
        int i = 0;
        // basic data filtering - try to read 3 times
        while ((dist > 5000) && (i < 3))
        {
            dist = mtrn3100::get<FRONT>(lidars)->readRangeSingleMillimeters();
            i++;
        }
        return dist;
    }
    }
}

// TODO - add to util command?
//  setters + util
void MicroMouse::startLidars() {}

void MicroMouse::startBluetooth() {}

void MicroMouse::printLidars()
{
    Serial.print("Lidar readings (L, R, F): ");
    Serial.print(lidarReading(LEFT));
    Serial.print("\t");
    Serial.print(lidarReading(RIGHT));
    Serial.print("\t");
    Serial.print(lidarReading(FRONT));
    Serial.println('\r');
}

// calculate PID signal based on current pos
float MicroMouse::leftPIDsignal() { return mtrn3100::get<PID1>(leftDrive).compute(encoderTarget[LEFT], leftEncoderPos()); }
float MicroMouse::rightPIDsignal() { return mtrn3100::get<PID1>(rightDrive).compute(encoderTarget[RIGHT], rightEncoderPos()); }
float MicroMouse::leftTurnPIDsignal() { return (-1 * mtrn3100::get<PID2>(leftDrive).compute(encoderTarget[0], gyro->read())); }
float MicroMouse::rightTurnPIDsignal() { return mtrn3100::get<PID2>(rightDrive).compute(encoderTarget[0], gyro->read()); }

/**
 * @brief Sets left and right motor PWM values.
 * @param signal <float, float> Tuple containing left and right signals.
 * @return void.
 */
void MicroMouse::setMotorPWM(mtrn3100::Tuple<float, float> signal)
{
    mtrn3100::get<MOTOR>(leftDrive)->setPWM(mtrn3100::get<LEFT>(signal));
    mtrn3100::get<MOTOR>(rightDrive)->setPWM(mtrn3100::get<RIGHT>(signal));
}

/**
 * @brief Calculates correction value for lidar.
 * @param id int specifying LEFT, RIGHT or FRONT.
 * @return void.
 */
float MicroMouse::lidarCorrection(int id)
{
    float correction{0};
    uint16_t left{lidarReading(LEFT)};
    uint16_t right{lidarReading(RIGHT)};

    if (withinThreshold(left) && withinThreshold(right))
    {
        float diff = static_cast<float>(right) - static_cast<float>(left);
        if (abs(diff) > 10 && abs(diff) < 50)
        {
            correction = static_cast<float>(diff);
            correction *= lidarScalar;
        }
    }

    return {correction * (-1 * id)};
}

void MicroMouse::setLinearTarget(float distance)
{
    encoderTarget[LEFT] = leftEncoderPos() + mtrn3100::get<LEFT>(linear(distance));
    encoderTarget[RIGHT] = rightEncoderPos() + mtrn3100::get<RIGHT>(linear(distance));
}

float MicroMouse::yawCorrection(int ID) { return (-1 * ID) * (gyro->read() * yawScalar); }

// TODO - rework using arduino constrain functions
// Reduces output of stronger motor and clamps to [-255, 255]
mtrn3100::Tuple<float, float> scale(float leftSignal, float rightSignal)
{

    auto leadingMotor = static_cast<int>(leftCompensation > 0);
    float signals[2] = {leftSignal, rightSignal};

    if (abs(signals[leadingMotor]) <= leftCompensation)
    {
        return {signals[LEFT], signals[RIGHT]};
    }
    else if (signals[leadingMotor] > 0)
    {
        signals[leadingMotor] -= leftCompensation;
    }
    else if (signals[leadingMotor] < 0)
    {
        signals[leadingMotor] += leftCompensation;
    }

    return {signals[LEFT], signals[RIGHT]};
}

//  Calculates required PWM value to encoderTarget setpoint stored in memory
mtrn3100::Tuple<float, float> MicroMouse::calculatePWM(int cycle)
{
    auto left = leftPIDsignal();
    auto right = rightPIDsignal();

    if ((left == 0) && (right == 0))
    {
        return {0, 0};
    }

    if (cycle == 0)
    {
        gyro->reset();
    }

    // left += lidarCorrection(LEFT) + yawCorrection(LEFT);
    // right += lidarCorrection(RIGHT) + yawCorrection(RIGHT);
    // return { left, right }
    auto ramp_scalar = ramp_up(cycle, 40);
    return {(left * ramp_scalar), right * ramp_scalar};
}
// return scale(left * ramp_scalar, right * ramp_scalar);

// Non-class helpers

// Compute left and right wheel position changes for pure translational movement.
mtrn3100::Tuple<float, float> linear(float x1, float x2, bool equalDist)
{
    if (equalDist)
    {
        x2 = x1;
    }
    return {x1 / R, x2 / R};
}

// Compute left and right wheel position changes for pure rotational movement.
mtrn3100::Tuple<float, float> rotational(float h1)
{
    float thetaR = L * h1 / R;
    float thetaL = -thetaR;
    return {thetaL, thetaR};
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

// debugging function that prints command (couldnt be bothered overloading ostream)
void Command::print()
{
    String command = String("===========COMMAND===========\nProcessID: ");
    command.concat(processID);
    command.concat("\nLifespan: ");
    command.concat(processLifespan);
    command.concat("\nFloatdata: ");
    command.concat(mfloatData);
    command.concat("\nCompleted: ");
    command.concat(completed);
    command.concat("\n=============================\n");

    Serial.print(command);
}

void MicroMouse::reset()
{
    mDataBuffer = String();
    encoderTarget[0] = 0.0;
    encoderTarget[1] = 0.0;
}

mtrn3100::Tuple<float, float> MicroMouse::calculateRotationalPWM(int cycle)
{
    return {leftTurnPIDsignal() * ramp_up(cycle, 40), rightTurnPIDsignal() * ramp_up(cycle, 40)};
}

bool withinThreshold(float const distance) { return distance < 150; }

// returns 0 if successful
int MicroMouse::writeBTtransmission()
{
    if (Serial.available())
    {
        delay(10);
        mDataBuffer += Serial.readString();
        if (mDataBuffer.length() > 1)
        {
            mDataBuffer += " ";
            Serial.print("Recieved data (allegedly) - ");
            Serial.println(mDataBuffer);
            return 0;
        }
    }
    return 1;
}

// Converts ASCII stored in buffer to motion plan and writes motion plan to charMotionPlan
void MicroMouse::ASCIItoMotionPlan()
{
}

float limit(float val, float lower, float upper)
{
    bool sign = val < 0;
    float absval = fabs(val);
    if (absval < lower)
        return sign ? -lower : lower;
    if (absval > upper)
        return sign ? -upper : upper;

    return val;
}

float deadband(float val, float threshold) { return fabs(val) < threshold ? 0 : val; }

float average(float *array, size_t len)
{
    long sum = 0L; // Sum will be larger than an item, long for safety.
    for (size_t i = 0; i < len; i++)
        sum += array[i];
    return ((float)sum) / len; // Average will be fractional, so float may be appropriate.
}

void printLeftRightArgs(float leftArg, float rightArg, String title)
{
    Serial.print(title);
    Serial.print(" (L/R): ");
    Serial.print(leftArg);
    Serial.print('\t');
    Serial.print(rightArg);
    Serial.print('\t');
    Serial.print('\t');
}

// Command Functions
void MicroMouse::init()
{
    Serial.println("Function called");
    // just wait and set next command to idle for now
    delay(100);
    CommandRegistry.push_front(Command(IDLE));
    mCurrentCommand.completed = true;
}
void MicroMouse::idle()
{
    // CLear robot data
    reset();
    // wait?
    delay(100);

    // if there is no next command, set to await_bt_command
    if (CommandRegistry.empty())
    {
        CommandRegistry.push_back(Command(AWAIT_BT_COMMAND));
        // CommandRegistry.push_back(Command(MOVE, 250.0));
        // CommandRegistry.push_back(Command(MOVE, -250.0));
        // CommandRegistry.push_back(Command(ROTATE, 90));
        // CommandRegistry.push_back(Command(ROTATE, -90));
    }

    mCurrentCommand.completed = true;
}

void MicroMouse::move()
{
    // Set targets if first iter
    if (mCurrentCommand.processLifespan == 0)
    {
        Serial.println("Targets set");
        setLinearTarget(mCurrentCommand.mfloatData);
    }
    // calculate left and right signals
    auto signal = calculatePWM(mCurrentCommand.processLifespan);

    printLeftRightArgs(leftEncoderPos(), rightEncoderPos(), "Encoder Pos");
    printLeftRightArgs(encoderTarget[LEFT], encoderTarget[RIGHT], "Target Pos");
    printLeftRightArgs(mtrn3100::get<LEFT>(signal), mtrn3100::get<RIGHT>(signal), "Signal");
    Serial.println();

    if ((signal == zero) && mCurrentCommand.processLifespan > 0)
    {
        CommandRegistry.push_front(IDLE);
        mCurrentCommand.completed = true;
        if (globalCoords.defined())
        {
            globalCoords.x += mCurrentCommand.mfloatData * globalCoords.headingCos();
            globalCoords.y += mCurrentCommand.mfloatData * globalCoords.headingSin();
        }
        delay(1000);
    }
    else
    {

        setMotorPWM(signal);
    }
}

void MicroMouse::rotate()
{

    if (mCurrentCommand.processLifespan == 0)
    {
        gyro->reset();
        encoderTarget[0] = mCurrentCommand.mfloatData;
    }

    auto signal = calculateRotationalPWM(mCurrentCommand.processLifespan);

    if (signal == zero && mCurrentCommand.processLifespan > 0)
    {
        CommandRegistry.push_front(IDLE);
        mCurrentCommand.completed = true;
        if (globalCoords.heading > -1)
        {
            globalCoords.heading += static_cast<double>(mCurrentCommand.mfloatData);
            // reset heading to be in range [0, 360)
            while (globalCoords.heading >= 360)
            {
                globalCoords.heading -= 360;
            }
            while (globalCoords.heading < 0)
            {
                globalCoords.heading += 360;
            }
        }
        delay(1000);
    }
    else
    {
        setMotorPWM(signal);
    }
}
void MicroMouse::define_maze()
{
    if (solveMode == INIT)
    {
        CommandRegistry.push_front(Command(DEFINE_MAZE));
        // send
        mSendBuffer = String("Solve maze called! Please input maze data, or alternatively send the keyword \"auto\" to commence autonomous exploration - based solving ");
        while (!Serial3.availableForWrite())
        {
        }
        Serial3.println(mSendBuffer);
        CommandRegistry.push_front(Command(TRANSMIT_BT));
        solveMode = AWAIT_BT_COMMAND;
        mCurrentCommand.completed = true;
    }
    else if (solveMode == AWAIT_BT_COMMAND)
    {
        // wait for BT
        for (int i = 0; i < 2; i++)
        {
            while (writeBTtransmission())
            {
                delay(10);
            }
        }
        Serial.println("Breakpoint 1: Recieved maze data");
        Serial.print("data buffer:");
        Serial.println(mDataBuffer);
        if (0)
        {
            Serial.println("wtf");
            solveMode = AUTO;
            CommandRegistry.push_front(Command(EXPLORE_MAZE));
            mCurrentCommand.completed = true;
        }
        else
        {
            solveMode = MANUAL;
            Serial.println("Breakpoint 1.5: before string2graph");
            mMazeData = mazeInfo(mDataBuffer);
            Serial.println("Breakpoint 2: Commited maze data to memory");

            // Set global coords
            globalCoords = Coords(mMazeData.startRow, mMazeData.startCol, mMazeData.heading);
            mDataBuffer = String();
            CommandRegistry.push_back(Command(SOLVE_MAZE));
            mCurrentCommand.completed = true;
        }
        solveMode = 0;
    }
    else
    {
        Serial.print("Maze already defined!");
        mCurrentCommand.completed = true;
    }
}
void MicroMouse::explore_maze()
{
    // add explore function here
    mSendBuffer = String("Whoops! Functionality not yet implemented :(\n Please send a new command");
    CommandRegistry.push_front(Command(TRANSMIT_BT));

    mCurrentCommand.completed = true;
}
// Solve the maze from internal graph representation

void MicroMouse::solve_maze()
{
    Serial.println("Breakpoint 2.5: SolveMaze called");

    // if (mMazeData.initialised())
    // {
    //     motionPlan = mazeSolver(&mMazeData);
    //     Serial.println("Breakpoint 3: motionPlan generated");
    // }
    motionPlan = mMazeData.generateMotionPlan();
    mSendBuffer = "Motion Plan: " + motionPlan;
    while (!Serial3.availableForWrite())
    {
    }
    Serial3.println(mSendBuffer);

    // send motion plan to bt
    mSendBuffer = String(motionPlan);
    CommandRegistry.push_back(Command(TRANSMIT_BT));

    // convert motion plan to commands
    for (char movement : motionPlan)
    {
        // TODO - fix conflicting co-ord conventions
        switch (movement)
        {
        case 'F':
            CommandRegistry.push_back(Command(MOVE, 250.0));
            break;
        case 'B':
            CommandRegistry.push_back(Command(MOVE, -250.0));
            break;
        case 'L':
            CommandRegistry.push_back(Command(ROTATE, 90));
            break;
        case 'R':
            CommandRegistry.push_back(Command(ROTATE, -90));
            break;
        default:
            Serial.print("Error parsing motion plan");
            break;
        }
    }

    // empty motion plan
    motionPlan = String();
    mCurrentCommand.completed = true;
}

void MicroMouse::await_bt_command()
{
    if (mCurrentCommand.processLifespan == 0)
    {
        while (!Serial3.availableForWrite())
        {
        }
        Serial3.println("Robot initialised and awaiting command");
    }
    delay(100);
    if (!writeBTtransmission())
    {
        if (mDataBuffer.indexOf("solve maze") != -1)
        {
            CommandRegistry.push_back(Command(IDLE));
            CommandRegistry.push_back(Command(DEFINE_MAZE));
        }
        // Sends move command if recieved data is of the form "move 23.4" (can be any float)
        else if (mDataBuffer.indexOf("move") != -1)
        {
            float dist = mDataBuffer.substring(mDataBuffer.indexOf("move") + 5).toFloat();

            if (dist == 0)
            {
                Serial3.print("Error reading distance val!");
            }
            else
            {
                CommandRegistry.push_back(Command(IDLE));
                CommandRegistry.push_back(Command(MOVE, dist));
            }
        }

        mCurrentCommand.completed = true;
    }
}

void MicroMouse::transmit_bt()
{
    if (Serial3.available())
    {
        Serial3.print(mSendBuffer);
        mSendBuffer = String("Send buffer empty");
        mCurrentCommand.completed = true;
    }
}

void MicroMouse::run()
{
    Serial.print("Current command ID: ");
    Serial.println(mCurrentCommand.processID);
    int i = 0;
    gyro->reset();
    while (1)
    {

        // update current command if nessecary
        if (mCurrentCommand.completed)
        {
            Serial.println("New command recieved, printing old command");
            mCurrentCommand.print();
            if (CommandRegistry.empty())
            {
                CommandRegistry.push_front(Command(IDLE));
            }
            Serial.println("New command recieved, printing new command");
            mCurrentCommand = CommandRegistry.pop_front();
            mCurrentCommand.print();
        }

        // execute command
        (this->*mCurrentCommand.func_ptr)();

        // increment the process lifespan counter
        mCurrentCommand.processLifespan++;

        // update  current robot params??
        // if (i % 50 == 0)
        // {
        //     if (mCurrentCommand.processID != AWAIT_BT_COMMAND)
        //     {

        //         mCurrentCommand.print();
        //     }
        // }
        // i++;

        // Serial.print("Yaw: ");
        // Serial.println(gyro->read());
    }
}

//
void MicroMouse::debugMenu()
{
    bool exit_flag = false;
    String title = "Initialised debug menu. Please select sensor you'd like to check:";
    String menu = "1: Lidars\n"
                  "2: IMU\n"
                  "3. Encoders\n"
                  "4. Motors";
    Serial.println(title);
    Serial.println(menu);
    while (!exit_flag)
    {
        while (!(Serial.available() > 0))
        {
            delay(15);
        }
        String input = Serial.readString();
        if (input.equalsIgnoreCase("e"))
        {
            exit_flag = true;
        }
        else if (input == "1" || input.equalsIgnoreCase("lidar") || input.equalsIgnoreCase("lidars"))
        {
            Serial.println("Lidar selected. Printing lidar information: (q to quit)");

            while (lidarDiagnostics())
                ;
            Serial.readString();
            Serial.println("Select a new diagnostic target, or press E to exit");
        }
        else if (input == "2" || input.equalsIgnoreCase("imu") || input.equalsIgnoreCase("imus"))
        {
            Serial.println("IMU selected. Printing IMU information: (press any key to interrupt)");

            while (imuDiagnostics())
                ;
            Serial.readString();
            Serial.println("Select a new diagnostic target, or press E to exit");
        }
        else if (input == "3" || input.equalsIgnoreCase("encoder") || input.equalsIgnoreCase("encoders"))
        {
            Serial.println("Encoders selected. Printing encoder information: (press any key to interrupt)");

            while (encoderDiagnostics())
                ;
            Serial.readString();
            Serial.println("Select a new diagnostic target, or press E to exit");
        }
        else if (input == "4" || input.equalsIgnoreCase("motor") || input.equalsIgnoreCase("motors"))
        {
            Serial.println("Motors selected. Printing motor information: (press any key to interrupt)");

            while (motorDiagnostics())
                ;
            Serial.println("Select a new diagnostic target, or press E to exit");
        }
        else
        {
            Serial.println("Invalid input. Please select a valid option from the menu, or press E to exit:");
        }

        if (exit_flag)
        {
            Serial.println("Exiting...");
        }
        else
        {

            Serial.println(menu);
        }
    }
}

bool MicroMouse::lidarDiagnostics()
{
    printLidars();
    return (Serial.available() == 0);
}

bool MicroMouse::imuDiagnostics()
{
    gyro->read();

    Serial.print("IMU Readings (Roll, Pitch, Yaw): ");
    Serial.print(gyro->roll());
    Serial.print("\t");
    Serial.print(gyro->pitch());
    Serial.print("\t");
    Serial.print(gyro->yaw());
    Serial.println('\r');

    return (Serial.available() == 0);
}

bool MicroMouse::encoderDiagnostics()
{
    Serial.print("Encoder Readings (Left, Right): ");
    Serial.print(leftEncoderPos());
    Serial.print("\t");
    Serial.print(rightEncoderPos());
    Serial.println('\r');

    return (Serial.available() == 0);
}

bool MicroMouse::motorDiagnostics()
{
    float kp{5}, ki{0.5}, kd{4}, max{30};
    Serial.println("Entered Motor Diagnostic mode. Press Q to quit.");
    boolean exitFlag = false;
    while (!exitFlag)
    {
        Serial.println("Type p to modify test params, or enter the starting and ending PWM values for the test, separated by a comma:");
        while (Serial.available() == 0)
            ;
        delay(5);
        String input = Serial.readString();
        if (input.equals("q"))
        {
            exitFlag = true;
            break;
        }
        else if (input.equals("p"))
        {
            Serial.println("Modifying test params. Current parameters:");
            Serial.print("kp: ");
            Serial.println(kp, 4);
            Serial.print("ki: ");
            Serial.println(ki, 4);
            Serial.print("kd: ");
            Serial.println(kd, 4);
            Serial.println("Enter new values, separated by a comma, or leave blank to keep as is.");
            while (Serial.available() == 0)
                ;
            delay(5);
            String paramIn = Serial.readString();
            if (paramIn.length() > 1)
            {
                int firstComma = paramIn.indexOf(',');
                int secondComma = paramIn.lastIndexOf(',');

                if (firstComma == -1)
                {
                    kp = paramIn.toFloat();
                }
                else if (firstComma == secondComma)
                {
                    kp = paramIn.substring(0, firstComma).toFloat();
                    ki = paramIn.substring(firstComma + 1).toFloat();
                }
                else if (firstComma != -1 && secondComma != -1)
                {
                    kp = paramIn.substring(0, firstComma).toFloat();
                    ki = paramIn.substring(firstComma + 1, secondComma).toFloat();
                    kd = paramIn.substring(secondComma + 1).toFloat();
                }
            }
            continue;
        }
        else
        {

            String left = input.substring(0, input.indexOf(',') - 1);
            String right = input.substring(input.indexOf(',') + 1);

            runMotorDiagnosics({left.toFloat(), right.toFloat()}, 1, 800, kp, ki, kd, max);
        }
    }
}

void MicroMouse::runMotorDiagnosics(mtrn3100::Tuple<float, float> bounds, int step, int numSteps, float kp, float ki, float kd, float max)
{
    mtrn3100::PIDController diff = mtrn3100::PIDController(kp, ki, kd, 0.0, 0.0, max, 0.1);

    float initL = leftEncoderPos();
    float initR = rightEncoderPos();

    float lpos{0}, rpos{0}, lprev{0}, rprev{0}, deltaL{0}, deltaR{0};
    float minPWM = mtrn3100::get<0>(bounds);
    float maxPWM = mtrn3100::get<1>(bounds);
    float signal = minPWM;
    int j = 0;
    Serial.println("BEGIN DATA TRANSMISSION");
    Serial.println("motor.csv");
    float compensation = 0;
    for (int i = 0; i < numSteps * 2; i++)
    {
        if (i < numSteps)
        {
            signal = MicroMouse::signalGenerator(i, numSteps, 1, minPWM, maxPWM);
        }
        else
        {
            signal = MicroMouse::signalGenerator(i - numSteps, numSteps, 3, minPWM, maxPWM);
        }

        compensation = (signal > 0) ? compensation : -compensation;

        mtrn3100::Tuple<float, float> sigs = normaliseSignals(signal + compensation, signal);

        setMotorPWM(sigs);

        lpos = leftEncoderPos() - initL;
        rpos = rightEncoderPos() - initR;
        compensation = diff.compute(rpos, lpos);

        Serial.print(i);
        Serial.print(',');
        Serial.print(mtrn3100::get<0>(sigs), 4);
        Serial.print(',');
        Serial.print(mtrn3100::get<1>(sigs), 4);
        Serial.print(',');
        Serial.print(compensation, 4);
        Serial.print(',');
        Serial.print(lpos, 4);
        Serial.print(',');
        Serial.println(rpos, 4);

        lprev = lpos;
        rprev = rpos;

        delay(10);
    }
    setMotorPWM({0, 0});
    Serial.println("END DATA TRANSMISSION");
}

mtrn3100::Tuple<float, float> normaliseSignals(float leftSignal, float rightSignal)
{
    if (-255 > leftSignal && leftSignal > 255 && -255 > rightSignal && rightSignal > 255)
    {
        return {leftSignal, rightSignal};
    }
    bool negFlag = (leftSignal < 0) && (rightSignal < 0);
    float overflow;
    if (!negFlag)
    {
        // if left stronger than right
        if (leftSignal > rightSignal)
        {
            overflow = (leftSignal < 255) ? 0 : leftSignal - 255;
        }
        else
        {
            overflow = (rightSignal < 255) ? 0 : rightSignal - 255;
        }
        return {leftSignal - overflow, rightSignal - overflow};
    }
    else
    {
        if (leftSignal < rightSignal)
        {
            overflow = (leftSignal > -255) ? 0 : -leftSignal + 255;
        }
        else
        {
            overflow = (rightSignal > -255) ? 0 : -rightSignal + 255;
        }
        return {leftSignal + overflow, rightSignal + overflow};
    }
}

float MicroMouse::signalGenerator(int step, int numSteps, int mode, float minVal, float maxVal)
{
    int flip = numSteps / 4;
    int doubleflip = flip / 2;
    float increment = (maxVal - minVal) / flip;
    switch (mode)
    {
    // ramp case
    case 1:
        if (step < flip)
        {
            return {minVal + increment * step};
        }
        else if (step < 2 * flip)
        {
            return {maxVal - increment * (step - flip)};
        }
        else if (step < 3 * flip)
        {
            return {-(minVal + increment * (step - 2 * flip))};
        }
        else
        {
            return {-maxVal + increment * (step - 3 * flip)};
        }
        break;
    // alternating
    case 2:
        if (step < flip)
        {
            return {-maxVal};
        }
        else if (step < 2 * flip)
        {
            return {maxVal};
        }
        else if (step < 3 * flip)
        {
            return {-maxVal};
        }
        else
        {
            return {maxVal};
        }
        break;
        // step
    case 3:
        if (step < doubleflip)
        {
            return {minVal};
        }
        else if (step < 2 * doubleflip)
        {
            return {minVal + (maxVal - minVal) / 2};
        }
        else if (step < 3 * doubleflip)
        {
            return {maxVal};
        }
        else if (step < 4 * doubleflip)
        {
            return {minVal};
        }
        if (step < 5 * doubleflip)
        {
            return {-minVal};
        }
        else if (step < 6 * doubleflip)
        {
            return {-(minVal + (maxVal - minVal) / 2)};
        }
        else if (step < 7 * doubleflip)
        {
            return {-maxVal};
        }
        else
        {
            return {-minVal};
        }
        break;
    }
}

// calibrate the bot based on expected distance to walls
void MicroMouse::calibrate()
{
    if (static_cast<int>(mCurrentCommand.mfloatData) == 0)
    {
        // reading 1
        for (int i = 0; i < 3; i++)
        {
            lidarReadings[0][i] = lidarReading(i);
        }
        CommandRegistry.push_front(Command(CALIBRATE, 1));
        CommandRegistry.push_front(Command(ROTATE, 180));
        mCurrentCommand.completed = true;
    }
    else if (static_cast<int>(mCurrentCommand.mfloatData) == 1)
    {
        // reading 2
        for (int i = 0; i < 3; i++)
        {
            lidarReadings[1][i] = lidarReading(i);
        }
        CommandRegistry.push_front(Command(CALIBRATE, 2));
        CommandRegistry.push_front(Command(ROTATE, 180));
        mCurrentCommand.completed = true;
    }
    else
    {

        // reading 3
        for (int i = 0; i < 3; i++)
        {
            lidarReadings[2][i] = lidarReading(i);
        }

        // compare to expected values

        // calculate positional offset

        // push move commands to front

        // set task as complete

        mCurrentCommand.completed = true;
    }
}

/*
TODOs
//Incorporate global positioning (currently initialised at [-1, -1, -1]), need to set after reading start data

Finish calibration function

read gyros and lidars after each cycle of run()? and replace readLidars() - use for global pos

//Implement maze solving stuff written by Cian

Implement exploring jawn

Add yaw correction

Tune Lidar correction

Implement move command stacking

fix pose() command and figure out wtf wrong wit %

*/

//! Debugging Checklist:
/*
Does the command function terminate with a mCommand.completed = true?
Are the equivalencies in if() statements properly set? (i.e '=' vs '==')
Are while loops avoided as much as possible to keep functions interruptable?

*/
