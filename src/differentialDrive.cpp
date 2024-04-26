#include "include/differentialDrive.hpp"

// constructor
differentialDrive::differentialDrive() {

}
bool differentialDrive::calibrate(IMU &imu) {
// get base PWM of motors for movement
while (left->encoderPos() == init) {
    left->setPWM
}

}


// move to position, return true when complete
bool differentialDrive::move(coords dest) {

}

// move to waypoints, return true when all moves complete
bool differentialDrive::move(coords dest...) {

}