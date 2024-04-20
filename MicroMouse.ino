#include "include/MicroMouse.hpp"

// config info
//  Encoder pins.
uint8_t encoderL_a{18};
uint8_t encoderL_b{22};
uint8_t encoderR_a{19};
uint8_t encoderR_b{23};

// Drive params
const int pwmL{2};
const int input1L{4};
const int input2L{3};

const int pwmR{7};
const int input1R{5};
const int input2R{6};

constexpr uint8_t pwm{100};
constexpr uint16_t forwardDuration{500};
constexpr uint16_t turnDuration{300};

// LiDAR Pins & addresses
const int enablePinL{8};
const int enablePinR{9};
const int enablePinF{10};

const int addressL{0x20};
const int addressR{0x22};
const int addressF{0x24};

// Set up motors.
mtrn3100::Motor l_motor(pwmL, input1L, input2L);
mtrn3100::Motor r_motor(pwmR, input1R, input2R);

// Set up encoders.
void readLeftEncoder();
void readRightEncoder();
mtrn3100::Encoder l_encoder(encoderL_a, encoderL_b, readLeftEncoder);
mtrn3100::Encoder r_encoder(encoderR_a, encoderR_b, readRightEncoder);

void readLeftEncoder() { l_encoder.readEncoder(); }
void readRightEncoder() { r_encoder.readEncoder(); }

// Set up PIDs.
mtrn3100::PIDController pidLinearL(10, 50, 0, 0.0005, 60, 255, 1);
mtrn3100::PIDController pidLinearR(10, 50, 0, 0.0005, 60, 255, 1);
mtrn3100::PIDController pidRotationalL(.25, 0, 0, 55, 0, 90, .1);
mtrn3100::PIDController pidRotationalR(.25, 0, 0, 55, 0, 90, .1);

MotorAssembly leftAssembly(&l_motor, &l_encoder, pidLinearL, pidRotationalL);
MotorAssembly rightAssembly(&r_motor, &r_encoder, pidLinearR, pidRotationalR);

mtrn3100::GYRO gyro(0);

VL6180X sensorL;
VL6180X sensorR;
VL53L0X sensorF;

lidarObj lidars(&sensorL, &sensorR, &sensorF);

void startLidars()
{
  pinMode(enablePinL, OUTPUT);
  pinMode(enablePinR, OUTPUT);
  pinMode(enablePinF, OUTPUT);

  digitalWrite(enablePinL, LOW);
  digitalWrite(enablePinR, LOW);
  digitalWrite(enablePinF, LOW);
  delay(1000);

  Serial.println("Starting Left Sensor...");
  digitalWrite(enablePinL, HIGH);
  delay(50);
  sensorL.init();
  sensorL.configureDefault();
  sensorL.setAddress(addressL); // Set the new address.
  sensorL.setTimeout(25);
  Serial.print("sensorL I2C address: 0x");
  Serial.println(sensorL.readReg(0x212), HEX);
  delay(100);

  Serial.println("Starting Right sensor...");
  digitalWrite(enablePinR, HIGH);
  delay(50);
  sensorR.init();
  sensorR.configureDefault();
  sensorR.setAddress(addressR); // Set the new address.
  sensorR.setTimeout(25);
  Serial.print("sensorR I2C address: 0x");
  Serial.println(sensorR.readReg(0x212), HEX);
  delay(100);

  digitalWrite(enablePinF, HIGH);
  delay(100);
  sensorF.init();
  sensorF.setTimeout(10);
  Serial.print("sensorF I2C address: 0x");
  Serial.println(sensorF.readReg(0x212), HEX);
  Serial.println("Sensors ready!");
  delay(100);
}

void startBluetooth()
{
  Serial3.begin(9600);
  while (!Serial3.read())
  {
    Serial3.println("Waiting");
  }
  Serial3.println("starting");
}

MicroMouse *Robot = new MicroMouse(leftAssembly, rightAssembly, lidars, &gyro);

void setup()
{
  delay(100);
  Serial.begin(115200);
  Serial.println("henlo");
  Wire.begin();

  // startBluetooth();
  startLidars();

  gyro.begin();
  gyro.reset();
}

void loop()
{
  Robot->run();
}
