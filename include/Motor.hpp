#pragma once

#include <Arduino.h>

namespace mtrn3100 {

class Motor {
public:
    Motor(uint8_t analog, uint8_t input1, uint8_t input2) : analog_pin(analog), input1_pin(input1),input2_pin(input2) {
        //pinMode(analog_pin, OUTPUT);
        pinMode(input1_pin, OUTPUT);
        pinMode(input2_pin, OUTPUT);
      }

    // COMPLETE THIS FUNCTION.
    // This function sets the PWM of the motor and returns nothing. This function accepts a signed PWM signal with range
    // [-255, 255]. Any values outside this range are clamped to the limits.
    void setPWM(int16_t pwm) {
      if (pwm >=0){
        digitalWrite(input1_pin, 1);
        digitalWrite(input2_pin, 0);
      }else{
        //Serial.println(pwm);
        digitalWrite(input1_pin, 0);
        digitalWrite(input2_pin, 1);
        
      }
      analogWrite(analog_pin, abs(pwm));
      }

private:
    const uint8_t analog_pin;
    const uint8_t input1_pin;
    const uint8_t input2_pin;
};

}  // namespace mtrn3100
