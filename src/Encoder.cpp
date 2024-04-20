#pragma once

#include "../include/Encoder.hpp"

mtrn3100::Encoder::Encoder(uint8_t enc1, uint8_t enc2, void (*callback)()) : encoder1_pin(enc1), encoder2_pin(enc2)
{
    pinMode(encoder1_pin, INPUT_PULLUP);
    pinMode(encoder2_pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoder1_pin), callback, RISING);
    position = 0;
    counts = 0;
    prev_time = 0;
}

void mtrn3100::Encoder::readEncoder()
{
    noInterrupts();
    uint32_t trig_time = micros();
    uint32_t deltaT = 0;
    if (digitalRead(encoder2_pin))
    {
        direction = 1;
    }
    else
    {
        direction = -1;
    }
    if (trig_time > prev_time)
    {
        deltaT = (trig_time - prev_time);
        prev_time = trig_time;
    }
    interrupts();

    noInterrupts();
    const uint16_t counts_per_revolution = 274;
    counts += direction;
    float deltaS = static_cast<float>(direction) / counts_per_revolution * (2 * M_PI);
    position += deltaS;
    // add timeout if last reading was more than 0.5s ago
    if (deltaT != 0 && 5 * 1e5 > (micros() - trig_time))
    {
        velocity = deltaS * 1e6 / deltaT;
    }
    else
    {
        velocity = 0;
    }
    if (0 < velocity && velocity < 30)
    {
        velocityFilter.sample(velocity);
    }

    interrupts();
}

// Not inside Encoder because only free functions are interruptable.
void mtrn3100::setEncoder(mtrn3100::Encoder &encoder) { encoder.read = true; }

// namespace mtrn3100