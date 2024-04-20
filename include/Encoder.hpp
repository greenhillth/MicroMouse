#pragma once

#include <Arduino.h>
#include <math.h>
#include "MovingAverageFilter.hpp"

namespace mtrn3100
{

    class Encoder
    {
    public:
        Encoder(uint8_t enc1, uint8_t enc2, void (*callback)());
        void readEncoder();

    public:
        const uint8_t encoder1_pin;
        const uint8_t encoder2_pin;
        uint32_t prev_time;
        int8_t direction;
        int8_t counts;
        float velocity;
        float position;
        bool read = false;
        MovingAverageFilter<float, 10> velocityFilter;
    };

    // Not inside Encoder because only free functions are interruptable.
    void setEncoder(mtrn3100::Encoder &encoder);

} // namespace mtrn3100