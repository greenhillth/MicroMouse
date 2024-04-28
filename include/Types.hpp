#pragma once
#include <Arduino.h>

#include "Tuple.hpp"

/*

Cell co-ord layout:
Each cell is based on the relative position of the robot to the
central bounding box
Each box 250x250 mm
Robot

     --
    |--->
     --

+-----------------------------------------------+
| 0,6 | 1,6 | 2,5 | 3,5 | 4,5 | 5,5 | 6,5 | 7,5 |
|-----|-----|-----|-----|-----|-----|-----|-----|
| 0,5 | 1,5 | 2,5 | 3,5 | 4,5 | 5,5 | 6,5 | 7,5 |
|-----|-----|-----|-----|-----|-----|-----|-----|
| 0,4 | 1,4 | 2,4 | 3,4 | 4,4 | 5,4 | 6,4 | 7,4 |
|-----|-----|-----|-----|-----|-----|-----|-----|
| 0,3 | 1,3 | 2,3 | 3,3 | 4,3 | 5,3 | 6,3 | 7,3 |
|-----|-----|-----|-----|-----|-----|-----|-----|
| 0,2 | 1,2 | 2,2 | 3,2 | 4,2 | 5,2 | 6,2 | 7,2 |
|-----|-----|-----|-----|-----|-----|-----|-----|
| 0,1 | 1,1 | 2,1 | 3,1 | 4,1 | 5,1 | 6,1 | 7,1 |
|-----|-----|-----|-----|-----|-----|-----|-----|
| 0,0 | 1,0 | 2,0 | 3,0 | 4,0 | 5,0 | 6,0 | 7,0 |
+-----------------------------------------------+


*/
struct polar
{
    float arg;
    float theta;
};

struct coords
{
    float x;
    float y;
    float heading;
    // mm tolerance
    float tol;
    // rotational tolerance (degrees)
    float rtol;

    // constructor for x, y, z
    coords(float x, float y, float heading) : x(x), y(y), heading(heading), tol(5), rtol(2){};
    // constructor for cell coords
    coords(mtrn3100::Tuple<int, int, float> cellCoords) : tol(5), rtol(2)
    {
        x = mtrn3100::get<0>(cellCoords) * 250 + 125;
        y = mtrn3100::get<1>(cellCoords) * 250 + 125;
        heading = mtrn3100::get<2>(cellCoords);
    };
    mtrn3100::Tuple<int, int, float> getCellCoords()
    {
        return {static_cast<int>(x - 125), static_cast<int>(y - 125), heading};
    }
    // equality overload
    bool operator==(const coords &other) const
    {
        return (fabs(x - other.x) < tol) && (fabs(y - other.y) < tol) && (fabs(heading - other.heading) < rtol);
    }
    // minus overload for arg, angle between two coords
    polar operator-(const coords &other) const
    {
        polar result;
        result.arg = sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y));
        float angleRadians = atan2(y - other.y, x - other.x);
        result.theta = angleRadians * 180 / PI;
        return result;
    }
};

/*
Movement process:
1. Rotate
2. Move straight
3. Rotate to desired final heading
*/

constexpr int NOPLAN{-1};
constexpr int COMPLETE{0};
constexpr int INITR{1};
constexpr int FINALR{3};

struct movePlan
{
    bool valid;
    int currentStep;
    float lineardist;
    float initR;
    float finalR;

    movePlan(coords start, coords end)
    {
        if (start == end)
        {
            valid = false;
            currentStep = COMPLETE;
        }
        else
        {
            valid = true;
            currentStep = INITR;
            polar diff = end - start;
            initR = diff.theta - start.heading;
            lineardist = diff.arg;
            finalR = end.heading - diff.theta;
        }
    }
    movePlan() : valid(false), currentStep(NOPLAN){};
};

constexpr int LEFT{0};
constexpr int RIGHT{1};

constexpr int ROTATION_CONST{264};

struct targets
{
    float left;
    float right;
    float initL;
    float initR;
    bool set;

    targets() : left(0), right(0), initL(0), initR(0), set(false){};

    targets(float ldist, float lpos, float rpos) : set(true), initL(lpos), initR(rpos)
    {
        left = ldist / ROTATION_CONST + lpos;
        right = ldist / ROTATION_CONST + rpos;
    }
};