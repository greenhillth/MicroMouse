#pragma once

#include <Arduino.h>
#include "shortest_path.hpp"
#include "graph2ascii.hpp"
#include "Graph.hpp"

class mazeInfo
{
public:
    int rows;
    int cols;
    int startRow;
    int startCol;
    int heading;
    int endRow;
    int endCol;
    mtrn3100::Graph<int, int> mazeGraph;

    bool initialised() { return ((rows != -1) && (cols != -1) && (startRow != -1) && (startCol != -1) && (heading != -1)); }
    mazeInfo(String mazeString);
    mazeInfo();
    String generateMotionPlan();
};
