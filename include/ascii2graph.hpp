#pragma once

#include <Arduino.h>
#include "Graph.hpp"

namespace mtrn3100
{

    // Converts a maze string to a graph that uses indexes for nodes.
    Graph<int, int> ascii2graph(String maze);
} // namespace mtrn3100
