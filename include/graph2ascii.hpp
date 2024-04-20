#pragma once

#include "Graph.hpp"

namespace mtrn3100 {

// COMPLETE THIS FUNCTION.
template <typename N, typename E>
char* graph2ascii(Graph<N, E> const& g) {
    // Take in maze size
    int const numRows = 5;
    int const numCols = 9;

    // Helper function to convert between (rows, cols) and number of chars.
    auto row2charPos = [](int const r) { return r * 2 + 1; };
    auto col2charPos = [](int const c) { return c * 4 + 2; };

    int const numCharRows = row2charPos(numRows);
    int const numCharCols = col2charPos(numCols);  
    char* maze = new char[numCharCols * numCharRows];

    // Helper function to access the maze with a 2D model.
    auto maze2d = [&maze, &numCharCols](unsigned const r, unsigned const c) -> char& {
        return maze[r * numCharCols + c];
    };

    // Initialise the maze values.
    for (int i = 0; i < numCharCols * numCharRows; i++) {
        maze[i] = ' ';
    }

    // Do new lines.
    for (int r = 0; r < numCharRows; r++) {
        maze2d(r, numCharCols - 1) = '\n';
    }

    // Terminate the string.
    maze2d(numCharRows - 1, numCharCols - 1) = '\0';

    // Creating every wall first
    for (int i = 0; i < numCharCols - 1; i++) {
        for (int j = 0; j < numCharRows; j++) {
            if (i % 4 != 0 && j % 2 == 0){
                maze2d(j, i) = '-';
            }
            if (i % 4 == 0 && j % 2 != 0){
                maze2d(j, i) = '|';
            }
        }
    }
    
    // Do external walls.
    int k = 1;
    while (k < (numRows * numCols + 1)) {
        int row = (k - 1) / numCols;
        int col = (k - 1) % numCols;
        if (row != numCharRows && col != numCharCols) {
            if (g.is_connected(k, k + numCols)) {
                maze2d(2*row + 2, col*4 + 1) = ' ';
                maze2d(2*row + 2, col*4 + 2) = ' ';
                maze2d(2*row + 2, col*4 + 3) = ' ';
                // std::cout << k << " S" << std::endl; need iostream for these
            }
            if (g.is_connected(k, k + 1)) {
                maze2d(2*row + 1, col*4 + 4) = ' ';
                // std::cout << k << " E" << std::endl;
            }
        }
        k++;
    }

    return maze;
}

}  // namespace mtrn3100