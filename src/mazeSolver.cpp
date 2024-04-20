#pragma once

#include "../include/mazeSolver.hpp"
mazeInfo::mazeInfo() : rows(-1), cols(-1), startRow(-1), startCol(-1), heading(-1), endRow(-1), endCol(-1) {}

mazeInfo::mazeInfo(String mazeString)
{

    // Establishing the size of the maze
    rows = 5;
    cols = 9;

    // A string array to contain all 50 elements of the maze data (start (x & y), heading, goal (x & y) and all 45 cells)
    String mazeElement[50];
    int mazeCount = 0;

    // An array to store the array data elements once they have been converted to ints
    int maze[50];

    // Split the string into substrings
    while (mazeString.length() > 0)
    {
        Serial.println("sub stringin");
        int index = mazeString.indexOf(' ');
        if (index == -1) // No space found
        {
            mazeElement[mazeCount++] = mazeString;
            break;
        }
        else
        {
            mazeElement[mazeCount++] = mazeString.substring(0, index);
            mazeString = mazeString.substring(index + 1);
        }
    }
    Serial.println("stop stringin");

    // Show the resulting substrings
    for (int i = 0; i < mazeCount; i++)
    {
        maze[i] = mazeElement[i].toInt();
    }

    startRow = maze[0];
    startCol = maze[1];
    heading = maze[2];
    endRow = maze[3];
    endCol = maze[4];

    mtrn3100::Graph<int, int> mazeGraph(1); // Creating the graph with 1 node

    // Adding the remaining nodes
    for (int j = 2; j <= (rows * cols); j++)
    {
        mazeGraph.insert_node(j);
    }
    Serial.println("stop stringin again!");

    const int mazeSize = mazeGraph.size();
    for (int i = 5; i < mazeSize + 5; i++)
    {

        int cellNum = i - 4;
        int nodeVal = maze[i];
        int inverseValue = 15 - nodeVal;

        if (inverseValue >= 8)
        {
            Serial.print("8Inserting edges at: ");
            Serial.print(cellNum);
            Serial.print(", ");
            Serial.print(cellNum - cols);
            Serial.print("\t and also: ");
            Serial.print(cellNum - cols);
            Serial.print(", ");
            Serial.println(cellNum);
            inverseValue -= 8;
            mazeGraph.insert_edge(cellNum, (cellNum - cols), 1);
            mazeGraph.insert_edge((cellNum - cols), cellNum, 1); // Bidirectional edges throughout
        }
        if (inverseValue >= 4)
        {
            Serial.print("4Inserting edges at: ");
            Serial.print(cellNum);
            Serial.print(", ");
            Serial.print(cellNum + 1);
            Serial.print("\t and also: ");
            Serial.print(cellNum + 1);
            Serial.print(", ");
            Serial.println(cellNum);
            inverseValue -= 4;
            mazeGraph.insert_edge(cellNum, (cellNum + 1), 1);
            mazeGraph.insert_edge((cellNum + 1), cellNum, 1);
        }
        if (inverseValue >= 2)
        {
            Serial.print("2Inserting edges at: ");
            Serial.print(cellNum);
            Serial.print(", ");
            Serial.print(cellNum + cols);
            Serial.print("\t and also: ");
            Serial.print(cellNum + cols);
            Serial.print(", ");
            Serial.println(cellNum);
            inverseValue -= 2;
            mazeGraph.insert_edge(cellNum, (cellNum + cols), 1);
            mazeGraph.insert_edge((cellNum + cols), cellNum, 1);
        }
        if (inverseValue >= 1)
        {
            Serial.print("1Inserting edges at: ");
            Serial.print(cellNum);
            Serial.print(", ");
            Serial.print(cellNum - 1);
            Serial.print("\t and also: ");
            Serial.print(cellNum - 1);
            Serial.print(", ");
            Serial.println(cellNum);
            inverseValue -= 1;
            mazeGraph.insert_edge(cellNum, (cellNum - 1), 1);
            mazeGraph.insert_edge((cellNum - 1), cellNum, 1);
        }
    }

    Serial.println("wtf");
    Serial.println("wtf");
}

String mazeInfo::generateMotionPlan()
{
    int rows = rows;
    int cols = cols;

    int startRow = startRow;
    int startCol = startCol;
    int heading = heading;
    int endRow = endRow;
    int endCol = endCol;

    char *asciiMaze = mtrn3100::graph2ascii(this->mazeGraph);
    Serial.print(asciiMaze);

    int start = (startRow * cols) + (startCol + 1);
    int goal = (endRow * cols) + (endCol + 1);

    // Debug print to show the starting and ending cell of the maze
    Serial.println("breakpoint 1.7: inside function");
    mtrn3100::LinkedList<int> path = mtrn3100::bfs_single(this->mazeGraph, start, goal);

    String robotInstructions = "";
    int prevCell = start;
    int currentCell = start;

    for (int k = 1; k < path.size(); k++)
    {
        currentCell = path.get(k)->value;

        switch (heading)
        {
        case 0: // North
            if (currentCell == prevCell + 1)
            { // To East
                robotInstructions += ("RF");
                heading++;
            }
            else if (currentCell == prevCell - cols)
            { // To North
                robotInstructions += ("F");
            }
            else if (currentCell == prevCell - 1)
            { // To West
                robotInstructions += ("LF");
                heading = 3;
            }
            else
            {
                robotInstructions += ("Invalid3");
            }
            break;
        case 1: // East
            if (currentCell == prevCell + 1)
            { // To East
                robotInstructions += ("F");
            }
            else if (currentCell == prevCell - cols)
            { // To North
                robotInstructions += ("LF");
                heading--;
            }
            else if (currentCell == prevCell + cols)
            { // To South
                robotInstructions += ("RF");
                heading++;
            }
            else
            {
                robotInstructions += ("Invalid0");
            }
            break;
        case 2: // South
            if (currentCell == prevCell + 1)
            { // To East
                robotInstructions += ("LF");
                heading--;
            }
            else if (currentCell == prevCell - 1)
            { // To West
                robotInstructions += ("RF");
                heading++;
            }
            else if (currentCell == prevCell + cols)
            { // To South
                robotInstructions += ("F");
            }
            else
            {
                robotInstructions += ("Invalid1");
            }
            break;
        case 3: // West
            if (currentCell == prevCell - 1)
            { // To West
                robotInstructions += ("F");
            }
            else if (currentCell == prevCell - cols)
            { // To North
                robotInstructions += ("RF");
                heading = 0;
            }
            else if (currentCell == prevCell + cols)
            { // To South
                robotInstructions += ("LF");
                heading--;
            }
            else
            {
                robotInstructions += ("Invalid2");
            }
            break;
        }

        // Debug that explores step-by-step movements and changes in heading
        prevCell = currentCell;
    }
    Serial.print("breakpoint 1.9: finished determining motion plan. Printing: ");
    Serial.println(robotInstructions);
    return robotInstructions;
}