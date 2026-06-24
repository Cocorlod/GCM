#pragma once

#include <Arduino.h>
#include <stdint.h>
#include "ToF_Setup.hpp"

constexpr uint16_t MAZE_MAX_CELLS = 1024;

struct Cell {
    uint8_t wallDirections = 0; // Utilizamos una bitmask 
    // para guardar en una unica variable la existencia de las paredes en esa 
    // celda en el eje cardinal

    bool goal = false;
};

enum WallDir : uint8_t {
    WALL_N = 1,     
    WALL_E = 2,
    WALL_S = 4,
    WALL_W = 8
};

enum Heading : uint8_t {
    NORTH = 0,
    EAST,
    SOUTH,
    WEST
};

class Maze {
    public:
        int16_t addCell();

        Cell& getCell(uint16_t index);

        uint16_t cellCount() const;

        void setWall(uint16_t index, WallDir dir);
        bool isWall(uint16_t index, WallDir dir) const;

        void setGoal(uint16_t index);
        bool isGoal(uint16_t index) const;

        void mazeUpdate(Maze& maze, ToFSensor& tof, Heading heading);
        WallDir localToGlobal(WallSides side, Heading heading) const;
    private:
        Cell cells[MAZE_MAX_CELLS];
        uint16_t count = 0;
};