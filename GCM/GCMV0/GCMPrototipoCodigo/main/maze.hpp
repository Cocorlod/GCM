#pragma once
#include <Arduino.h>

enum WallDir : uint8_t { WALL_N = 1, WALL_E = 2, WALL_S = 4, WALL_W = 8 }

class Maze {
    public: 
        bool isWall(uint16_t x, uint16_t y, WallDir dir) const;
        void setWall(uint16_t x, uint16_t y, WallDir dir);
    private:

}