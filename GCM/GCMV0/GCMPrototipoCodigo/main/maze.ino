#include "maze.hpp"

int16_t Maze::addCell() {
    if(count >= MAZE_MAX_CELLS) return -1;
    cells[count] = Cell();
    return (int16_t)(count++);
}

Cell& Maze::getCell(uint16_t index) {
    static Cell invalid;
    if(index >= count) return invalid;

    return cells[index];
}

uint16_t Maze::cellCount() const {
    return count;
}

void Maze::setWall(uint16_t index, WallDir dir) {
    if(index >= count) return;
    cells[index].wallDirections |= dir;
}

bool Maze::isWall(uint16_t index, WallDir dir) const {
    if(index >= count) return false;
    return cells[index].wallDirections & dir; 
} 

void Maze::setGoal(uint16_t index) {
    if(index >= count) return;
    cells[index].goal = true;
}

bool Maze::isGoal(uint16_t index) const {
    if(index >= count) return false;
    return cells[index].goal;
}