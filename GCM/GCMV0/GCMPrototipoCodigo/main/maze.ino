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

void Maze::mazeUpdate(ToFSensor& tof, Heading heading) {
    bool cellAdded = cellAdded();

    if(!cellAdded) {
        uint16_t currentIndex = maze.cellCount() - 1;  
    
        if (tof.isThereWall(FRONT)) {
            maze.setWall(currentIndex, localToGlobal(FRONT, heading));
        }
        if(tof.isThereWall(LEFT)) {
            maze.setWall(currentIndex, localToGlobal(LEFT, heading));
        }
        if(tof.isThereWall(RIGHT)) {
            maze.setWall(currentIndex, localToGlobal(RIGHT, heading));
        }

        maze.addCell();
    }
    cellAdded = true;
}

WallDir Maze::localToGlobal(WallSides side, Heading heading) const {
    int dir = ((int)heading + (int)side + 4) % 4;
    return (WallDir)(1 << dir);
}