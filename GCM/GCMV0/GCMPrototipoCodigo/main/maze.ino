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

void Maze::mazeUpdate(ToFSensor& tof, Heading heading) {;
    uint16_t currentIndex = cellCount() - 1;  
    if(tof.isThereWall(FRONT)) {
        setWall(currentIndex, localToGlobal(FRONT, heading));
    }

    if(tof.isThereWall(LEFT)) {
        setWall(currentIndex, localToGlobal(LEFT, heading));
    }

    if(tof.isThereWall(RIGHT)) {
        setWall(currentIndex, localToGlobal(RIGHT, heading));
    }
}

WallDir Maze::localToGlobal(WallSides side, Heading heading) const {
    int dir = ((int)heading + (int)side + 4) % 4;
    return (WallDir)(1 << dir);
}

void Maze::updateCoordinates(Heading heading) {
    if(count == 0) {
        cells[0].x = 0;
        cells[0].y = 0;
        return;
    }

    cells[count].x = cells[count - 1].x;   
    cells[count].y = cells[count - 1].y;

    if(heading == NORTH) {
        cells[count].y++;
    } else if(heading == EAST) {
        cells[count].x++;
    } else if(heading == SOUTH) {
        cells[count].y--;
    } else if(heading == WEST) {
        cells[count].x--;
    }
}