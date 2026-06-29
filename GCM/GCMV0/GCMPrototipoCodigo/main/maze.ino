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

int16_t Maze::cellAt(int16_t x, int16_t y) const {
    for(uint16_t i = 0; i < count; i++) {
        if(cells[i].x == x && cells[i].y == y) {
            return (int16_t)i;
        }
    }
    return -1;  
}

int16_t Maze::adjacentCell(int16_t index, Heading heading) const {
    if(index >= count) return -1;

    int16_t x = cells[index].x;
    int16_t y = cells[index].y;

    switch(heading) {
        case NORTH: y++;
        break;
        case EAST:  x++;
        break;
        case SOUTH: y--;
        break;
        case WEST:  x--;
        break;
    }

    return cellAt(x, y);
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
    if(count == 0) return;
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

    if(currentIndex == 0){
        cells[0].x = 0;
        cells[0].y = 0;
    } else {
        cells[currentIndex].x = cells[currentIndex-1].x;
        cells[currentIndex].y = cells[currentIndex-1].y;

        switch(heading){    
            case NORTH:
                cells[currentIndex].y++;
                break;

            case EAST:
                cells[currentIndex].x++;
                break;

            case SOUTH:
                cells[currentIndex].y--;
                break;

            case WEST:
                cells[currentIndex].x--;
                break;
        }
    }   
    cells[currentIndex].visited = true;
}

WallDir Maze::localToGlobal(WallSides side, Heading heading) const {
    int dir = ((int)heading + (int)side + 4) % 4;
    return (WallDir)(1 << dir);
}