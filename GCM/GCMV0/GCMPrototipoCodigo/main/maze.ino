#include "maze.hpp"

static WallDir oppositeWall(WallDir d) {
    switch(d) {
        case WALL_N: return WALL_S;
        case WALL_E: return WALL_W;
        case WALL_S: return WALL_N;
        case WALL_W: return WALL_E;
    }
    return WALL_N;
}

static Heading wallDirToHeading(WallDir d) {
    switch(d) {
        case WALL_N: return NORTH;
        case WALL_E: return EAST;
        case WALL_S: return SOUTH;
        case WALL_W: return WEST;
    }
    return NORTH;
}

void Maze::reset() {
    count = 0;
    for(uint16_t i = 0; i < MAZE_MAX_CELLS; i++) {
        cells[i] = Cell();
    }
}

int16_t Maze::addCell() {
    if(count >= MAZE_MAX_CELLS) return -1;
    cells[count] = Cell();
    return (int16_t)(count++);
}   

Cell& Maze::getCell(uint16_t index) {
    static Cell invalid;

    if(index >= count) {
        return invalid;
    }

    return cells[index];
}

const Cell& Maze::getCell(uint16_t index) const {
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
    if(index < 0 || index >= (int16_t)count) return -1;

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

    int16_t neighbor = adjacentCell(index, wallDirToHeading(dir));
    if(neighbor != -1) {
        cells[neighbor].wallDirections |= oppositeWall(dir);
    }
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

int16_t Maze::visitCell(int16_t fromIndex, Heading heading) {
    int16_t x = 0;
    int16_t y = 0;

    if(fromIndex >= 0 && fromIndex < (int16_t)count) {
        x = cells[fromIndex].x;
        y = cells[fromIndex].y;

        switch(heading) {
            case NORTH: y++; break;
            case EAST:  x++; break;
            case SOUTH: y--; break;
            case WEST:  x--; break;
        }
    }
    int16_t existing = cellAt(x, y);
    if(existing != -1) {
        cells[existing].visited = true;
        return existing;
    }

    int16_t idx = addCell();
    if(idx == -1) {
        return fromIndex >= 0 ? fromIndex : 0;
    }

    cells[idx].x = x;
    cells[idx].y = y;
    cells[idx].visited = true;
    return idx;
}

void Maze::recordWalls(ToFSensor& tof, uint16_t index, Heading heading) {
    if(index >= count) return;

    if(tof.isThereWall(FRONT)) {
        setWall(index, localToGlobal(FRONT, heading));
    }

    if(tof.isThereWall(LEFT)) {
        setWall(index, localToGlobal(LEFT, heading));
    }

    if(tof.isThereWall(RIGHT)) {
        setWall(index, localToGlobal(RIGHT, heading));
    }
}

WallDir Maze::localToGlobal(WallSides side, Heading heading) const {
    int dir = ((int)heading + (int)side + 4) % 4;
    return (WallDir)(1 << dir);
}