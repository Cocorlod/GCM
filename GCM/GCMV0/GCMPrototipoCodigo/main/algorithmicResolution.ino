#include "algorithmicResolution.h"

AlgorithmicResolution solver;
Maze maze;

void AlgorithmicResolution::clearArrays() {
    for(uint16_t i = 0; i < MAZE_MAX_CELLS; i++) {
        _visited[i] = false;
        _flood[i] = FLOOD_INF;
        _parent[i] = INVALID_INDEX;
        _parentHeading[i] = NORTH;
    }

    _dfsPathLen = 0;
    _floodPathLen = 0;
}

void AlgorithmicResolution::reset() {
    clearArrays();
}

WallDir AlgorithmicResolution::headingToWall(Heading h) {
    return (WallDir)(1 << (uint8_t)h);
}

Heading AlgorithmicResolution::turnRight(Heading h) {
    return (Heading)(((uint8_t)h + 1) % 4);
}

Heading AlgorithmicResolution::turnLeft(Heading h) {
    return (Heading)(((uint8_t)h + 3) % 4);
}

Heading AlgorithmicResolution::turnBack(Heading h) {
    return (Heading)(((uint8_t)h + 2) % 4);
}

void AlgorithmicResolution::stepFromHeading(Heading h, int16_t& dx, int16_t& dy) {
    dx = 0;
    dy = 0;

    switch(h) {
        case NORTH: dy =  1; break;
        case EAST:  dx =  1; break;
        case SOUTH: dy = -1; break;
        case WEST:  dx = -1; break;
    }
}

int16_t AlgorithmicResolution::findCellByCoord(Maze& maze, int16_t x, int16_t y) const {
    for(uint16_t i = 0; i < maze.cellCount(); i++) {
        const Cell& c = maze.getCell(i);
        if(c.x == x && c.y == y) {
            return (int16_t)i;
        }
    }

    return -1;
}

int16_t AlgorithmicResolution::neighborIndex(Maze& maze, uint16_t index, Heading h) const {
    if(index >= maze.cellCount()) {
        return -1;
    }

    const Cell& c = maze.getCell(index);

    int16_t dx = 0;
    int16_t dy = 0;
    stepFromHeading(h, dx, dy);

    return findCellByCoord(maze, c.x + dx, c.y + dy);
}

// -----------------------------------------------------------------------------
// DFS
// -----------------------------------------------------------------------------

bool AlgorithmicResolution::dfsRecursive(Maze& maze, uint16_t current, uint16_t goal) {
    if(current >= maze.cellCount()) {
        return false;
    }

    _visited[current] = true;

    if(current == goal) {
        return true;
    }

    // Exploration order: straight, right, left, back
    const Heading currentHeading = _parentHeading[current];
    const Heading candidates[4] = {
        currentHeading,
        turnRight(currentHeading),
        turnLeft(currentHeading),
        turnBack(currentHeading)
    };

    for(uint8_t i = 0; i < 4; i++) {
        Heading nextHeading = candidates[i];
        WallDir wall = headingToWall(nextHeading);

        if(maze.isWall(current, wall)) {
            continue;
        }

        int16_t nextIndex = neighborIndex(maze, current, nextHeading);
        if(nextIndex < 0) {
            continue;
        }

        if(_visited[(uint16_t)nextIndex]) {
            continue;
        }

        _parent[(uint16_t)nextIndex] = current;
        _parentHeading[(uint16_t)nextIndex] = nextHeading;

        if(dfsRecursive(maze, (uint16_t)nextIndex, goal)) {
            return true;
        }
    }

    return false;
}

void AlgorithmicResolution::reconstructDFSPath(uint16_t startIndex, uint16_t goalIndex) {
    _dfsPathLen = 0;

    if(startIndex == goalIndex) {
        return;
    }

    uint16_t cur = goalIndex;

    while(cur != startIndex && cur != INVALID_INDEX && _dfsPathLen < MAZE_MAX_CELLS) {
        uint16_t prev = _parent[cur];
        if(prev == INVALID_INDEX) {
            break;
        }

        _dfsPath[_dfsPathLen].from = prev;
        _dfsPath[_dfsPathLen].to = cur;
        _dfsPath[_dfsPathLen].heading = _parentHeading[cur];
        _dfsPathLen++;

        cur = prev;
    }

    // Reverse so the path goes start -> goal
    for(uint16_t i = 0; i < _dfsPathLen / 2; i++) {
        PathStep tmp = _dfsPath[i];
        _dfsPath[i] = _dfsPath[_dfsPathLen - 1 - i];
        _dfsPath[_dfsPathLen - 1 - i] = tmp;
    }
}

bool AlgorithmicResolution::solveDFS(Maze& maze, uint16_t startIndex, uint16_t goalIndex) {
    clearArrays();

    if(startIndex >= maze.cellCount() || goalIndex >= maze.cellCount()) {
        return false;
    }

    // DFS tree root
    _parent[startIndex] = INVALID_INDEX;
    _parentHeading[startIndex] = NORTH;

    bool found = dfsRecursive(maze, startIndex, goalIndex);
    if(found) {
        reconstructDFSPath(startIndex, goalIndex);
    }

    return found;
}

void AlgorithmicResolution::printDFSPath(Stream& out) const {
    out.println(F("DFS path:"));
    for(uint16_t i = 0; i < _dfsPathLen; i++) {
        out.print(F("  "));
        out.print(_dfsPath[i].from);
        out.print(F(" -> "));
        out.print(_dfsPath[i].to);
        out.print(F("  heading="));
        out.println((uint8_t)_dfsPath[i].heading);
    }
}

// -----------------------------------------------------------------------------
// Flood Fill
// -----------------------------------------------------------------------------

void AlgorithmicResolution::floodFill(Maze& maze, uint16_t goalIndex) {
    for(uint16_t i = 0; i < MAZE_MAX_CELLS; i++) {
        _flood[i] = FLOOD_INF;
    }

    if(goalIndex >= maze.cellCount()) {
        return;
    }

    uint16_t queue[MAZE_MAX_CELLS];
    uint16_t qHead = 0;
    uint16_t qTail = 0;

    _flood[goalIndex] = 0;
    queue[qTail++] = goalIndex;

    while(qHead < qTail) {
        uint16_t current = queue[qHead++];
        uint16_t base = _flood[current];

        for(uint8_t i = 0; i < 4; i++) {
            Heading h = (Heading)i;
            WallDir wall = headingToWall(h);

            if(maze.isWall(current, wall)) {
                continue;
            }

            int16_t next = neighborIndex(maze, current, h);
            if(next < 0) {
                continue;
            }

            uint16_t nextIndex = (uint16_t)next;

            if(_flood[nextIndex] > base + 1) {
                _flood[nextIndex] = base + 1;

                if(qTail < MAZE_MAX_CELLS) {
                    queue[qTail++] = nextIndex;
                }
            }
        }
    }
}

void AlgorithmicResolution::reconstructFloodPath(Maze& maze, uint16_t startIndex, uint16_t goalIndex) {
    _floodPathLen = 0;

    if(startIndex >= maze.cellCount() || goalIndex >= maze.cellCount()) {
        return;
    }

    if(_flood[startIndex] == FLOOD_INF) {
        return;
    }

    uint16_t cur = startIndex;

    while(cur != goalIndex && _floodPathLen < MAZE_MAX_CELLS) {
        uint16_t bestNext = INVALID_INDEX;
        Heading bestHeading = NORTH;
        uint16_t bestValue = _flood[cur];

        for(uint8_t i = 0; i < 4; i++) {
            Heading h = (Heading)i;
            WallDir wall = headingToWall(h);

            if(maze.isWall(cur, wall)) {
                continue;
            }

            int16_t next = neighborIndex(maze, cur, h);
            if(next < 0) {
                continue;
            }

            uint16_t nextIndex = (uint16_t)next;
            if(_flood[nextIndex] < bestValue) {
                bestValue = _flood[nextIndex];
                bestNext = nextIndex;
                bestHeading = h;
            }
        }

        if(bestNext == INVALID_INDEX) {
            break;
        }

        _floodPath[_floodPathLen].from = cur;
        _floodPath[_floodPathLen].to = bestNext;
        _floodPath[_floodPathLen].heading = bestHeading;
        _floodPathLen++;

        cur = bestNext;
    }
}

bool AlgorithmicResolution::solveFloodFill(Maze& maze, uint16_t startIndex, uint16_t goalIndex) {
    if(startIndex >= maze.cellCount() || goalIndex >= maze.cellCount()) {
        return false;
    }

    floodFill(maze, goalIndex);
    reconstructFloodPath(maze, startIndex, goalIndex);

    return (_floodPathLen > 0 || startIndex == goalIndex);
}

void AlgorithmicResolution::printFloodPath(Stream& out) const {
    out.println(F("Flood fill path:"));
    for(uint16_t i = 0; i < _floodPathLen; i++) {
        out.print(F("  "));
        out.print(_floodPath[i].from);
        out.print(F(" -> "));
        out.print(_floodPath[i].to);
        out.print(F("  heading="));
        out.println((uint8_t)_floodPath[i].heading);
    }
}

// -----------------------------------------------------------------------------
// Example sketch entry points
// -----------------------------------------------------------------------------

static int16_t findGoalIndex(Maze& maze) {
    for(uint16_t i = 0; i < maze.cellCount(); i++) {
        if(maze.isGoal(i)) {
            return (int16_t)i;
        }
    }
    return -1;
}

void setup() {
    Serial.begin(115200);
    while(!Serial) {
        delay(10);
    }

    Serial.println(F("\nAlgorithmicResolution base loaded"));

    // Ensure there is at least one start cell.
    if(maze.cellCount() == 0) {
        int16_t idx = maze.addCell();
        if(idx >= 0) {
            maze.getCell((uint16_t)idx).x = 0;
            maze.getCell((uint16_t)idx).y = 0;
        }
    }

    solver.reset();

    Serial.println(F("Waiting for a mapped maze with a goal cell..."));
}

void loop() {
    // This file is intentionally algorithmic-only.
    // Once your exploration layer has filled the maze and set one goal cell,
    // you can call:

    int16_t goalIndex = findGoalIndex(maze);
    if(goalIndex >= 0 && maze.cellCount() > 0) {
        uint16_t startIndex = 0;

        if(solver.solveDFS(maze, startIndex, (uint16_t)goalIndex)) {
            solver.printDFSPath(Serial);
        }

        if(solver.solveFloodFill(maze, startIndex, (uint16_t)goalIndex)) {
            solver.printFloodPath(Serial);
        }

        // Prevent spamming Serial forever.
        while(true) {
            delay(1000);
        }
    }

    delay(100);
}