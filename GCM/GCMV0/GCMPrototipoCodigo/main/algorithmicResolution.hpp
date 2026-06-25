#pragma once

#include <Arduino.h>
#include "maze.hpp"

constexpr uint16_t INVALID_INDEX = 0xFFFF;
constexpr uint16_t FLOOD_INF     = 0xFFFF;

struct PathStep {
    uint16_t from = INVALID_INDEX;
    uint16_t to   = INVALID_INDEX;
    Heading heading = NORTH;
};

class AlgorithmicResolution {
public:
    void reset();

    bool solveDFS(Maze& maze, uint16_t startIndex, uint16_t goalIndex);
    bool solveFloodFill(Maze& maze, uint16_t startIndex, uint16_t goalIndex);

    uint16_t dfsPathLength() const   { return _dfsPathLen; }
    uint16_t floodPathLength() const { return _floodPathLen; }

    const PathStep* dfsPath() const   { return _dfsPath; }
    const PathStep* floodPath() const { return _floodPath; }

    void printDFSPath(Stream& out) const;
    void printFloodPath(Stream& out) const;

private:
    bool _visited[MAZE_MAX_CELLS];
    uint16_t _flood[MAZE_MAX_CELLS];
    uint16_t _parent[MAZE_MAX_CELLS];
    Heading _parentHeading[MAZE_MAX_CELLS];

    PathStep _dfsPath[MAZE_MAX_CELLS];
    uint16_t _dfsPathLen = 0;

    PathStep _floodPath[MAZE_MAX_CELLS];
    uint16_t _floodPathLen = 0;

    static WallDir headingToWall(Heading h);
    static Heading turnRight(Heading h);
    static Heading turnLeft(Heading h);
    static Heading turnBack(Heading h);
    static void stepFromHeading(Heading h, int16_t& dx, int16_t& dy);

    int16_t findCellByCoord(Maze& maze, int16_t x, int16_t y) const;
    int16_t neighborIndex(Maze& maze, uint16_t index, Heading h) const;

    bool dfsRecursive(Maze& maze, uint16_t current, uint16_t goal);
    void reconstructDFSPath(uint16_t startIndex, uint16_t goalIndex);

    void floodFill(Maze& maze, uint16_t goalIndex);
    void reconstructFloodPath(Maze& maze, uint16_t startIndex, uint16_t goalIndex);

    void clearArrays();
};
