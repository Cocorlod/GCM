#include "algorithmicResolution.hpp"
#include "bluetooth.hpp"
#include <Preferences.h>

static bool goalFound = false;
static int16_t dfsStack[MAZE_MAX_CELLS];
static int16_t dfsStackSize = 0;
static bool dfsDone = false;

static void pushCell(int16_t cellIndex) {
    if (dfsStackSize < (int16_t)MAZE_MAX_CELLS) {
        dfsStack[dfsStackSize++] = cellIndex;
    }
}

static int16_t popCell() {
    if (dfsStackSize == 0) return -1;
    return dfsStack[--dfsStackSize];
}

static int16_t peekCell() {
    if (dfsStackSize == 0) return -1;
    return dfsStack[dfsStackSize - 1];
}

bool explorationComplete() {
    return dfsDone || goalFound;
}

static constexpr float CELL_LENGTH_MM = 280.0f;
static constexpr float CELL_TRAVEL_TIMEOUT_MS = 3000;

static uint32_t cellStartTime = 0;
static long cellStartLeftCount = 0;
static long cellStartRightCount = 0;
static bool cellTravelActive = false;

void beginCellTravel() {
    cellStartTime = millis();
    cellStartLeftCount = leftEncoder.getCount();
    cellStartRightCount = rightEncoder.getCount();
    cellTravelActive = true;
}

bool cellComplete() {
    if (!cellTravelActive) return false;
 
    long dl = abs(leftEncoder.getCount() - cellStartLeftCount);
    long dr = abs(rightEncoder.getCount() - cellStartRightCount);
    float distanceMM = ((dl + dr) / 2.0f) * MM_PER_COUNT;

    if (distanceMM > CELL_LENGTH_MM) {
        cellTravelActive = false;
        return true;
    }
 
    if (millis() - cellStartTime >= CELL_TRAVEL_TIMEOUT_MS) {
        debugPrint("Cell travel timed out before reaching expected distance (possible stall/slip)");
        cellTravelActive = false;
        return true;
    }
 
    return false;
}

bool goalDetected() {
    static uint8_t consecutiveHits = 0;
    static constexpr uint8_t REQUIRED_CONSECUTIVE = 4;

    if (analogRead(IR_PIN) >= IR_THRESHOLD) {
        if (consecutiveHits < 255) consecutiveHits++;
    } else {
        consecutiveHits = 0;
    }

    return consecutiveHits >= REQUIRED_CONSECUTIVE;
}

void commitGoal(Maze& maze, uint16_t currentCell) {
    if (goalFound) return;

    maze.setGoal(currentCell);
    goalFound = true;

    debugPrintf("Goal reached at cell %u", currentCell);

    stopMotors();

    int16_t startCell = maze.cellAt(0, 0);

    if (startCell < 0) {
        debugPrint("Flood fill: start cell (0,0) not found (unexpected)");
        return;
    }

    buildSpeedrunPath(maze, (uint16_t)startCell, currentCell);
}

Heading rotate(Turn t, Heading h) {
    switch (t) {
        case LEFT:  return (Heading)((h + 3) % 4);
        case RIGHT: return (Heading)((h + 1) % 4);
        case BACK:  return (Heading)((h + 2) % 4);
    }
    return h;
}

static Heading directionBetween(const Cell& fromCell, const Cell& toCell) {
    if (toCell.x > fromCell.x) return EAST;
    if (toCell.x < fromCell.x) return WEST;
    if (toCell.y > fromCell.y) return NORTH;
    return SOUTH;
}

TurnDecision decisionForHeading(Heading current, Heading desired) {
    switch (((int8_t)desired - (int8_t)current + 4) % 4) {
        case 0:  return GO_FORWARD;
        case 1:  return TURN_RIGHT;
        case 2:  return TURN_BACK;
        default: return TURN_LEFT;
    }
}

TurnDecision DFS(Maze& maze, uint16_t current, Heading heading)
{
    if (dfsDone || goalFound) return NO_MOVE;

    Cell &cell = maze.getCell(current);
    cell.visited = true;

    if (peekCell() != (int16_t)current)
        pushCell(current);

    Heading order[4] =
    {
        heading,
        rotate(LEFT, heading),
        rotate(RIGHT, heading),
        rotate(BACK, heading)
    };

    TurnDecision action[4] =
    {
        GO_FORWARD,
        TURN_LEFT,
        TURN_RIGHT,
        TURN_BACK
    };

    for (uint8_t i = 0; i < 4; i++)
    {
        Heading dir = order[i];
        WallDir wall = (WallDir)(1 << dir);

        if (maze.isWall(current, wall)) continue;

        int16_t neighbour = maze.adjacentCell(current, dir);

        if (neighbour == -1)
        {
            return action[i];
        }

        if (!maze.getCell(neighbour).visited)
        {
            return action[i];
        }
    }

    popCell();

    int16_t parent = peekCell();

    if (parent == -1) {
        dfsDone = true;
        return NO_MOVE;
    }

    Heading h = directionBetween(maze.getCell(current), maze.getCell(parent));

    return decisionForHeading(heading, h);
}


static constexpr uint16_t BFS_UNVISITED = 0xFFFF;

static uint16_t bfsDistance[MAZE_MAX_CELLS];
static int16_t  bfsQueue[MAZE_MAX_CELLS];

static constexpr WallDir WALL_BITS[4] = {WALL_N, WALL_E, WALL_S, WALL_W};
static constexpr Heading COMPASS[4]   = {NORTH, EAST, SOUTH, WEST};

void floodFill(Maze& maze, uint16_t goalCell) {
    uint16_t cellCount = maze.cellCount();

    for (uint16_t i = 0; i < cellCount; i++) {
        bfsDistance[i] = BFS_UNVISITED;
    }

    uint16_t head = 0;
    uint16_t tail = 0;

    bfsDistance[goalCell] = 0;
    bfsQueue[tail++] = (int16_t)goalCell;

    while (head < tail) {
        int16_t cellIndex = bfsQueue[head++];
        uint16_t currentDist = bfsDistance[cellIndex];

        for (uint8_t dir = 0; dir < 4; dir++) {
            if (maze.isWall((uint16_t)cellIndex, WALL_BITS[dir])) continue;

            int16_t neighbor = maze.adjacentCell(cellIndex, COMPASS[dir]);
            if (neighbor == -1) continue;

            if (bfsDistance[neighbor] != BFS_UNVISITED) continue;

            bfsDistance[neighbor] = currentDist + 1;
            bfsQueue[tail++] = neighbor;
        }
    }
}

static constexpr uint16_t MAX_PATH_LENGTH = MAZE_MAX_CELLS;

static TurnDecision speedrunPath[MAX_PATH_LENGTH];
static uint16_t speedrunPathLength = 0;
static uint16_t speedrunCursor = 0;
static bool speedrunPathReady = false;

bool buildSpeedrunPath(Maze& maze, uint16_t startCell, uint16_t goalCell) {
    floodFill(maze, goalCell);

    if (bfsDistance[startCell] == BFS_UNVISITED) {
        debugPrint("Flood fill: start not connected to goal in current map");
        return false;
    }

    speedrunPathLength = 0;

    Heading currentHeading = NORTH;
    int16_t cellIndex = (int16_t)startCell;
    uint16_t remainingDist = bfsDistance[startCell];

    while (cellIndex != (int16_t)goalCell && remainingDist > 0) {
        int16_t nextCell = -1;
        Heading nextHeading = NORTH;

        for (uint8_t dir = 0; dir < 4; dir++) {
            if (maze.isWall((uint16_t)cellIndex, WALL_BITS[dir])) continue;

            int16_t neighbor = maze.adjacentCell(cellIndex, COMPASS[dir]);
            if (neighbor == -1) continue;

            if (bfsDistance[neighbor] == remainingDist - 1) {
                nextCell = neighbor;
                nextHeading = COMPASS[dir];
                break;
            }
        }

        if (nextCell == -1) {
            debugPrint("Flood fill: path reconstruction failed (unexpected)");
            return false;
        }

        if (speedrunPathLength >= MAX_PATH_LENGTH) {
            debugPrint("Flood fill: path exceeds storage capacity");
            return false;
        }

        speedrunPath[speedrunPathLength++] = decisionForHeading(currentHeading, nextHeading);

        currentHeading = nextHeading;
        cellIndex = nextCell;
        remainingDist--;
    }

    speedrunCursor = 0;
    speedrunPathReady = true;

    savePath();

    debugPrintf("Speedrun path built: %u steps", speedrunPathLength);
    return true;
}

TurnDecision speedrunNext() {
    if (!speedrunPathReady || speedrunCursor >= speedrunPathLength) {
        return NO_MOVE;
    }

    return speedrunPath[speedrunCursor++];
}

bool speedrunFinished() {
    return !speedrunPathReady || speedrunCursor >= speedrunPathLength;
}

void resetSpeedrunProgress() {
    speedrunCursor = 0;
}

static TurnDecision returnPath[MAX_PATH_LENGTH];
static uint16_t returnPathLength = 0;
static uint16_t returnCursor = 0;
static bool returnPathReady = false;

void buildReturnPath() {
    returnPathLength = 0;
    returnPathReady = false;

    if (!speedrunPathReady || speedrunPathLength == 0) {
        debugPrint("Return path: no speedrun path available");
        return;
    }

    static Heading forwardHeadingAtStep[MAX_PATH_LENGTH];

    Heading h = NORTH;
    for (uint16_t i = 0; i < speedrunPathLength; i++) {
        switch (speedrunPath[i]) {
            case TURN_LEFT:  h = rotate(LEFT,  h); break;
            case TURN_RIGHT: h = rotate(RIGHT, h); break;
            case TURN_BACK:  h = rotate(BACK,  h); break;
            default: break;
        }
        forwardHeadingAtStep[i] = h;
    }

    uint16_t n = speedrunPathLength;
    Heading currentHeading = forwardHeadingAtStep[n - 1];

    for (uint16_t k = 0; k < n; k++) {
        Heading forwardStepHeading = forwardHeadingAtStep[n - 1 - k];
        Heading desired = rotate(BACK, forwardStepHeading);

        returnPath[returnPathLength++] = decisionForHeading(currentHeading, desired);
        currentHeading = desired;
    }

    returnCursor = 0;
    returnPathReady = true;

    debugPrintf("Return path built: %u steps", returnPathLength);
}

TurnDecision returnPathNext() {
    if (!returnPathReady || returnCursor >= returnPathLength) {
        return NO_MOVE;
    }
    return returnPath[returnCursor++];
}

bool returnPathFinished() {
    return !returnPathReady || returnCursor >= returnPathLength;
}

void resetExploration() {
    dfsStackSize = 0;
    dfsDone = false;
    goalFound = false;

    speedrunPathLength = 0;
    speedrunCursor = 0;
    speedrunPathReady = false;

    returnPathLength = 0;
    returnCursor = 0;
    returnPathReady = false;
}

static Preferences prefs;
static constexpr const char* PREF_NAMESPACE = "gcm";
static constexpr const char* PREF_KEY_LEN   = "pathLen";
static constexpr const char* PREF_KEY_DATA  = "pathData";

void savePath() {
    if (speedrunPathLength == 0) return;

    prefs.begin(PREF_NAMESPACE, false);
    prefs.putUShort(PREF_KEY_LEN, speedrunPathLength);
    prefs.putBytes(PREF_KEY_DATA, speedrunPath, speedrunPathLength * sizeof(TurnDecision));
    prefs.end();

    debugPrintf("Speedrun path saved to flash (%u steps)", speedrunPathLength);
}

bool loadPath() {
    prefs.begin(PREF_NAMESPACE, true);

    uint16_t storedLength = prefs.getUShort(PREF_KEY_LEN, 0);

    if (storedLength == 0 || storedLength > MAX_PATH_LENGTH) {
        prefs.end();
        return false;
    }

    size_t readBytes = prefs.getBytes(PREF_KEY_DATA, speedrunPath, storedLength * sizeof(TurnDecision));
    prefs.end();

    if (readBytes != storedLength * sizeof(TurnDecision)) {
        return false;
    }

    speedrunPathLength = storedLength;
    speedrunCursor = 0;
    speedrunPathReady = true;

    debugPrintf("Speedrun path loaded from flash (%u steps)", speedrunPathLength);
    return true;
}

void clearPath() {
    prefs.begin(PREF_NAMESPACE, false);
    prefs.clear();
    prefs.end();

    speedrunPathLength = 0;
    speedrunCursor = 0;
    speedrunPathReady = false;

    debugPrint("Speedrun path erased from flash");
}