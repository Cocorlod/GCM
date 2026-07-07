#include "FSM.hpp"

uint16_t pathLength = 0;

TurnDecision speedrunPath[MAZE_MAX_CELLS];

Maze maze;

bool pathSaved = false;

bool goalFound = false;

static constexpr uint8_t CELL_BOUNDARY_STABLE_SAMPLES = 4;
static constexpr unsigned long CELL_BOUNDARY_MIN_INTERVAL_MS = 150;

bool isGoalDetected() {
    static uint8_t consecutiveHits = 0;
    static constexpr uint8_t REQUIRED_CONSECUTIVE = 4;

    if(analogRead(IR_PIN) >= IR_THRESHOLD) {
        if(consecutiveHits < 255) consecutiveHits++;
    } else {
        consecutiveHits = 0;
    }

    return consecutiveHits >= REQUIRED_CONSECUTIVE;
}

Heading rotateLeft(Heading h) {
    return (Heading)((h + 3) % 4);
}

Heading rotateRight(Heading h) {
    return (Heading)((h + 1) % 4);
}

Heading rotateBack(Heading h) {
    return (Heading)((h + 2) % 4);
}

WallDir headingToWall(Heading h) {
    switch(h) {
        case NORTH: return WALL_N;
        case EAST:  return WALL_E;
        case SOUTH: return WALL_S;
        case WEST:  return WALL_W;
    }

    return WALL_N;
}

void CellBoundaryDetector::reset() {
    stableCount = 0;
    lastEventTime = millis();
}

bool CellBoundaryDetector::check(bool rawCondition) {
    unsigned long now = millis();

    if(now - lastEventTime < CELL_BOUNDARY_MIN_INTERVAL_MS) {
        stableCount = 0;
        return false;
    }

    if(rawCondition) {
        if(stableCount < 255) stableCount++;
    } else {
        stableCount = 0;
    }

    if(stableCount >= CELL_BOUNDARY_STABLE_SAMPLES) {
        stableCount = 0;
        lastEventTime = now;
        return true;
    }

    return false;
}

static bool canMove(Maze& maze, uint16_t current, Heading h) {
    return !maze.isWall(current, headingToWall(h));
}

static bool isUnvisited(Maze& maze, uint16_t current, Heading h) {
    if(!canMove(maze, current, h)) return false;

    int16_t next = maze.adjacentCell(current, h);

    if(next == -1) return true;

    return !maze.getCell(next).visited;
}

static TurnDecision turnToFace(Heading current, Heading target) {
    if(target == current) return GO_FORWARD;
    if(target == rotateLeft(current)) return TURN_LEFT;
    if(target == rotateRight(current)) return TURN_RIGHT;
    return TURN_BACK;
}

struct PathNode {
    uint16_t cell;
    Heading enteredHeading;
};

static PathNode pathStack[MAZE_MAX_CELLS];
static int16_t pathTop = -1;

void resetDFS() {
    pathTop = -1;
}

TurnDecision chooseDFS(Maze& maze, uint16_t current, Heading heading) {
    if(isGoalDetected()) {
        maze.setGoal(current);
        goalFound = true;
        debugPrint("GOAL DETECTED");
        return NO_MOVE;
    }

    maze.getCell(current).visited = true;

    if(pathTop < 0 || pathStack[pathTop].cell != current) {
        if(pathTop < (int16_t)MAZE_MAX_CELLS - 1) {
            pathStack[++pathTop] = PathNode{current, heading};
        }
    }

    Heading options[4] = {
        heading,
        rotateLeft(heading),
        rotateRight(heading),
        rotateBack(heading)
    };

    TurnDecision actions[4] = {
        GO_FORWARD,
        TURN_LEFT,
        TURN_RIGHT,
        TURN_BACK
    };

    for(int i = 0; i < 4; i++) {
        if(isUnvisited(maze, current, options[i])) {
            return actions[i];
        }
    }

    if(pathTop > 0) {
        Heading enteredHeading = pathStack[pathTop].enteredHeading;
        pathTop--;

        Heading target = rotateBack(enteredHeading);
        return turnToFace(heading, target);
    }

    return NO_MOVE;
}

static void resetFlood(Maze& maze) {
    for(uint16_t i = 0; i < maze.cellCount(); i++) {
        flood[i] = 65535;
    }
}

static int findGoal(Maze& maze) {
    for(uint16_t i = 0; i < maze.cellCount(); i++) {
        if(maze.isGoal(i)) return i;
    }

    return -1;
}

void computeFloodFill(Maze& maze) {
    resetFlood(maze);

    int goal = findGoal(maze);

    if(goal == -1) return;

    static uint16_t queue[MAZE_MAX_CELLS];

    int head = 0;
    int tail = 0;

    if(tail < MAZE_MAX_CELLS) queue[tail++] = goal;

    flood[goal] = 0;

    while(head < tail) {

        uint16_t current = queue[head++];

        Heading dirs[4] = {
            NORTH,
            EAST,
            SOUTH,
            WEST
        };

        for(int i = 0; i < 4; i++) {
            Heading h = dirs[i];

            if(!canMove(maze, current, h)) {
                continue;
            }

            int16_t next = maze.adjacentCell(current, h);

            if(next == -1) {
                continue;
            }

            if(flood[next] > flood[current] + 1) {
                flood[next] = flood[current] + 1;
                if(tail < (int)MAZE_MAX_CELLS) {
                    queue[tail++] = next;
                }
            }
        }
    }
}

TurnDecision chooseFloodFill(Maze& maze, uint16_t current, Heading heading) {
    Heading options[4] = {
        heading,
        rotateLeft(heading),
        rotateRight(heading),
        rotateBack(heading)
    };

    TurnDecision actions[4] = {
        GO_FORWARD,
        TURN_LEFT,
        TURN_RIGHT,
        TURN_BACK
    };

    uint16_t best = 65535;

    TurnDecision chosen = NO_MOVE;

    for(int i = 0; i < 4; i++) {
        if(!canMove(maze, current, options[i])) continue;
        
        int16_t next = maze.adjacentCell(current, options[i]);

        if(next == -1) continue;

        if(flood[next] < best) {
            best = flood[next];

            chosen = actions[i];
        }
    }

    return chosen;
}

void generateSpeedrunPath(Maze& maze) {
    pathLength = 0;

    int16_t current = 0;    

    Heading heading = NORTH;

    while(!maze.isGoal(current)) {
        TurnDecision move = chooseFloodFill(maze, current, heading);

        if(move == NO_MOVE) {
            break;
        }

        if(pathLength < MAZE_MAX_CELLS) speedrunPath[pathLength++] = move;

        switch(move) {
            case TURN_LEFT:
                heading = rotateLeft(heading);
                break;

            case TURN_RIGHT:
                heading = rotateRight(heading);
                break;

            case TURN_BACK:
                heading = rotateBack(heading);
                break;

            default:
                break;
        }

        current = maze.adjacentCell(current, heading);
        if(current == -1) break; // defensive: shouldn't happen on a consistent maze
    }
}

static TurnDecision mirrorTurn(TurnDecision d) {
    switch(d) {
        case TURN_LEFT:  return TURN_RIGHT;
        case TURN_RIGHT: return TURN_LEFT;
        default:         return d;
    }
}

enum class ReturnPhase : uint8_t {
    INITIAL_TURN,
    TRAVERSE_KICKOFF,
    TRAVERSE_DRIVE,
    FINAL_TURN,
    DONE
};

static ReturnPhase returnPhase = ReturnPhase::INITIAL_TURN;
static int16_t returnIndex = -1;

void resetReturnToStart() {
    returnPhase = ReturnPhase::INITIAL_TURN;
    returnIndex = -1;
}

bool returnToStartStep(Heading& heading, CellBoundaryDetector& boundary) {
    switch(returnPhase) {
        case ReturnPhase::INITIAL_TURN: {
            stopMotors();
            turnBack();
            delay(2 * TURN_DELAY);
            stopMotors();
            heading = rotateBack(heading);

            returnIndex = (int16_t)pathLength - 1;
            boundary.reset();
            returnPhase = (returnIndex >= 0) ? ReturnPhase::TRAVERSE_KICKOFF : ReturnPhase::FINAL_TURN;
            return false;
        }

        case ReturnPhase::TRAVERSE_KICKOFF: {
            executeMove(mirrorTurn(speedrunPath[returnIndex]), heading);
            boundary.reset();
            returnPhase = ReturnPhase::TRAVERSE_DRIVE;
            return false;
        }

        case ReturnPhase::TRAVERSE_DRIVE: {
            moveForward(tof);

            bool front = tof.isThereWall(FRONT);
            bool left = tof.isThereWall(LEFT);
            bool right = tof.isThereWall(RIGHT);
            bool centered = tof.isCentered();
            bool rawDecision = centered && (front || (!left || !right));

            if(boundary.check(rawDecision)) {
                returnIndex--;
                returnPhase = (returnIndex >= 0) ? ReturnPhase::TRAVERSE_KICKOFF : ReturnPhase::FINAL_TURN;
            }

            return false;
        }

        case ReturnPhase::FINAL_TURN: {
            stopMotors();
            turnBack();
            delay(2 * TURN_DELAY);
            stopMotors();
            heading = rotateBack(heading);
            returnPhase = ReturnPhase::DONE;
            return false;
        }

        case ReturnPhase::DONE: {
            returnPhase = ReturnPhase::INITIAL_TURN;
            return true;
        }
    }

    return true;
}

void savePath() {
    prefs.begin("micromouse", false);

    prefs.putUInt("length", pathLength);

    prefs.putBytes("path", speedrunPath, pathLength * sizeof(TurnDecision)); 

    prefs.end();
}

void loadPath() {
    prefs.begin("micromouse", true);

    uint32_t storedLength = prefs.getUInt("length", 0);

    if(storedLength > MAZE_MAX_CELLS) {
        storedLength = MAZE_MAX_CELLS;
    }

    size_t bytesRead = prefs.getBytes("path", speedrunPath, storedLength * sizeof(TurnDecision));

    prefs.end();

    pathLength = (uint16_t)(bytesRead / sizeof(TurnDecision));
    pathSaved = (pathLength > 0);
}

void clearPath() {
    prefs.begin("micromouse", false);
    prefs.clear();
    prefs.end();
    pathSaved = false;
    pathLength = 0;
    resetDFS();
}
