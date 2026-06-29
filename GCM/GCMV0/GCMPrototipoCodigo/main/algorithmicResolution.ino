#include "algorithmicResolution.hpp"

bool isGoalDetected() {
    return digitalRead(IR_PIN) == LOW;
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

static DFSNode stackDFS[MAX_STACK];
static int sp = -1;

static void push(uint16_t cell, Heading h) {

    if(sp >= MAX_STACK - 1) return;

    stackDFS[++sp] = {cell, h};
}

static void pop() {
    if(sp >= 0) sp--;
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

TurnDecision chooseDFS(Maze& maze, uint16_t current, Heading heading) {
    if(isGoalDetected()) {
        maze.setGoal(current);

        return NO_MOVE;
    }

    maze.getCell(current).visited = true;

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

    int chosen = -1;

    for(int i = 0; i < 4; i++) {
        if(isUnvisited(maze, current, options[i])) {
            chosen = i;
            break;
        }
    }

    if(chosen != -1) {
        int branches = 0;

        for(int i = 0; i < 4; i++) {
            if(isUnvisited(maze, current, options[i])) {
                branches++;
            }
        }

        if(branches > 1) {
            push(current, heading);
        }

        return actions[chosen];
    }

    if(sp >= 0) {
        DFSNode node = stackDFS[sp];
        pop();

        return TURN_BACK;
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
        if(maze.isGoal(i)) {
            return i;
        }
    }

    return -1;
}

void computeFloodFill(Maze& maze) {
    resetFlood(maze);

    int goal = findGoal(maze);

    if(goal == -1) return;

    uint16_t queue[MAX_CELLS];

    int head = 0;
    int tail = 0;

    queue[tail++] = goal;

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
                queue[tail++] = next;
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
        if(!canMove(maze, current, options[i])) {
            continue;
        }

        int16_t next = maze.adjacentCell(current, options[i]);

        if(next == -1) {
            continue;
        }

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

        speedrunPath[pathLength++] = move;

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

        current = maze.adjacentCell(current,heading);
    }
}

void returnToStart(Heading& heading) {
    stopMotors();

    delay(1000);

    turnBack();
    delay(2 * TURN_DELAY);

    stopMotors();

    heading = rotateBack(heading);

    for(int i = pathLength - 1; i >= 0; i--) {

        switch(speedrunPath[i]) {

            case GO_FORWARD:
                executeMove(GO_FORWARD, heading);
                break;

            case TURN_LEFT:
                executeMove(TURN_RIGHT, heading);
                break;

            case TURN_RIGHT:
                executeMove(TURN_LEFT, heading);
                break;

            case TURN_BACK:
                executeMove(TURN_BACK, heading);
                break;

            default:
                break;
        }
    }

    stopMotors();

    turnBack();
    delay(2 * TURN_DELAY);

    stopMotors();

    heading = rotateBack(heading);
}