// ═══════════════════════════════════════════════════════════════════════════
//  GDW (Giomi Drunk Walk) Maze Planner  
//  C++17  |  Single translation unit
// ═══════════════════════════════════════════════════════════════════════════

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <array>
#include <algorithm>
#include <limits>
#include <numeric>
#include <cassert>

// ───────────────────────────────────────────────────────────────────────────
//  Constants
// ───────────────────────────────────────────────────────────────────────────

constexpr int   MAZE_SIZE = 16;
constexpr int   MAX_FLOOD = MAZE_SIZE * MAZE_SIZE;
constexpr float INF_F     = 1e9f;
constexpr float SQRT2F    = 1.41421356f;

// Standard IEEE/APEC micromouse goal: 2×2 centre cells
constexpr std::array<std::pair<int,int>, 4> GOAL_CELLS = {{
    {7,7}, {8,7}, {7,8}, {8,8}
}};

// ───────────────────────────────────────────────────────────────────────────
//  Direction system
//
//  Walls use 4 cardinal indices  (WN WE WS WW).
//  Movement uses 8 indices       (DN DNE DE DSE DS DSW DW DNW).
//  Diagonal moves require TWO cardinal walls to both be open.
// ───────────────────────────────────────────────────────────────────────────

enum WallDir : int { WN=0, WE=1, WS=2, WW=3 };

// 8-direction movement enum
enum Dir8 : int { DN=0, DNE=1, DE=2, DSE=3, DS=4, DSW=5, DW=6, DNW=7 };

static constexpr int   D8X[8]    = {  0, 1, 1, 1, 0,-1,-1,-1 };
static constexpr int   D8Y[8]    = { -1,-1, 0, 1, 1, 1, 0,-1 };
static constexpr int   D8OPP[8]  = { DS, DSW, DW, DNW, DN, DNE, DE, DSE };
static constexpr float D8COST[8] = {
    1.0f, SQRT2F, 1.0f, SQRT2F, 1.0f, SQRT2F, 1.0f, SQRT2F
};
static constexpr bool  D8DIAG[8] = { false,true,false,true,false,true,false,true };

// Each direction must clear these wall slots (-1 = not applicable)
//   N  → WN          NE → WN+WE        E  → WE
//   SE → WE+WS       S  → WS           SW → WS+WW
//   W  → WW          NW → WN+WW
static constexpr int D8W[8][2] = {
    {WN,-1}, {WN,WE}, {WE,-1}, {WE,WS},
    {WS,-1}, {WS,WW}, {WW,-1}, {WN,WW}
};

static constexpr int WALL_OPP[4] = { WS, WW, WN, WE };
static constexpr int WALL_DX[4]  = {  0,  1,  0, -1 };
static constexpr int WALL_DY[4]  = { -1,  0,  1,  0 };

// ───────────────────────────────────────────────────────────────────────────
//  Cell
// ───────────────────────────────────────────────────────────────────────────

struct Cell {
    // Unknown walls default to "present" (pessimistic physical model).
    // wallKnown tracks whether the sensor has confirmed each wall.
    bool wall[4]      = { true, true, true, true  };
    bool wallKnown[4] = { false,false,false,false  };

    bool explored   = false;
    bool isJunction = false;  // 3+ exits
    bool isDeadEnd  = false;  // 1 exit

    float floodDist = (float)MAX_FLOOD;
};

// ───────────────────────────────────────────────────────────────────────────
//  Maze
// ───────────────────────────────────────────────────────────────────────────

class Maze {
public:
    std::array<std::array<Cell, MAZE_SIZE>, MAZE_SIZE> grid{};

    bool  valid(int x, int y) const {
        return (unsigned)x < (unsigned)MAZE_SIZE &&
               (unsigned)y < (unsigned)MAZE_SIZE;
    }
    Cell&       at(int x, int y)       { return grid[y][x]; }
    const Cell& at(int x, int y) const { return grid[y][x]; }

    // Set a wall and mirror it on the neighbour.
    void setWall(int x, int y, int wdir, bool present) {
        at(x,y).wall[wdir]      = present;
        at(x,y).wallKnown[wdir] = true;
        int nx = x + WALL_DX[wdir], ny = y + WALL_DY[wdir];
        if (valid(nx,ny)) {
            at(nx,ny).wall[WALL_OPP[wdir]]      = present;
            at(nx,ny).wallKnown[WALL_OPP[wdir]] = true;
        }
    }

    void initBorderWalls() {
        for (int i = 0; i < MAZE_SIZE; i++) {
            setWall(i,           0,           WN, true);
            setWall(i, MAZE_SIZE-1,           WS, true);
            setWall(0,           i,           WW, true);
            setWall(MAZE_SIZE-1, i,           WE, true);
        }
    }

    // Pessimistic: wall=true blocks movement (includes unknown=walled default).
    bool canMove(int x, int y, int d8) const {
        for (int k = 0; k < 2; k++) {
            int w = D8W[d8][k];
            if (w != -1 && at(x,y).wall[w]) return false;
        }
        return valid(x + D8X[d8], y + D8Y[d8]);
    }

    // Optimistic: only confirmed walls block movement.
    bool canMoveOpt(int x, int y, int d8) const {
        for (int k = 0; k < 2; k++) {
            int w = D8W[d8][k];
            if (w != -1 && at(x,y).wallKnown[w] && at(x,y).wall[w]) return false;
        }
        return valid(x + D8X[d8], y + D8Y[d8]);
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Cell Classifier
// ───────────────────────────────────────────────────────────────────────────

class CellClassifier {
public:
    static void analyze(Maze& maze) {
        for (int y = 0; y < MAZE_SIZE; y++)
            for (int x = 0; x < MAZE_SIZE; x++) {
                auto& c = maze.at(x,y);
                int exits = 0;
                for (int w = 0; w < 4; w++) if (!c.wall[w]) exits++;
                c.isJunction = exits >= 3;
                c.isDeadEnd  = exits == 1;
            }
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Flood Fill  (8-directional, multi-goal, Dijkstra-weighted)
//
//  Uses a priority queue instead of a BFS queue so diagonal costs (√2)
//  are handled correctly.  Seeds all four goal cells simultaneously.
// ───────────────────────────────────────────────────────────────────────────

class FloodFill {
public:
    // optimistic=true → treat unknown walls as open (exploration phase).
    // optimistic=false → treat unknown walls as closed  (speed run).
    static void solve(Maze& maze, bool optimistic = false) {
        for (int y = 0; y < MAZE_SIZE; y++)
            for (int x = 0; x < MAZE_SIZE; x++)
                maze.at(x,y).floodDist = INF_F;

        using Entry = std::pair<float, std::pair<int,int>>;
        std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;

        for (auto& [gx,gy] : GOAL_CELLS) {
            if (!maze.valid(gx,gy)) continue;
            maze.at(gx,gy).floodDist = 0.0f;
            pq.push({0.0f, {gx,gy}});
        }

        while (!pq.empty()) {
            auto [dist, pos] = pq.top(); pq.pop();
            auto [x,y] = pos;
            if (dist > maze.at(x,y).floodDist) continue; // stale

            for (int d8 = 0; d8 < 8; d8++) {
                bool ok = optimistic ? maze.canMoveOpt(x,y,d8)
                                     : maze.canMove   (x,y,d8);
                if (!ok) continue;
                int nx = x + D8X[d8], ny = y + D8Y[d8];
                float nd = dist + D8COST[d8];
                if (nd < maze.at(nx,ny).floodDist) {
                    maze.at(nx,ny).floodDist = nd;
                    pq.push({nd, {nx,ny}});
                }
            }
        }
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Robot Model
// ───────────────────────────────────────────────────────────────────────────

struct RobotModel {
    float maxAccel        =  9.0f;  // m/s²
    float maxBraking      = 10.0f;  // m/s²
    float maxLatAccel     =  8.0f;  // m/s²
    float maxVelocity     =  5.0f;  // m/s  (speed run)
    float exploreVelocity =  1.0f;  // m/s  (scout run)
    float wheelbase       =  0.07f; // m
    float mass            =  0.09f; // kg
    float cellSize        =  0.18f; // m per cell
};

// ───────────────────────────────────────────────────────────────────────────
//  A* Planner  (8-directional, turn-cost aware, multi-goal)
//
//  Turn cost penalises direction changes so the planner prefers long
//  straights and smooth diagonals over zigzag cardinal paths.
// ───────────────────────────────────────────────────────────────────────────

class AStarPlanner {
public:
    using Coord = std::pair<int,int>;

    struct Node {
        int   x, y, prevDir;
        float g, f;
        bool operator>(const Node& o) const { return f > o.f; }
    };

    // Turn cost proportional to angular change (in 45° increments).
    static float turnCost(int prev, int next) {
        if (prev < 0) return 0.0f;
        int diff = std::abs(next - prev);
        if (diff > 4) diff = 8 - diff;
        return diff * 0.18f; // tunable: higher → smoother path
    }

    // Heuristic: flood distance is admissible (computed without turn costs)
    // and already encodes the multi-goal minimum.
    static float heuristic(int x, int y, const Maze& maze) {
        return maze.at(x,y).floodDist;
    }

    // optimistic=true for exploration (treat unknowns as open).
    static std::vector<Coord> findPath(
        const Maze& maze, int sx, int sy,
        bool optimistic = false
    ) {
        std::array<std::array<float, MAZE_SIZE>, MAZE_SIZE> gCost{};
        std::array<std::array<Coord, MAZE_SIZE>, MAZE_SIZE> parent{};
        std::array<std::array<int,   MAZE_SIZE>, MAZE_SIZE> fromDir{};

        for (auto& row : gCost)   row.fill(INF_F);
        for (auto& row : parent)  row.fill({-1,-1});
        for (auto& row : fromDir) row.fill(-1);

        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
        gCost[sy][sx] = 0.0f;
        open.push({sx, sy, -1, 0.0f, heuristic(sx, sy, maze)});

        int goalX = -1, goalY = -1;

        while (!open.empty()) {
            auto top = open.top(); open.pop();
            int x = top.x, y = top.y, fd = top.prevDir;
            float g = top.g;

            // Goal check
            bool atGoal = false;
            for (auto& [gx,gy] : GOAL_CELLS)
                if (x == gx && y == gy) { goalX=gx; goalY=gy; atGoal=true; break; }
            if (atGoal) break;

            if (g > gCost[y][x]) continue; // stale entry

            for (int d8 = 0; d8 < 8; d8++) {
                bool ok = optimistic ? maze.canMoveOpt(x,y,d8)
                                     : maze.canMove   (x,y,d8);
                if (!ok) continue;
                int nx = x + D8X[d8], ny = y + D8Y[d8];
                float ng = g + D8COST[d8] + turnCost(fd, d8);
                if (ng < gCost[ny][nx]) {
                    gCost[ny][nx]  = ng;
                    parent[ny][nx] = {x,y};
                    fromDir[ny][nx]= d8;
                    open.push({nx, ny, d8, ng, ng + heuristic(nx,ny,maze)});
                }
            }
        }

        if (goalX < 0) {
            std::cerr << "A*: no path from (" << sx << "," << sy << ")\n";
            return {};
        }

        std::vector<Coord> path;
        int cx = goalX, cy = goalY;
        while (cx != -1) {
            path.push_back({cx,cy});
            auto [px,py] = parent[cy][cx];
            cx = px; cy = py;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Trajectory point
// ───────────────────────────────────────────────────────────────────────────

struct TrajectoryPoint {
    float x, y;        // world position  (m)
    float heading;     // radians
    float curvature;   // κ (1/m)
    float velocity;    // m/s
    float arcLen;      // cumulative arc length from start (m)
};

// ───────────────────────────────────────────────────────────────────────────
//  Catmull-Rom Spline Trajectory Generator
//
//  Replaces the degenerate Bézier from v1.
//  C¹ continuity at every waypoint → no velocity spikes at cell boundaries.
//  Phantom endpoints mirror the first/last segment for smooth entry/exit.
// ───────────────────────────────────────────────────────────────────────────

class CatmullRom {
    using Pt = std::pair<float,float>;

    static Pt pos(const Pt& p0, const Pt& p1,
                  const Pt& p2, const Pt& p3, float t) {
        float t2=t*t, t3=t2*t;
        auto eval = [&](float a,float b,float c,float d) {
            return 0.5f*( 2*b + (-a+c)*t
                        + (2*a-5*b+4*c-d)*t2
                        + (-a+3*b-3*c+d)*t3 );
        };
        return { eval(p0.first,p1.first,p2.first,p3.first),
                 eval(p0.second,p1.second,p2.second,p3.second) };
    }

    static Pt tang(const Pt& p0, const Pt& p1,
                   const Pt& p2, const Pt& p3, float t) {
        float t2=t*t;
        auto eval = [&](float a,float b,float c,float d) {
            return 0.5f*( (-a+c)
                        + 2*(2*a-5*b+4*c-d)*t
                        + 3*(-a+3*b-3*c+d)*t2 );
        };
        return { eval(p0.first,p1.first,p2.first,p3.first),
                 eval(p0.second,p1.second,p2.second,p3.second) };
    }

    static Pt acc(const Pt& p0, const Pt& p1,
                  const Pt& p2, const Pt& p3, float t) {
        auto eval = [&](float a,float b,float c,float d) {
            return 0.5f*( 2*(2*a-5*b+4*c-d)
                        + 6*(-a+3*b-3*c+d)*t );
        };
        return { eval(p0.first,p1.first,p2.first,p3.first),
                 eval(p0.second,p1.second,p2.second,p3.second) };
    }

public:
    static std::vector<TrajectoryPoint> generate(
        const std::vector<std::pair<float,float>>& waypoints,
        float maxVelocity,
        int   samplesPerSeg = 60    // more samples → smoother profile
    ) {
        if (waypoints.size() < 2) return {};

        // Build extended point list with phantom endpoints
        std::vector<Pt> pts;
        {
            auto& f = waypoints.front();
            auto& s = waypoints[1];
            pts.push_back({2*f.first-s.first, 2*f.second-s.second});
        }
        for (auto& p : waypoints) pts.push_back(p);
        {
            auto& l  = waypoints.back();
            auto& sl = waypoints[waypoints.size()-2];
            pts.push_back({2*l.first-sl.first, 2*l.second-sl.second});
        }

        std::vector<TrajectoryPoint> traj;
        float cumArc = 0.0f;

        // seg iterates over every consecutive pair of original waypoints
        for (size_t seg = 1; seg + 2 < pts.size(); seg++) {
            const Pt& p0 = pts[seg-1];
            const Pt& p1 = pts[seg];
            const Pt& p2 = pts[seg+1];
            const Pt& p3 = pts[seg+2];

            int start = (seg == 1) ? 0 : 1; // avoid duplicate boundary points
            for (int i = start; i <= samplesPerSeg; i++) {
                float t  = (float)i / samplesPerSeg;
                auto  p  = pos (p0,p1,p2,p3,t);
                auto  dp = tang(p0,p1,p2,p3,t);
                auto  ddp= acc (p0,p1,p2,p3,t);

                float speed = std::sqrt(dp.first*dp.first + dp.second*dp.second);
                float hdg   = std::atan2(dp.second, dp.first);
                float kappa = 0.0f;
                if (speed > 1e-6f)
                    kappa = (dp.first*ddp.second - dp.second*ddp.first)
                          / (speed*speed*speed);

                if (!traj.empty()) {
                    float dx = p.first  - traj.back().x;
                    float dy = p.second - traj.back().y;
                    cumArc += std::sqrt(dx*dx + dy*dy);
                }

                traj.push_back({p.first, p.second, hdg, kappa, maxVelocity, cumArc});
            }
        }
        return traj;
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Velocity Profile  (arc-length based, curvature-aware)
//
//  Uses actual arc-length increments (not a fixed DS constant) so time
//  estimates are accurate even on curved segments.
// ───────────────────────────────────────────────────────────────────────────

class VelocityProfile {
public:
    // Maximum speed for a given radius of curvature.
    static float vCurve(float kappa, float maxLatAccel) {
        if (std::abs(kappa) < 1e-6f) return INF_F;
        return std::sqrt(maxLatAccel / std::abs(kappa));
    }

    static void forwardPass(
        std::vector<TrajectoryPoint>& traj,
        float maxAccel, float maxLatAccel
    ) {
        traj.front().velocity = 0.0f;
        for (size_t i = 1; i < traj.size(); i++) {
            float ds    = traj[i].arcLen - traj[i-1].arcLen;
            float vAcc  = std::sqrt(traj[i-1].velocity*traj[i-1].velocity
                                    + 2.0f*maxAccel*ds);
            float vCurv = vCurve(traj[i].curvature, maxLatAccel);
            traj[i].velocity = std::min({traj[i].velocity, vAcc, vCurv});
        }
    }

    static void backwardPass(
        std::vector<TrajectoryPoint>& traj,
        float maxBraking, float maxLatAccel
    ) {
        traj.back().velocity = 0.0f;
        for (int i = (int)traj.size()-2; i >= 0; i--) {
            float ds    = traj[i+1].arcLen - traj[i].arcLen;
            float vBrk  = std::sqrt(traj[i+1].velocity*traj[i+1].velocity
                                    + 2.0f*maxBraking*ds);
            float vCurv = vCurve(traj[i].curvature, maxLatAccel);
            traj[i].velocity = std::min({traj[i].velocity, vBrk, vCurv});
        }
    }

    // Accurate time integration using per-segment average velocity.
    static float estimateTime(const std::vector<TrajectoryPoint>& traj) {
        float total = 0.0f;
        for (size_t i = 1; i < traj.size(); i++) {
            float ds   = traj[i].arcLen - traj[i-1].arcLen;
            float vAvg = 0.5f*(traj[i].velocity + traj[i-1].velocity);
            total += ds / std::max(vAvg, 1e-3f);
        }
        return total;
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Maze Explorer  (scout run – simulates wall sensing)
//
//  In hardware: senseAndUpdate() reads the IR sensors and updates the
//  robot's internal map.  Here we read from a ground-truth maze instead.
//
//  Strategy:
//    • Run flood-fill optimistically (unknown = open).
//    • Greedily follow the lowest flood-gradient neighbour.
//    • After each step, sense walls and re-flood if new walls found.
//    • Repeat until a goal cell is reached.
// ───────────────────────────────────────────────────────────────────────────

class MazeExplorer {
public:
    // Simulate reading sensors at (x,y) from the truth maze.
    static bool senseAndUpdate(Maze& bot, const Maze& truth, int x, int y) {
        bool newWall = false;
        for (int w = 0; w < 4; w++) {
            bool knownBefore = bot.at(x,y).wallKnown[w];
            bool realVal     = truth.at(x,y).wall[w];
            bot.setWall(x, y, w, realVal);
            if (!knownBefore && realVal) newWall = true; // discovered a wall
        }
        bot.at(x,y).explored = true;
        return newWall;
    }

    static std::vector<std::pair<int,int>> scoutRun(
        Maze& bot, const Maze& truth, int startX, int startY
    ) {
        int x = startX, y = startY;
        std::vector<std::pair<int,int>> visited = {{x,y}};

        senseAndUpdate(bot, truth, x, y);
        FloodFill::solve(bot, /*optimistic=*/true);

        for (int step = 0; step < MAX_FLOOD * 4; step++) {
            // Goal check
            for (auto& [gx,gy] : GOAL_CELLS)
                if (x == gx && y == gy) return visited;

            // Pick the open neighbour with the smallest flood distance.
            // During exploration we use cardinal moves only (8-dir sensing
            // is unreliable mid-diagonal).
            int   bestDir  = -1;
            float bestDist = INF_F;
            for (int d8 = 0; d8 < 8; d8 += 2) { // cardinal: 0,2,4,6
                if (!bot.canMove(x,y,d8)) continue;
                int nx = x + D8X[d8], ny = y + D8Y[d8];
                if (bot.at(nx,ny).floodDist < bestDist) {
                    bestDist = bot.at(nx,ny).floodDist;
                    bestDir  = d8;
                }
            }
            if (bestDir < 0) break; // isolated — should not happen

            x += D8X[bestDir];
            y += D8Y[bestDir];
            visited.push_back({x,y});

            bool newInfo = senseAndUpdate(bot, truth, x, y);
            if (newInfo) FloodFill::solve(bot, /*optimistic=*/true);
        }
        return visited;
    }

    // Navigate back to (tx,ty) using the known map (post-scout).
    static std::vector<std::pair<int,int>> returnTo(
        Maze& bot, int sx, int sy, int tx, int ty
    ) {
        // Temporarily re-seed flood fill from the target
        // We do a one-off A* from current to target using known walls.
        // Re-use AStarPlanner but swap goal — simplest: reverse flood fill.
        // Here we just return the A* path directly (caller prints it).
        return AStarPlanner::findPath(bot, sx, sy, /*optimistic=*/false);
        // Note: for return-to-start, caller should swap goal cells
        // to {0,0} by overriding GOAL_CELLS — or use a dedicated flood.
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Turn Violence metric
// ───────────────────────────────────────────────────────────────────────────

class TurnViolence {
public:
    static float compute(float kappa, float v) { return std::abs(kappa)*v*v; }
    static float peak(const std::vector<TrajectoryPoint>& traj) {
        float pk = 0.0f;
        for (auto& p : traj) pk = std::max(pk, compute(p.curvature, p.velocity));
        return pk;
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  GDW Planner  (top-level orchestrator)
// ───────────────────────────────────────────────────────────────────────────

class GDWPlanner {
public:
    Maze       botMaze;    // what the robot knows (grows during scout)
    Maze       truthMaze;  // ground truth  (hardware replaces this)
    RobotModel robot;

    // ── Build a representative test maze ─────────────────────────────────
    void buildTruthMaze() {
        truthMaze.initBorderWalls();

        // Internal walls — a non-trivial pattern to exercise the planner.
        // Replace with real maze data in competition firmware.
        for (auto& [x,y,w] : std::vector<std::tuple<int,int,int>>{
            {1,0,WS},{1,1,WS},{3,1,WE},{3,2,WS},{5,0,WS},{5,1,WE},
            {7,0,WS},{7,1,WE},{8,2,WW},{9,1,WS},{9,2,WE},{11,3,WN},
            {12,5,WW},{6,4,WS},{4,6,WE},{2,8,WS},{8,2,WE},{10,4,WN},
            {13,7,WW},{6,10,WS},{3,12,WE},{9,9,WN},{5,14,WE},{11,11,WS}
        }) {
            truthMaze.setWall(x,y,w,true);
        }
    }

    void initialize() {
        buildTruthMaze();
        botMaze.initBorderWalls(); // robot starts knowing only the border
    }

    // ── Generate and profile a trajectory from a cell path ───────────────
    void profileAndPrint(
        const std::vector<std::pair<int,int>>& path,
        float maxVelocity,
        const std::string& label
    ) {
        std::vector<std::pair<float,float>> waypoints;
        for (auto& [x,y] : path)
            waypoints.push_back({x * robot.cellSize, y * robot.cellSize});

        auto traj = CatmullRom::generate(waypoints, maxVelocity, 60);
        if (traj.empty()) { std::cerr << label << ": trajectory empty\n"; return; }

        VelocityProfile::forwardPass (traj, robot.maxAccel,   robot.maxLatAccel);
        VelocityProfile::backwardPass(traj, robot.maxBraking, robot.maxLatAccel);

        float time   = VelocityProfile::estimateTime(traj);
        float peakTV = TurnViolence::peak(traj);

        std::cout << "\n── " << label << " ──\n";
        std::cout << "  Path cells    : " << path.size()           << "\n";
        std::cout << "  Traj points   : " << traj.size()           << "\n";
        std::cout << "  Path length   : " << traj.back().arcLen    << " m\n";
        std::cout << "  Estimated time: " << time                  << " s\n";
        std::cout << "  Peak TV       : " << peakTV                << "\n";

        std::cout << "  Velocity profile (every 30th point):\n";
        for (size_t i = 0; i < traj.size(); i += 30) {
            const auto& p = traj[i];
            std::cout << "    [" << i << "]"
                      << "  arc=" << p.arcLen    << " m"
                      << "  v="   << p.velocity  << " m/s"
                      << "  κ="   << p.curvature << " 1/m\n";
        }
    }

    // ── Scout run ─────────────────────────────────────────────────────────
    void scoutRun() {
        std::cout << "╔══════════════════════════════╗\n"
                  << "║        SCOUT  RUN            ║\n"
                  << "╚══════════════════════════════╝\n";

        FloodFill::solve(botMaze, /*optimistic=*/true);
        auto visited = MazeExplorer::scoutRun(botMaze, truthMaze, 0, 0);

        std::cout << "  Cells visited during scout: " << visited.size() << "\n";

        // After reaching the goal, lock in what we know.
        FloodFill::solve(botMaze, /*optimistic=*/false);
        CellClassifier::analyze(botMaze);

        // Produce a trajectory for the scout path (exploration-speed profile)
        profileAndPrint(visited, robot.exploreVelocity, "Scout trajectory");
    }

    // ── Speed run ─────────────────────────────────────────────────────────
    void speedRun() {
        std::cout << "\n╔══════════════════════════════╗\n"
                  << "║        SPEED  RUN            ║\n"
                  << "╚══════════════════════════════╝\n";

        // Re-solve flood fill pessimistically (full known map).
        FloodFill::solve(botMaze, /*optimistic=*/false);

        auto path = AStarPlanner::findPath(botMaze, 0, 0, /*optimistic=*/false);
        if (path.empty()) { std::cerr << "Speed run: no path.\n"; return; }

        std::cout << "  Optimal path (" << path.size() << " cells):\n";
        for (auto& [x,y] : path)
            std::cout << "    (" << x << "," << y << ")\n";

        profileAndPrint(path, robot.maxVelocity, "Speed-run trajectory");
    }

    // ── Return to start (after reaching goal) ─────────────────────────────
    //  Competition rules typically require the robot to return to the
    //  start square before its second timed run.
    void returnToStart(int goalX, int goalY) {
        std::cout << "\n╔══════════════════════════════╗\n"
                  << "║      RETURN  TO  START       ║\n"
                  << "╚══════════════════════════════╝\n";

        // Temporarily re-seed flood fill from (0,0) instead of goal cells.
        // We clone the maze, override floodDist, run fill, find path.
        Maze tmp = botMaze;
        for (int y = 0; y < MAZE_SIZE; y++)
            for (int x = 0; x < MAZE_SIZE; x++)
                tmp.at(x,y).floodDist = INF_F;

        // Seed origin
        tmp.at(0,0).floodDist = 0.0f;
        using Entry = std::pair<float, std::pair<int,int>>;
        std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;
        pq.push({0.0f, {0,0}});
        while (!pq.empty()) {
            auto [d, pos] = pq.top(); pq.pop();
            auto [x,y] = pos;
            if (d > tmp.at(x,y).floodDist) continue;
            for (int d8 = 0; d8 < 8; d8++) {
                if (!tmp.canMove(x,y,d8)) continue;
                int nx = x+D8X[d8], ny = y+D8Y[d8];
                float nd = d + D8COST[d8];
                if (nd < tmp.at(nx,ny).floodDist) {
                    tmp.at(nx,ny).floodDist = nd;
                    pq.push({nd,{nx,ny}});
                }
            }
        }

        auto path = AStarPlanner::findPath(tmp, goalX, goalY, false);
        if (path.empty()) { std::cerr << "Return: no path.\n"; return; }

        std::cout << "  Return path (" << path.size() << " cells)\n";
        profileAndPrint(path, robot.maxVelocity * 0.6f, "Return trajectory");
    }

    void run() {
        scoutRun();

        // Find which goal cell was actually reached (first in list).
        int reachedX = GOAL_CELLS[0].first;
        int reachedY = GOAL_CELLS[0].second;

        returnToStart(reachedX, reachedY);
        speedRun();
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Entry point
// ───────────────────────────────────────────────────────────────────────────

int main() {
    GDWPlanner planner;
    planner.initialize();
    planner.run();
    return 0;
}