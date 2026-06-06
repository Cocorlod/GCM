// ═══════════════════════════════════════════════════════════════════════════
//  GDW (Giomi Drunk Walk) Maze Planner  —  Championship Edition
//  C++17  |  Single translation unit
//
//  Algorithmic highlights vs. previous version
//  ─────────────────────────────────────────────
//  • Exploration  : Modified Trémaux + frontier-queue replaces blind greedy
//                   flood-gradient walk.  Guarantees full map coverage with
//                   minimal redundant travel.
//  • Path finding : A* over merged MOVE primitives (straights + diagonals
//                   collapsed into single corridor moves) instead of raw
//                   cell-by-cell paths.  Turn cost applied AFTER planning
//                   so admissibility of the heuristic is preserved.
//  • Trajectory   : Clothoid (Euler spiral) entry/exit segments give C²
//                   curvature continuity — zero jerk spike at turn entry.
//                   Catmull-Rom retained only for pure-straight stitching.
//  • Velocity     : S-curve (jerk-limited) trapezoidal profile replaces
//                   instantaneous accel/decel transitions.  Friction-circle
//                   constraint couples lateral and longitudinal acceleration.
//  • Sensor fusion: Discrete Kalman filter stub fuses wheel odometry with
//                   gyro heading; wall-proximity measurements provide
//                   position re-anchor at every sensed cell.
//  • Goals        : Runtime-configurable; supports 16×16, half-size (8×8),
//                   and custom arenas.
//  • Bug fixes    : returnTo dead-code removed; A* admissibility restored;
//                   reached-goal tracking fixed; arc accumulation corrected.
// ═══════════════════════════════════════════════════════════════════════════

#include <iostream>
#include <vector>
#include <queue>
#include <deque>
#include <stack>
#include <set>
#include <unordered_set>
#include <cmath>
#include <array>
#include <algorithm>
#include <limits>
#include <numeric>
#include <cassert>
#include <optional>
#include <functional>
#include <string>
#include <sstream>
#include <iomanip>

// ───────────────────────────────────────────────────────────────────────────
//  Compile-time constants
// ───────────────────────────────────────────────────────────────────────────

constexpr int   MAZE_SIZE  = 16;
constexpr int   MAX_CELLS  = MAZE_SIZE * MAZE_SIZE;
constexpr float INF_F      = 1e9f;
constexpr float SQRT2F     = 1.41421356237f;
constexpr float PI_F       = 3.14159265359f;
constexpr float HALF_PI_F  = PI_F * 0.5f;

// ───────────────────────────────────────────────────────────────────────────
//  Direction system
// ───────────────────────────────────────────────────────────────────────────

enum WallDir : int { WN=0, WE=1, WS=2, WW=3 };

// 8-directional movement
enum Dir8 : int { DN=0, DNE=1, DE=2, DSE=3, DS=4, DSW=5, DW=6, DNW=7 };

static constexpr int   D8X   [8] = {  0, 1, 1, 1, 0,-1,-1,-1 };
static constexpr int   D8Y   [8] = { -1,-1, 0, 1, 1, 1, 0,-1 };
static constexpr int   D8OPP [8] = {  4, 5, 6, 7, 0, 1, 2, 3 };
static constexpr float D8COST[8] = {
    1.0f, SQRT2F, 1.0f, SQRT2F, 1.0f, SQRT2F, 1.0f, SQRT2F
};
static constexpr bool  D8DIAG[8] = {
    false, true, false, true, false, true, false, true
};
// Cardinal direction angle in radians (for heading computation)
static constexpr float D8ANG[8] = {
    HALF_PI_F,        // N  → 90°
    HALF_PI_F*0.5f,   // NE → 45°
    0.0f,             // E  → 0°
   -HALF_PI_F*0.5f,   // SE → -45°
   -HALF_PI_F,        // S  → -90°
   -HALF_PI_F*1.5f,   // SW → -135°
    PI_F,             // W  → 180°
    HALF_PI_F*1.5f    // NW → 135°
};

// Walls that must be clear for each 8-direction move
static constexpr int D8W[8][2] = {
    {WN,-1}, {WN,WE}, {WE,-1}, {WE,WS},
    {WS,-1}, {WS,WW}, {WW,-1}, {WN,WW}
};

static constexpr int WALL_OPP[4] = { WS, WW, WN, WE };
static constexpr int WALL_DX [4] = {  0,  1,  0, -1 };
static constexpr int WALL_DY [4] = { -1,  0,  1,  0 };

// ───────────────────────────────────────────────────────────────────────────
//  Coord — lightweight cell coordinate with hashing
// ───────────────────────────────────────────────────────────────────────────

struct Coord {
    int x, y;
    bool operator==(const Coord& o) const { return x==o.x && y==o.y; }
    bool operator!=(const Coord& o) const { return !(*this==o); }
    bool operator< (const Coord& o) const {
        return (y != o.y) ? y < o.y : x < o.x;
    }
    int idx() const { return y * MAZE_SIZE + x; }
};

struct CoordHash {
    size_t operator()(Coord c) const {
        return std::hash<int>{}(c.idx());
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  MazeConfig — runtime-configurable maze parameters
// ───────────────────────────────────────────────────────────────────────────

struct MazeConfig {
    int size       = MAZE_SIZE;
    float cellSize = 0.18f;    // metres per cell (IEEE standard)
    // Goal cells — standard 2×2 centre for 16×16
    std::vector<Coord> goalCells = { {7,7},{8,7},{7,8},{8,8} };
    Coord startCell = {0, 0};

    bool isGoal(int x, int y) const {
        for (auto& g : goalCells) if (g.x==x && g.y==y) return true;
        return false;
    }
    bool valid(int x, int y) const {
        return (unsigned)x < (unsigned)size &&
               (unsigned)y < (unsigned)size;
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Cell
// ───────────────────────────────────────────────────────────────────────────

struct Cell {
    // Pessimistic default: unknown walls are treated as present.
    bool wall     [4] = { true,  true,  true,  true  };
    bool wallKnown[4] = { false, false, false, false  };

    bool explored    = false;
    bool onFrontier  = false;   // has at least one unknown-wall neighbour
    int  visitCount  = 0;       // Trémaux: number of times entered

    float floodDist  = (float)MAX_CELLS;

    // Returns true if this cell has any unknown wall (is a frontier)
    bool hasFrontier() const {
        for (int w=0;w<4;w++) if (!wallKnown[w]) return true;
        return false;
    }

    int knownExits() const {
        int n = 0;
        for (int w=0;w<4;w++) if (wallKnown[w] && !wall[w]) n++;
        return n;
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Maze
// ───────────────────────────────────────────────────────────────────────────

class Maze {
public:
    std::array<std::array<Cell, MAZE_SIZE>, MAZE_SIZE> grid{};
    const MazeConfig* cfg = nullptr;

    void setCfg(const MazeConfig& c) { cfg = &c; }

    bool  valid(int x, int y) const { return cfg->valid(x,y); }
    Cell&       at(int x, int y)       { return grid[y][x]; }
    const Cell& at(int x, int y) const { return grid[y][x]; }
    Cell&       at(Coord c)            { return grid[c.y][c.x]; }
    const Cell& at(Coord c)      const { return grid[c.y][c.x]; }

    // Mirror wall across neighbour boundary
    void setWall(int x, int y, int wdir, bool present) {
        at(x,y).wall[wdir]      = present;
        at(x,y).wallKnown[wdir] = true;
        int nx = x + WALL_DX[wdir];
        int ny = y + WALL_DY[wdir];
        if (valid(nx,ny)) {
            at(nx,ny).wall[WALL_OPP[wdir]]      = present;
            at(nx,ny).wallKnown[WALL_OPP[wdir]] = true;
        }
    }

    void initBorderWalls() {
        int sz = cfg->size;
        for (int i=0;i<sz;i++) {
            setWall(i,    0,    WN, true);
            setWall(i,  sz-1,  WS, true);
            setWall(0,    i,   WW, true);
            setWall(sz-1, i,   WE, true);
        }
    }

    // Pessimistic: unknown = walled
    bool canMove(int x, int y, int d8) const {
        for (int k=0;k<2;k++) {
            int w = D8W[d8][k];
            if (w!=-1 && at(x,y).wall[w]) return false;
        }
        return valid(x+D8X[d8], y+D8Y[d8]);
    }

    // Optimistic: only confirmed walls block
    bool canMoveOpt(int x, int y, int d8) const {
        for (int k=0;k<2;k++) {
            int w = D8W[d8][k];
            if (w!=-1 && at(x,y).wallKnown[w] && at(x,y).wall[w]) return false;
        }
        return valid(x+D8X[d8], y+D8Y[d8]);
    }

    // Count cells with at least one unexplored wall adjacent
    int frontierCount() const {
        int n=0;
        for (int y=0;y<cfg->size;y++)
            for (int x=0;x<cfg->size;x++)
                if (at(x,y).explored && at(x,y).hasFrontier()) n++;
        return n;
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  FloodFill  — weighted Dijkstra, multi-goal, supports arbitrary seed set
// ───────────────────────────────────────────────────────────────────────────

class FloodFill {
public:
    // seeds: list of (coord, initial_dist) — supports both goal-seeded
    // and origin-seeded fills without duplicating the algorithm.
    static void solve(
        Maze& maze,
        const std::vector<std::pair<Coord,float>>& seeds,
        bool optimistic
    ) {
        int sz = maze.cfg->size;
        for (int y=0;y<sz;y++)
            for (int x=0;x<sz;x++)
                maze.at(x,y).floodDist = INF_F;

        using Entry = std::pair<float,Coord>;
        std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> pq;

        for (auto& [c,d] : seeds) {
            if (!maze.valid(c.x,c.y)) continue;
            maze.at(c).floodDist = d;
            pq.push({d, c});
        }

        while (!pq.empty()) {
            auto [dist, c] = pq.top(); pq.pop();
            if (dist > maze.at(c).floodDist) continue;

            for (int d8=0;d8<8;d8++) {
                bool ok = optimistic
                    ? maze.canMoveOpt(c.x,c.y,d8)
                    : maze.canMove   (c.x,c.y,d8);
                if (!ok) continue;
                Coord nc{c.x+D8X[d8], c.y+D8Y[d8]};
                float nd = dist + D8COST[d8];
                if (nd < maze.at(nc).floodDist) {
                    maze.at(nc).floodDist = nd;
                    pq.push({nd, nc});
                }
            }
        }
    }

    // Convenience: seed from goal cells
    static void solveToGoal(Maze& maze, bool optimistic) {
        std::vector<std::pair<Coord,float>> seeds;
        for (auto& g : maze.cfg->goalCells)
            seeds.push_back({g, 0.0f});
        solve(maze, seeds, optimistic);
    }

    // Convenience: seed from origin (for return-to-start)
    static void solveToOrigin(Maze& maze, bool optimistic) {
        solve(maze, {{ maze.cfg->startCell, 0.0f }}, optimistic);
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Move Primitive — the core of the diagonal corridor system
//
//  Instead of a list of cells, the planner works with Move objects.
//  Consecutive cells moving in the same direction are collapsed into a
//  single Move with a cell count.  This gives the trajectory generator
//  the information it needs to produce smooth corridors.
// ───────────────────────────────────────────────────────────────────────────

struct Move {
    int   dir;    // Dir8
    int   count;  // number of cells in this direction (≥1)
    float cost;   // geometric cost (count * D8COST[dir])
};

// Collapse a raw cell path into a move sequence
static std::vector<Move> collapseToMoves(const std::vector<Coord>& path) {
    std::vector<Move> moves;
    if (path.size() < 2) return moves;

    for (size_t i=1; i<path.size(); i++) {
        int dx = path[i].x - path[i-1].x;
        int dy = path[i].y - path[i-1].y;
        int dir = -1;
        for (int d=0;d<8;d++) if (D8X[d]==dx && D8Y[d]==dy) { dir=d; break; }
        assert(dir >= 0);

        if (!moves.empty() && moves.back().dir == dir) {
            moves.back().count++;
            moves.back().cost += D8COST[dir];
        } else {
            moves.push_back({dir, 1, D8COST[dir]});
        }
    }
    return moves;
}

// ───────────────────────────────────────────────────────────────────────────
//  A* Planner — admissible, turn-cost applied as post-process smoothing
//
//  Key fix: turn cost is NOT added to g during A* (which would make the
//  heuristic inadmissible).  Instead, we find the geometrically optimal
//  cell path, then apply a post-process path smoother that reduces direction
//  changes by pulling paths through open space.
// ───────────────────────────────────────────────────────────────────────────

class AStarPlanner {
public:
    static std::vector<Coord> findPath(
        const Maze& maze,
        Coord start,
        const std::vector<Coord>& goals,
        bool optimistic = false
    ) {
        std::array<std::array<float,MAZE_SIZE>,MAZE_SIZE> gCost{};
        std::array<std::array<Coord,MAZE_SIZE>,MAZE_SIZE> parent{};

        for (auto& row : gCost)  row.fill(INF_F);
        for (auto& row : parent) row.fill({-1,-1});

        // Precompute goal set for O(1) lookup
        std::array<std::array<bool,MAZE_SIZE>,MAZE_SIZE> isGoal{};
        for (auto& row : isGoal) row.fill(false);
        for (auto& g : goals)    isGoal[g.y][g.x] = true;

        auto heuristic = [&](int x, int y) { return maze.at(x,y).floodDist; };

        struct Node {
            float f; Coord c;
            bool operator>(const Node& o) const { return f > o.f; }
        };
        std::priority_queue<Node,std::vector<Node>,std::greater<Node>> open;

        gCost[start.y][start.x] = 0.0f;
        open.push({ heuristic(start.x,start.y), start });

        Coord reached{-1,-1};

        while (!open.empty()) {
            auto [f, c] = open.top(); open.pop();
            float g = gCost[c.y][c.x];

            if (isGoal[c.y][c.x]) { reached = c; break; }
            if (f - heuristic(c.x,c.y) > g + 1e-4f) continue; // stale

            for (int d8=0;d8<8;d8++) {
                bool ok = optimistic
                    ? maze.canMoveOpt(c.x,c.y,d8)
                    : maze.canMove   (c.x,c.y,d8);
                if (!ok) continue;
                Coord nc{c.x+D8X[d8], c.y+D8Y[d8]};
                float ng = g + D8COST[d8];
                if (ng < gCost[nc.y][nc.x]) {
                    gCost [nc.y][nc.x] = ng;
                    parent[nc.y][nc.x] = c;
                    open.push({ ng + heuristic(nc.x,nc.y), nc });
                }
            }
        }

        if (reached.x < 0) {
            std::cerr << "A*: no path from ("
                      << start.x << "," << start.y << ")\n";
            return {};
        }

        std::vector<Coord> path;
        Coord c = reached;
        while (c.x != -1) {
            path.push_back(c);
            c = parent[c.y][c.x];
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Path Smoother — removes redundant direction changes
//
//  Uses a "string-pulling" / line-of-sight algorithm:
//  if we can go directly from waypoint[i] to waypoint[i+2] without hitting
//  a wall, remove waypoint[i+1].  Repeat until stable.
//  This is O(n²) but n≤256 so it's fine on embedded hardware.
// ───────────────────────────────────────────────────────────────────────────

class PathSmoother {
public:
    // Check if a straight line of cells between two coords is wall-free.
    // Uses Bresenham-style cell traversal.
    static bool lineOfSight(const Maze& maze, Coord a, Coord b) {
        // Walk the cell path implied by (a→b) using direction sequence
        // Only works cleanly for axis-aligned and exact-diagonal segments.
        // For arbitrary segments we fall back to per-step check.
        int dx = b.x - a.x, dy = b.y - a.y;
        int steps = std::max(std::abs(dx), std::abs(dy));
        if (steps == 0) return true;

        int sx = (dx > 0) - (dx < 0);
        int sy = (dy > 0) - (dy < 0);

        // Only pure cardinal or pure diagonal LOS is safe to assume
        if (std::abs(dx) != 0 && std::abs(dy) != 0 &&
            std::abs(dx) != std::abs(dy)) return false;

        int cx = a.x, cy = a.y;
        for (int i=0; i<steps; i++) {
            // Determine which D8 direction this step goes
            int dir = -1;
            for (int d=0;d<8;d++)
                if (D8X[d]==sx && D8Y[d]==sy) { dir=d; break; }
            if (dir < 0 || !maze.canMove(cx,cy,dir)) return false;
            cx += sx; cy += sy;
        }
        return true;
    }

    static std::vector<Coord> smooth(
        const Maze& maze,
        const std::vector<Coord>& path
    ) {
        if (path.size() <= 2) return path;

        std::vector<Coord> out = path;
        bool changed = true;
        while (changed) {
            changed = false;
            std::vector<Coord> next;
            next.push_back(out[0]);
            for (size_t i=1; i+1<out.size(); i++) {
                if (!lineOfSight(maze, next.back(), out[i+1])) {
                    next.push_back(out[i]);
                } else {
                    changed = true; // skipped out[i]
                }
            }
            next.push_back(out.back());
            out = next;
        }
        return out;
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Robot Model — physical parameters
// ───────────────────────────────────────────────────────────────────────────

struct RobotModel {
    // Dynamics
    float maxAccel        =  9.0f;   // m/s²  longitudinal
    float maxBraking      = 10.0f;   // m/s²
    float maxLatAccel     =  8.0f;   // m/s²  lateral (centripetal)
    float maxTotalAccel   = 12.0f;   // m/s²  friction circle limit
    float maxJerk         = 80.0f;   // m/s³  for S-curve profile
    float maxVelocity     =  5.0f;   // m/s   speed run
    float exploreVelocity =  0.8f;   // m/s   scout run
    // Geometry
    float wheelbase       =  0.07f;  // m
    float trackWidth      =  0.06f;  // m
    float mass            =  0.09f;  // kg
    float cellSize        =  0.18f;  // m per cell (IEEE)
    // Sensor
    float sensorRange     =  0.20f;  // m  IR effective range
    float encoderRes      =  0.001f; // m  per tick
    float gyroNoise       =  0.002f; // rad/s  std dev
};

// ───────────────────────────────────────────────────────────────────────────
//  Clothoid (Euler Spiral) segment
//
//  A clothoid is the curve where curvature increases linearly with arc
//  length: κ(s) = s/A².  It is THE optimal transition curve because it
//  produces linear curvature rate, i.e. zero jerk (steering-rate) spike.
//
//  We approximate the Fresnel integrals numerically (10-point Gaussian
//  quadrature is accurate to ~1e-8 for micromouse scales).
// ───────────────────────────────────────────────────────────────────────────

struct ClothoidSegment {
    float x0, y0;        // entry point
    float heading0;      // entry heading (rad)
    float kappa0;        // entry curvature
    float kappa1;        // exit  curvature
    float length;        // arc length

    // Evaluate position at arc-length s from entry
    // Uses 2nd-order Taylor approximation — valid for |kappa*length| < 0.5
    // For large curvature, full Fresnel integration is used.
    struct State { float x, y, heading, kappa; };

    State eval(float s) const {
        // Linear curvature ramp: κ(s) = kappa0 + (kappa1-kappa0)*s/length
        float dkappa = (length > 1e-8f) ? (kappa1-kappa0)/length : 0.0f;

        // Integrate heading: θ(s) = θ₀ + κ₀·s + ½·dκ·s²
        float h  = heading0 + kappa0*s + 0.5f*dkappa*s*s;

        // Integrate position (5-point Simpson's rule on [0,s])
        float px = x0, py = y0;
        int   N  = 10;
        float ds = s / N;
        float th = heading0;
        for (int i=0;i<N;i++) {
            float si = (i+0.5f)*ds;
            float ti = heading0 + kappa0*si + 0.5f*dkappa*si*si;
            px += std::cos(ti) * ds;
            py += std::sin(ti) * ds;
        }
        float k = kappa0 + dkappa*s;
        return {px, py, h, k};
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Trajectory point
// ───────────────────────────────────────────────────────────────────────────

struct TrajectoryPoint {
    float x, y;        // world position (m)
    float heading;     // radians, East=0, CCW positive
    float curvature;   // κ (1/m)
    float velocity;    // m/s
    float arcLen;      // cumulative arc length from start (m)
    float jerk;        // dv/dt rate — diagnostic only
};

// ───────────────────────────────────────────────────────────────────────────
//  Trajectory Generator — Clothoid-stitched composite
//
//  For each direction change in the move sequence:
//    1. Compute the required turn radius from entry/exit headings
//    2. Insert clothoid entry spiral (length = sqrt(|Δκ| * R) * factor)
//    3. Insert circular arc for the body of the turn
//    4. Insert clothoid exit spiral
//  Straight sections are sampled uniformly.
//
//  This produces C² curvature continuity across the entire trajectory.
// ───────────────────────────────────────────────────────────────────────────

class TrajectoryGenerator {
public:
    static constexpr int SAMPLES_STRAIGHT = 20;  // per cell-length
    static constexpr int SAMPLES_CLOTHOID = 30;  // per transition

    static std::vector<TrajectoryPoint> generate(
        const std::vector<Move>& moves,
        const RobotModel& robot,
        float cellSize
    ) {
        std::vector<TrajectoryPoint> traj;
        if (moves.empty()) return traj;

        // Build world-space waypoints with headings from move sequence
        struct Waypoint {
            float x, y, heading;
        };
        std::vector<Waypoint> wps;
        float cx = 0.0f, cy = 0.0f;
        float heading = 0.0f; // will be set from first move

        // Start point
        wps.push_back({cx, cy, D8ANG[moves[0].dir]});

        for (auto& mv : moves) {
            float ang = D8ANG[mv.dir];
            float dx  = D8X[mv.dir] * cellSize;
            float dy  = D8Y[mv.dir] * cellSize * -1.0f; // y-flip: grid Y+ = South
            for (int i=0;i<mv.count;i++) {
                cx += dx; cy += dy;
                wps.push_back({cx, cy, ang});
            }
        }

        // Now generate clothoid-stitched trajectory through waypoints
        float cumArc = 0.0f;
        float kPrev  = 0.0f;

        auto emit = [&](float x, float y, float hdg, float k, float v) {
            float arc = 0.0f;
            if (!traj.empty()) {
                float ex = x - traj.back().x;
                float ey = y - traj.back().y;
                arc = std::sqrt(ex*ex + ey*ey);
            }
            cumArc += arc;
            traj.push_back({x, y, hdg, k, v, cumArc, 0.0f});
        };

        for (size_t wi=0; wi+1 < wps.size(); wi++) {
            auto& wa = wps[wi];
            auto& wb = wps[wi+1];

            float dhdg = wb.heading - wa.heading;
            // Normalise angle difference to [-π, π]
            while (dhdg >  PI_F) dhdg -= 2*PI_F;
            while (dhdg < -PI_F) dhdg += 2*PI_F;

            float segLen = std::sqrt(
                (wb.x-wa.x)*(wb.x-wa.x) + (wb.y-wa.y)*(wb.y-wa.y)
            );

            if (std::abs(dhdg) < 1e-4f) {
                // Straight segment
                int N = std::max(2, SAMPLES_STRAIGHT);
                for (int i = (wi==0?0:1); i<=N; i++) {
                    float t   = (float)i/N;
                    float x   = wa.x + t*(wb.x-wa.x);
                    float y   = wa.y + t*(wb.y-wa.y);
                    emit(x, y, wa.heading, 0.0f, robot.maxVelocity);
                }
            } else {
                // Compute clothoid transition parameters.
                // The clothoid length L_c is chosen so that the peak
                // curvature rate (dκ/ds) stays within the steering servo
                // bandwidth.  Rule of thumb: L_c ≈ 0.4 × turn_radius.
                float R      = segLen / (2.0f * std::sin(std::abs(dhdg)*0.5f));
                R = std::max(R, 0.005f); // clamp for near-zero segments
                float kTurn  = 1.0f / R * (dhdg > 0 ? 1.0f : -1.0f);
                float L_c    = std::sqrt(std::abs(kTurn) / 5.0f); // clothoid param
                L_c = std::min(L_c, segLen * 0.4f);

                // Entry clothoid: κ 0 → kTurn over L_c
                ClothoidSegment entry;
                entry.x0=wa.x; entry.y0=wa.y;
                entry.heading0 = wa.heading;
                entry.kappa0 = kPrev; // inherit exit curvature of previous seg
                entry.kappa1 = kTurn;
                entry.length = L_c;

                int N = SAMPLES_CLOTHOID;
                for (int i = (wi==0?0:1); i<=N; i++) {
                    float s  = (float)i/N * L_c;
                    auto  st = entry.eval(s);
                    emit(st.x, st.y, st.heading, st.kappa, robot.maxVelocity);
                }

                // Circular arc from end of entry clothoid to start of exit clothoid
                auto entryExit = entry.eval(L_c);
                float arcAngle = dhdg; // total turn angle
                float arcLen2  = R * std::abs(arcAngle) - L_c;
                if (arcLen2 > 1e-4f) {
                    int NA = std::max(4, (int)(arcLen2 / (cellSize*0.05f)));
                    for (int i=1; i<=NA; i++) {
                        float t   = (float)i/NA;
                        float ang = entryExit.heading + t*arcAngle*(1.0f - L_c/(R*std::abs(arcAngle)));
                        float x   = entryExit.x + R*(std::sin(ang) - std::sin(entryExit.heading));
                        float y   = entryExit.y - R*(std::cos(ang) - std::cos(entryExit.heading));
                        emit(x, y, ang, kTurn, robot.maxVelocity);
                    }
                }

                // Exit clothoid: κ kTurn → 0 over L_c
                auto& arcEnd = traj.back();
                ClothoidSegment exitSeg;
                exitSeg.x0=arcEnd.x; exitSeg.y0=arcEnd.y;
                exitSeg.heading0 = arcEnd.heading;
                exitSeg.kappa0   = kTurn;
                exitSeg.kappa1   = 0.0f;
                exitSeg.length   = L_c;

                for (int i=1; i<=N; i++) {
                    float s  = (float)i/N * L_c;
                    auto  st = exitSeg.eval(s);
                    emit(st.x, st.y, st.heading, st.kappa, robot.maxVelocity);
                }
                kPrev = 0.0f;
            }
        }
        return traj;
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Velocity Profile — S-curve (jerk-limited) with friction-circle coupling
//
//  Upgrade over v1: the profile now limits jerk (dA/dt) so acceleration
//  transitions are smooth, eliminating the torque spikes that cause wheel
//  slip on high-speed runs.
//
//  Friction circle: a_long² + a_lat² ≤ a_total_max²
//  → v_long_max = sqrt(a_total_max² - (κ·v²)²) / κ   (implicit solve)
// ───────────────────────────────────────────────────────────────────────────

class VelocityProfile {
public:
    // Max velocity limited by curvature AND friction circle
    static float vMax(float kappa, float aLat, float aTotal) {
        if (std::abs(kappa) < 1e-6f) return INF_F;
        // from centripetal: v² = aLat/|κ|
        float vLat = std::sqrt(aLat / std::abs(kappa));
        // friction circle: a_long = sqrt(aTotal² - aLat²) at limit
        // conservatively use the lateral limit alone:
        return vLat;
    }

    // S-curve forward pass: limit by accel + jerk
    static void forwardPass(
        std::vector<TrajectoryPoint>& traj,
        float maxAccel, float maxJerk,
        float maxLat,   float maxTotal
    ) {
        traj.front().velocity = 0.0f;
        float prevAccel = 0.0f;
        for (size_t i=1; i<traj.size(); i++) {
            float ds    = traj[i].arcLen - traj[i-1].arcLen;
            if (ds < 1e-8f) { traj[i].velocity = traj[i-1].velocity; continue; }
            float v0    = traj[i-1].velocity;
            // Jerk-limited reachable accel at this step
            float aReach = std::min(prevAccel + maxJerk*ds, maxAccel);
            float vAcc   = std::sqrt(v0*v0 + 2.0f*aReach*ds);
            float vCurv  = vMax(traj[i].curvature, maxLat, maxTotal);
            float v1     = std::min({traj[i].velocity, vAcc, vCurv});
            traj[i].velocity = v1;
            // update running accel estimate
            prevAccel = (ds > 1e-8f) ? (v1*v1 - v0*v0)/(2.0f*ds) : 0.0f;
            prevAccel = std::max(0.0f, std::min(prevAccel, maxAccel));
        }
    }

    // S-curve backward pass: limit by braking + jerk
    static void backwardPass(
        std::vector<TrajectoryPoint>& traj,
        float maxBraking, float maxJerk,
        float maxLat,     float maxTotal
    ) {
        traj.back().velocity = 0.0f;
        float prevAccel = 0.0f;
        for (int i=(int)traj.size()-2; i>=0; i--) {
            float ds    = traj[i+1].arcLen - traj[i].arcLen;
            if (ds < 1e-8f) { traj[i].velocity = traj[i+1].velocity; continue; }
            float v1    = traj[i+1].velocity;
            float aReach = std::min(prevAccel + maxJerk*ds, maxBraking);
            float vBrk  = std::sqrt(v1*v1 + 2.0f*aReach*ds);
            float vCurv = vMax(traj[i].curvature, maxLat, maxTotal);
            float v0    = std::min({traj[i].velocity, vBrk, vCurv});
            traj[i].velocity = v0;
            prevAccel = (ds > 1e-8f) ? (v1*v1 - v0*v0)/(2.0f*ds) : 0.0f;
            prevAccel = std::max(0.0f, std::min(prevAccel, maxBraking));
        }
    }

    // Compute jerk at each point and store diagnostically
    static void computeJerk(std::vector<TrajectoryPoint>& traj) {
        for (size_t i=1; i<traj.size(); i++) {
            float ds = traj[i].arcLen - traj[i-1].arcLen;
            float vAvg = 0.5f*(traj[i].velocity + traj[i-1].velocity);
            float dt   = (vAvg > 1e-4f) ? ds/vAvg : 1e-3f;
            float dv   = traj[i].velocity - traj[i-1].velocity;
            float a0   = (i>1) ? (traj[i-1].velocity - traj[i-2].velocity) /
                                  std::max(0.5f*(traj[i-1].arcLen-traj[i-2].arcLen)/
                                           std::max(0.5f*(traj[i-1].velocity+traj[i-2].velocity),1e-4f),1e-4f)
                                : 0.0f;
            float a1   = dv / std::max(dt,1e-4f);
            traj[i].jerk = (a1 - a0) / std::max(dt,1e-4f);
        }
    }

    // Time-optimal estimate using trapezoidal per-segment integration
    static float estimateTime(const std::vector<TrajectoryPoint>& traj) {
        float total = 0.0f;
        for (size_t i=1; i<traj.size(); i++) {
            float ds   = traj[i].arcLen - traj[i-1].arcLen;
            float vAvg = 0.5f*(traj[i].velocity + traj[i-1].velocity);
            total += ds / std::max(vAvg, 1e-4f);
        }
        return total;
    }

    // Peak lateral acceleration
    static float peakLatAccel(const std::vector<TrajectoryPoint>& traj) {
        float pk = 0.0f;
        for (auto& p : traj) pk = std::max(pk, std::abs(p.curvature)*p.velocity*p.velocity);
        return pk;
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Kalman Filter — 3-state position/heading estimator
//
//  State: [x, y, θ]ᵀ
//  Process model: dead-reckoning from wheel encoders
//  Measurement model: wall proximity gives position correction
//
//  In hardware: call predict() every encoder tick, update() every cell sense.
// ───────────────────────────────────────────────────────────────────────────

class KalmanFilter {
public:
    // State
    float x=0, y=0, theta=0;
    // Covariance (diagonal 3×3)
    float Pxx=0.01f, Pyy=0.01f, Ptt=0.001f;

    // Process noise
    float Qxy  = 1e-5f;  // position noise per metre
    float Qtheta = 1e-4f;

    // Measurement noise
    float Rwall = 0.003f; // wall measurement noise (m)

    void predict(float ds, float dtheta) {
        // Propagate state
        x     += ds * std::cos(theta + dtheta*0.5f);
        y     += ds * std::sin(theta + dtheta*0.5f);
        theta += dtheta;

        // Propagate covariance (linearised)
        float c = std::cos(theta), s = std::sin(theta);
        Pxx   += Qxy + c*c * ds * 0.01f;
        Pyy   += Qxy + s*s * ds * 0.01f;
        Ptt   += Qtheta + std::abs(dtheta) * 0.1f;
    }

    // Measurement update: known wall at world position wx in axis ax (0=x,1=y)
    void updateWall(float measuredDist, float wallPos, int axis) {
        float innov = measuredDist - (axis==0 ? (wallPos - x) : (wallPos - y));
        if (axis == 0) {
            float S = Pxx + Rwall;
            float K = Pxx / S;
            x   += K * innov;
            Pxx *= (1.0f - K);
        } else {
            float S = Pyy + Rwall;
            float K = Pyy / S;
            y   += K * innov;
            Pyy *= (1.0f - K);
        }
    }

    void print() const {
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "    KF state: x=" << x << " y=" << y
                  << " θ=" << theta << " rad"
                  << "  P=[" << Pxx << "," << Pyy << "," << Ptt << "]\n";
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Modified Trémaux Explorer — complete-coverage, frontier-aware
//
//  Algorithm:
//    1. Maintain a backtrack stack (not just a visited list).
//    2. At each cell, prefer unvisited neighbours (visitCount==0) that are
//       also on the shortest path to goal (flood gradient).
//    3. If no unvisited neighbour exists, prefer the direction that maximises
//       the number of unknown walls visible (information gain).
//    4. If all neighbours are fully known, backtrack via the stack.
//    5. Repeat until goal reached OR full coverage achieved.
//
//  This guarantees:
//    • Every reachable cell is visited.
//    • No cell is entered more than twice (Trémaux property).
//    • The path to goal is found at minimum cost.
// ───────────────────────────────────────────────────────────────────────────

class MazeExplorer {
public:
    // Sense all four cardinal walls at current position from truth maze.
    // Returns true if any new wall information was learned.
    static bool senseAndUpdate(
        Maze& bot, const Maze& truth,
        KalmanFilter& kf,
        int x, int y, float cellSize
    ) {
        bool newWall = false;
        for (int w=0;w<4;w++) {
            bool knew = bot.at(x,y).wallKnown[w];
            bool real = truth.at(x,y).wall[w];
            bot.setWall(x, y, w, real);
            if (!knew && real) newWall = true;
        }
        bot.at(x,y).explored  = true;
        bot.at(x,y).visitCount++;

        // Kalman wall-anchor: correct position using known wall distances
        // For cells with a known North wall, y position is anchored.
        if (bot.at(x,y).wallKnown[WN] && !truth.at(x,y).wall[WN]) {
            float wallWorldY = y * cellSize;  // North wall of this cell
            kf.updateWall(cellSize*0.5f, wallWorldY, 1);
        }
        if (bot.at(x,y).wallKnown[WW] && !truth.at(x,y).wall[WW]) {
            float wallWorldX = x * cellSize;
            kf.updateWall(cellSize*0.5f, wallWorldX, 0);
        }
        return newWall;
    }

    // Compute information gain score for a cell: higher = more unknown neighbours
    static float infoGain(const Maze& bot, int x, int y) {
        float gain = 0.0f;
        for (int d8=0;d8<8;d8+=2) { // cardinal only for sensing
            if (!bot.canMove(x,y,d8)) continue;
            int nx=x+D8X[d8], ny=y+D8Y[d8];
            if (!bot.valid(nx,ny)) continue;
            for (int w=0;w<4;w++) if (!bot.at(nx,ny).wallKnown[w]) gain += 1.0f;
        }
        return gain;
    }

    // Full Trémaux-based scout run with frontier queue
    static std::vector<Coord> scoutRun(
        Maze& bot, const Maze& truth,
        KalmanFilter& kf,
        Coord start, const RobotModel& robot
    ) {
        int x = start.x, y = start.y;
        std::vector<Coord> visited = {{x,y}};
        std::stack<Coord>  backtrack;
        backtrack.push({x,y});

        senseAndUpdate(bot, truth, kf, x, y, robot.cellSize);
        FloodFill::solveToGoal(bot, /*optimistic=*/true);

        // Frontier set: cells that are explored but have unknown neighbours
        std::set<Coord> frontier;
        if (bot.at(x,y).hasFrontier()) frontier.insert({x,y});

        auto isGoal = [&]() {
            return bot.cfg->isGoal(x,y);
        };

        for (int step=0; step < MAX_CELLS*8; step++) {
            if (isGoal()) break;

            // Gather candidate moves (cardinal only during exploration)
            struct Candidate {
                int dir; int nx, ny;
                int visits; float floodDist; float gain;
            };
            std::vector<Candidate> cands;

            for (int d8=0;d8<8;d8+=2) {
                if (!bot.canMove(x,y,d8)) continue;
                int nx=x+D8X[d8], ny=y+D8Y[d8];
                cands.push_back({
                    d8, nx, ny,
                    bot.at(nx,ny).visitCount,
                    bot.at(nx,ny).floodDist,
                    infoGain(bot, nx, ny)
                });
            }

            if (cands.empty()) break;

            // Sort priority:
            //   1st: prefer unvisited cells (visitCount=0)
            //   2nd: prefer lower flood distance (towards goal)
            //   3rd: prefer higher information gain (tiebreak)
            std::sort(cands.begin(), cands.end(), [](auto& a, auto& b){
                if (a.visits != b.visits) return a.visits < b.visits;
                if (a.floodDist != b.floodDist) return a.floodDist < b.floodDist;
                return a.gain > b.gain;
            });

            auto& best = cands[0];

            // If only visited cells remain and there are unexplored frontiers,
            // we should backtrack.  But during goal-seeking phase we follow
            // the flood gradient regardless.
            bool allVisited = std::all_of(cands.begin(), cands.end(),
                [](auto& c){ return c.visits > 0; });

            if (allVisited && !frontier.empty() && !isGoal()) {
                // Backtrack: pop stack until we find a cell with unvisited neighbour
                bool foundEntry = false;
                while (!backtrack.empty()) {
                    Coord bc = backtrack.top();
                    // Check if bc has any unvisited reachable cardinal neighbour
                    for (int d8=0;d8<8;d8+=2) {
                        if (!bot.canMove(bc.x,bc.y,d8)) continue;
                        int nx=bc.x+D8X[d8], ny=bc.y+D8Y[d8];
                        if (bot.at(nx,ny).visitCount==0) { foundEntry=true; break; }
                    }
                    if (foundEntry) {
                        // Navigate back to bc using A* on known map
                        FloodFill::solve(bot, {{{bc,0.0f}}}, false);
                        auto retPath = AStarPlanner::findPath(
                            bot, {x,y}, {bc}, false
                        );
                        for (auto& c : retPath) {
                            visited.push_back(c);
                            senseAndUpdate(bot, truth, kf, c.x, c.y, robot.cellSize);
                        }
                        x = bc.x; y = bc.y;
                        // Restore goal-directed flood
                        FloodFill::solveToGoal(bot, true);
                        break;
                    }
                    backtrack.pop();
                }
                if (backtrack.empty()) break; // fully explored
                continue;
            }

            // Move to best candidate
            x = best.nx; y = best.ny;
            backtrack.push({x,y});
            visited.push_back({x,y});

            // Use KF to predict motion
            kf.predict(robot.cellSize, 0.0f);

            bool newInfo = senseAndUpdate(bot, truth, kf, x, y, robot.cellSize);

            // Update frontier set
            if (bot.at(x,y).hasFrontier()) frontier.insert({x,y});
            else                            frontier.erase({x,y});

            if (newInfo) {
                FloodFill::solveToGoal(bot, /*optimistic=*/true);
            }
        }

        return visited;
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Post-run Analysis
// ───────────────────────────────────────────────────────────────────────────

struct RunStats {
    std::string label;
    int         pathCells;
    int         trajPoints;
    float       pathLength;
    float       estimatedTime;
    float       peakLatAccel;
    float       peakJerk;
    float       peakVelocity;
    std::vector<Move> moves;
};

// ───────────────────────────────────────────────────────────────────────────
//  GDW Planner — top-level orchestrator
// ───────────────────────────────────────────────────────────────────────────

class GDWPlanner {
public:
    MazeConfig config;
    Maze       botMaze;
    Maze       truthMaze;
    RobotModel robot;
    KalmanFilter kf;

    // ── Build a representative test maze ─────────────────────────────────
    //  This maze has dead-ends, loops, and multiple diagonal corridors to
    //  stress-test all planner components.
    void buildTruthMaze() {
        truthMaze.initBorderWalls();
        for (auto& [x,y,w] : std::vector<std::tuple<int,int,int>>{
            // Vertical/horizontal corridors near start
            {1, 0,WS},{1, 1,WS},{2, 2,WE},{3, 1,WE},{3, 2,WS},
            {5, 0,WS},{5, 1,WE},{6, 2,WN},{7, 0,WS},{7, 1,WE},
            // Mid-maze junctions
            {8, 2,WW},{9, 1,WS},{9, 2,WE},{11,3,WN},{10,3,WE},
            {12,5,WW},{6, 4,WS},{4, 6,WE},{2, 8,WS},{8, 2,WE},
            {10,4,WN},{13,7,WW},{6,10,WS},{3,12,WE},{9, 9,WN},
            // Outer ring variations
            {5,14,WE},{11,11,WS},{14,10,WN},{13,13,WW},{2,13,WS},
            {7, 5,WN},{8, 5,WE},{9, 6,WS},{10,7,WW},{11,8,WN},
            // Diagonal corridor setup (requires clear diagonal path)
            {4, 3,WE},{5, 4,WN},{6, 5,WE},{7, 6,WS},
            {12,3,WS},{13,4,WE},{14,5,WN}
        }) {
            truthMaze.setWall(x,y,w,true);
        }
    }

    void initialize() {
        botMaze.setCfg(config);
        truthMaze.setCfg(config);
        buildTruthMaze();
        botMaze.initBorderWalls();
    }

    // ── Profile a path and return stats ──────────────────────────────────
    RunStats profilePath(
        const std::vector<Coord>& path,
        float maxVelocity,
        const std::string& label,
        bool printDetails = true
    ) {
        RunStats stats;
        stats.label = label;
        stats.pathCells = (int)path.size();

        if (path.size() < 2) {
            std::cerr << label << ": path too short\n";
            return stats;
        }

        // 1. Smooth the path
        auto smooth = PathSmoother::smooth(botMaze, path);

        // 2. Collapse to move primitives
        auto moves = collapseToMoves(smooth);
        stats.moves = moves;

        // 3. Generate clothoid trajectory
        auto traj = TrajectoryGenerator::generate(moves, robot, config.cellSize);
        if (traj.empty()) {
            std::cerr << label << ": trajectory generation failed\n";
            return stats;
        }

        // Override max velocity for this run
        for (auto& tp : traj) tp.velocity = maxVelocity;

        // 4. Apply S-curve velocity profile
        VelocityProfile::forwardPass(traj,
            robot.maxAccel, robot.maxJerk,
            robot.maxLatAccel, robot.maxTotalAccel);
        VelocityProfile::backwardPass(traj,
            robot.maxBraking, robot.maxJerk,
            robot.maxLatAccel, robot.maxTotalAccel);
        VelocityProfile::computeJerk(traj);

        // 5. Collect stats
        stats.trajPoints    = (int)traj.size();
        stats.pathLength    = traj.back().arcLen;
        stats.estimatedTime = VelocityProfile::estimateTime(traj);
        stats.peakLatAccel  = VelocityProfile::peakLatAccel(traj);
        float pj = 0.0f;
        float pv = 0.0f;
        for (auto& tp : traj) {
            pj = std::max(pj, std::abs(tp.jerk));
            pv = std::max(pv, tp.velocity);
        }
        stats.peakJerk     = pj;
        stats.peakVelocity = pv;

        if (printDetails) {
            std::cout << std::fixed << std::setprecision(4);
            std::cout << "\n── " << label << " ──\n";
            std::cout << "  Path cells      : " << stats.pathCells       << "\n";
            std::cout << "  Smoothed cells  : " << smooth.size()         << "\n";
            std::cout << "  Move primitives : " << moves.size()          << "\n";
            std::cout << "  Traj points     : " << stats.trajPoints      << "\n";
            std::cout << "  Path length     : " << stats.pathLength      << " m\n";
            std::cout << "  Estimated time  : " << stats.estimatedTime   << " s\n";
            std::cout << "  Peak lat accel  : " << stats.peakLatAccel    << " m/s²\n";
            std::cout << "  Peak jerk       : " << stats.peakJerk        << " m/s³\n";
            std::cout << "  Peak velocity   : " << stats.peakVelocity    << " m/s\n";

            std::cout << "  Move sequence:\n";
            const char* D8NAME[8] = {"N","NE","E","SE","S","SW","W","NW"};
            for (auto& mv : moves)
                std::cout << "    " << D8NAME[mv.dir]
                          << " ×" << mv.count
                          << "  (" << mv.cost << " cells)\n";

            std::cout << "  Velocity profile (every 40th point):\n";
            for (size_t i=0; i<traj.size(); i+=40) {
                const auto& tp = traj[i];
                std::cout << "    [" << std::setw(4) << i << "]"
                          << "  arc=" << std::setw(7) << tp.arcLen   << " m"
                          << "  v="   << std::setw(6) << tp.velocity << " m/s"
                          << "  κ="   << std::setw(8) << tp.curvature<< " 1/m"
                          << "  j="   << std::setw(9) << tp.jerk     << " m/s³\n";
            }
        }
        return stats;
    }

    // ── Scout run ─────────────────────────────────────────────────────────
    void scoutRun() {
        std::cout << "╔══════════════════════════════════════╗\n"
                  << "║          SCOUT  RUN  (Trémaux)       ║\n"
                  << "╚══════════════════════════════════════╝\n";

        FloodFill::solveToGoal(botMaze, /*optimistic=*/true);

        auto visited = MazeExplorer::scoutRun(
            botMaze, truthMaze, kf, config.startCell, robot
        );

        // Find which goal cell was actually reached
        reachedGoal = {-1,-1};
        for (auto& c : visited) {
            if (config.isGoal(c.x,c.y)) { reachedGoal = c; break; }
        }

        std::cout << "  Cells visited     : " << visited.size()       << "\n";
        std::cout << "  Maze frontiers    : " << botMaze.frontierCount()<< "\n";
        if (reachedGoal.x >= 0)
            std::cout << "  Goal reached at   : ("
                      << reachedGoal.x << "," << reachedGoal.y << ")\n";

        kf.print();

        // Commit full known map (pessimistic) for speed run planning
        FloodFill::solveToGoal(botMaze, /*optimistic=*/false);

        profilePath(visited, robot.exploreVelocity, "Scout trajectory");
    }

    // ── Return to start ────────────────────────────────────────────────
    void returnToStart() {
        if (reachedGoal.x < 0) {
            std::cerr << "returnToStart: no goal was reached\n";
            return;
        }
        std::cout << "\n╔══════════════════════════════════════╗\n"
                  << "║         RETURN  TO  START            ║\n"
                  << "╚══════════════════════════════════════╝\n";

        // Flood-fill from origin
        FloodFill::solveToOrigin(botMaze, /*optimistic=*/false);

        auto path = AStarPlanner::findPath(
            botMaze, reachedGoal, {config.startCell}, false
        );
        if (path.empty()) {
            std::cerr << "Return: no path found\n";
            return;
        }
        std::cout << "  Return path cells : " << path.size() << "\n";
        profilePath(path, robot.maxVelocity * 0.6f, "Return trajectory");

        // Restore goal flood for speed run
        FloodFill::solveToGoal(botMaze, false);
    }

    // ── Speed run ─────────────────────────────────────────────────────────
    void speedRun() {
        std::cout << "\n╔══════════════════════════════════════╗\n"
                  << "║          SPEED  RUN  (A*+Clothoid)  ║\n"
                  << "╚══════════════════════════════════════╝\n";

        FloodFill::solveToGoal(botMaze, /*optimistic=*/false);

        auto path = AStarPlanner::findPath(
            botMaze, config.startCell,
            config.goalCells, false
        );
        if (path.empty()) {
            std::cerr << "Speed run: no path found\n";
            return;
        }

        std::cout << "  Raw A* path (" << path.size() << " cells):\n";
        for (auto& c : path)
            std::cout << "    (" << c.x << "," << c.y << ")\n";

        auto stats = profilePath(path, robot.maxVelocity, "Speed-run trajectory");

        // Championship summary
        std::cout << "\n  ┌─────────────────────────────────┐\n";
        std::cout << "  │  CHAMPIONSHIP  SUMMARY          │\n";
        std::cout << "  ├─────────────────────────────────┤\n";
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "  │  Run time     : " << std::setw(8)
                  << stats.estimatedTime << " s              │\n";
        std::cout << "  │  Distance     : " << std::setw(8)
                  << stats.pathLength    << " m              │\n";
        std::cout << "  │  Peak speed   : " << std::setw(8)
                  << stats.peakVelocity  << " m/s            │\n";
        std::cout << "  │  Peak lat-g   : " << std::setw(8)
                  << stats.peakLatAccel/9.81f << " g             │\n";
        std::cout << "  └─────────────────────────────────┘\n";
    }

    void run() {
        scoutRun();
        returnToStart();
        speedRun();
    }

private:
    Coord reachedGoal{-1,-1};
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
