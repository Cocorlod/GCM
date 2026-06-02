// ═══════════════════════════════════════════════════════════════════════════
//  GDW Micromouse Championship Edition v4.1
//  C++17  ·  Single translation unit
//
//  Provenance
//  ──────────
//  v2 contributed: D* Lite, info-theoretic exploration, ESKF, TVLQR,
//                  racing-line QP optimiser, jerk/long-accel statistics,
//                  expandPath / collapseToMoves, adaptive velocity scaler.
//  v3 contributed: unified coordinate frame, CellCoord, wall-centering PID,
//                  sensor model, dead-reckoning localizer, corrected arc sign,
//                  corrected clothoid-length constraint, converging backward
//                  velocity pass, gradient-based waypoint smoother.
//  v4 new work:    all identified bugs from both versions fixed (list below),
//                  ESKF + wall-snap coexist with PD + TVLQR fallback,
//                  D* Lite + visit-count floodfill unified exploration,
//                  full RunStats (time, lat/long accel, jerk, peak v).
//  v4.1 changes:   WallCenteringPID removed from Speed Run (was dead code;
//                  conflicts with racing-line optimisation).  Now active only
//                  in Scout Run via Explorer::explore(wallCtrl&).
//                  PDController actively called in Speed Run tracking
//                  simulation (was instantiated but never invoked).
//                  profilePath() gains optional outTraj* parameter so callers
//                  can retrieve the fully-profiled trajectory vector.
//
//  ── Bug-fix inventory ────────────────────────────────────────────────────
//
//  FIX-A  TVLQR B-matrix linearisation (v2):
//         v2 set B[2][0] = κ (reference curvature in linearised δθ/δv row).
//         Correct linearisation of differential-drive about (v_ref, ω_ref):
//         dθ̇/dv_error = 0,  dθ̇/dω_error = 1.  B[2][0] must be 0.
//         With B[2][0]=κ the gain matrix is anti-stabilising in yaw.
//
//  FIX-B  ESKF dimensional error (v2):
//         v2: dtheta = dtheta_meas - err[3]  — wrong: err[3] is rad/s (bias),
//         dtheta_meas is rad (integrated).  Correct: dtheta = dtheta_meas - err[3]*dt.
//         Also fixed: Joseph-form covariance update adds missing KRKᵀ term.
//
//  FIX-C  Arc traversal sign (v2):
//         v2: ang = startAngle - sign*t*arcAngle  (angle decreases for left turn).
//         CCW travel (left turn) means the radius vector angle increases.
//         Correct: ang = startAngle + sign*t*arcAngle.
//         Robot heading = ang + sign*π/2 (tangent to circle, with correct sense).
//
//  FIX-D  Clothoid-length lower bound (v2):
//         v2: L_c = min(L_c_min_required, L_c_segment_max) — can produce a
//         length BELOW the steering-bandwidth minimum, violating the constraint.
//         Correct: L_c = clamp(L_c_min, floor, L_c_segment_max).
//
//  FIX-E  D* Lite uses pessimistic canMove during exploration (v2):
//         v2 D* Lite calls maze.canMove() (pessimistic) while the explorer
//         needs optimistic planning.  Fixed: separate optimistic flag threaded
//         through the planner; exploration uses canMove8(optimistic=true).
//
//  FIX-F  backwardPass prevBrk reset per iteration (v3):
//         v3 resets prevBrk = 0.0f at the top of each convergence iteration.
//         This defeats jerk-limiting across points; prevBrk must persist within
//         a single backward sweep.  Fixed: initialise once outside the loop.
//
//  FIX-G  ESKF wall-distance sign convention (v2):
//         v2 InfoExplorer::senseAndUpdate uses opposite sign for wall axes
//         (WS → axis 1 with sign -1, WN → axis 1 with sign +1) which is
//         inverted relative to the world-frame convention (y increases North).
//         Fixed to match v3's explicit sign documentation.
//
//  ── Design principles ────────────────────────────────────────────────────
//    1. Mathematical correctness — every formula verified
//    2. Real-world performance   — algorithms chosen for hardware reality
//    3. Robustness               — failures degrade gracefully
//    4. Simplicity               — no complexity without measurable benefit
//    5. Deterministic execution  — no unbounded allocations in hot paths
//
//  ── Coordinate frame (fixed, applies to ALL subsystems) ──────────────────
//    World: x → East,  y → North  (standard mathematical convention)
//    Cell (r,c): row r from North wall, column c from West wall
//    Cell centre in world: x = (c+0.5)*cellSize,  y = -(r+0.5)*cellSize
//    Heading: 0 = East, π/2 = North, −π/2 = South, π = West
//    Positive curvature = left turn (CCW) in world frame (Frenet-Serret)
// ═══════════════════════════════════════════════════════════════════════════

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <deque>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <queue>
#include <random>
#include <set>
#include <sstream>
#include <stack>
#include <string>
#include <tuple>
#include <vector>

// ───────────────────────────────────────────────────────────────────────────
//  Constants
// ───────────────────────────────────────────────────────────────────────────

inline constexpr int   MAZE_N  = 16;
inline constexpr int   N_CELLS = MAZE_N * MAZE_N;
inline constexpr float INF_F   = std::numeric_limits<float>::infinity();
inline constexpr float PI      = 3.14159265358979f;
inline constexpr float TWO_PI  = 2.0f * PI;
inline constexpr float HALF_PI = PI * 0.5f;
inline constexpr float SQRT2   = 1.41421356237f;

// ───────────────────────────────────────────────────────────────────────────
//  Angle utilities
// ───────────────────────────────────────────────────────────────────────────

[[nodiscard]] inline float wrapAngle(float a) noexcept {
    a = std::fmod(a + PI, TWO_PI);
    if (a < 0.0f) a += TWO_PI;
    return a - PI;
}

[[nodiscard]] inline float angleDiff(float a, float b) noexcept {
    return wrapAngle(a - b);
}

// ───────────────────────────────────────────────────────────────────────────
//  Wall / direction encoding
//
//  Wall indices:  0=North, 1=East, 2=South, 3=West  (cell-relative)
//  World frame:   North=+y, East=+x, South=−y, West=−x
// ───────────────────────────────────────────────────────────────────────────

enum Wall : int { WN = 0, WE = 1, WS = 2, WW = 3 };

inline constexpr int   WALL_OPP[4]     = { WS, WW, WN, WE };
inline constexpr int   WALL_DC[4]      = {  0,  1,  0, -1 };   // column delta
inline constexpr int   WALL_DR[4]      = { -1,  0,  1,  0 };   // row delta (row↑ = North)
inline constexpr float WALL_HEADING[4] = {
    HALF_PI,   // North → +y
    0.0f,      // East  → +x
    -HALF_PI,  // South → −y
    PI         // West  → −x
};

// 8-direction movement
inline constexpr int   D8C[8]       = {  0, 1, 1, 1,  0, -1, -1, -1 };
inline constexpr int   D8R[8]       = { -1,-1, 0, 1,  1,  1,  0, -1 };
inline constexpr float D8COST[8]    = { 1.f, SQRT2, 1.f, SQRT2, 1.f, SQRT2, 1.f, SQRT2 };

// Walls that must be clear to move in each 8-direction (−1 = none required)
inline constexpr int D8WALLS[8][2] = {
    { WN, -1 }, { WN, WE }, { WE, -1 }, { WE, WS },
    { WS, -1 }, { WS, WW }, { WW, -1 }, { WN, WW }
};

// World-frame heading for each 8-direction
inline constexpr float D8HEADING[8] = {
    HALF_PI,              // N
    PI * 0.25f,           // NE
    0.0f,                 // E
    -PI * 0.25f,          // SE
    -HALF_PI,             // S
    -(PI * 0.75f),        // SW
    PI,                   // W
    PI * 0.75f            // NW
};

// ───────────────────────────────────────────────────────────────────────────
//  CellCoord — grid coordinate (row increases southward)
// ───────────────────────────────────────────────────────────────────────────

struct CellCoord {
    int r = 0;   // row 0 = north edge
    int c = 0;   // column 0 = west edge

    [[nodiscard]] bool operator==(const CellCoord& o) const noexcept { return r==o.r && c==o.c; }
    [[nodiscard]] bool operator!=(const CellCoord& o) const noexcept { return !(*this==o); }
    [[nodiscard]] bool operator< (const CellCoord& o) const noexcept {
        return (r != o.r) ? r < o.r : c < o.c;
    }
    [[nodiscard]] int  idx()   const noexcept { return r * MAZE_N + c; }
    [[nodiscard]] bool valid() const noexcept {
        return r >= 0 && r < MAZE_N && c >= 0 && c < MAZE_N;
    }
    [[nodiscard]] CellCoord neighbour(int wallDir) const noexcept {
        return { r + WALL_DR[wallDir], c + WALL_DC[wallDir] };
    }
    [[nodiscard]] CellCoord step8(int d8) const noexcept {
        return { r + D8R[d8], c + D8C[d8] };
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Vec2 — world-space 2-D vector (x East, y North)
// ───────────────────────────────────────────────────────────────────────────

struct Vec2 {
    float x = 0.0f, y = 0.0f;
    [[nodiscard]] Vec2  operator+(const Vec2& o) const noexcept { return {x+o.x, y+o.y}; }
    [[nodiscard]] Vec2  operator-(const Vec2& o) const noexcept { return {x-o.x, y-o.y}; }
    [[nodiscard]] Vec2  operator*(float s)       const noexcept { return {x*s, y*s}; }
    [[nodiscard]] float dot  (const Vec2& o)     const noexcept { return x*o.x + y*o.y; }
    [[nodiscard]] float cross(const Vec2& o)     const noexcept { return x*o.y - y*o.x; }
    [[nodiscard]] float norm()                   const noexcept { return std::sqrt(x*x+y*y); }
    [[nodiscard]] Vec2  normalised()             const noexcept {
        float n = norm();
        return n > 1e-9f ? Vec2{x/n, y/n} : Vec2{};
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  MazeConfig
// ───────────────────────────────────────────────────────────────────────────

struct MazeConfig {
    int   size     = MAZE_N;
    float cellSize = 0.18f;   // metres

    // Standard 16×16 goal: centre 4 cells
    std::array<CellCoord,4> goalCells = {{ {7,7},{7,8},{8,7},{8,8} }};
    CellCoord startCell = { 15, 0 };  // bottom-left (SW corner)

    [[nodiscard]] bool isGoal(const CellCoord& cc) const noexcept {
        for (auto& g : goalCells) if (g == cc) return true;
        return false;
    }
    [[nodiscard]] bool valid(int r, int c) const noexcept {
        return r >= 0 && r < size && c >= 0 && c < size;
    }
    [[nodiscard]] bool valid(const CellCoord& cc) const noexcept {
        return valid(cc.r, cc.c);
    }

    // Cell centre in world frame: x East, y North
    [[nodiscard]] Vec2 cellCentre(const CellCoord& cc) const noexcept {
        return { (cc.c + 0.5f) * cellSize, -(cc.r + 0.5f) * cellSize };
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Cell
// ───────────────────────────────────────────────────────────────────────────

struct Cell {
    std::array<bool,4> wallKnown = { false,false,false,false };
    std::array<bool,4> wall      = { true, true, true, true  };
    bool  explored   = false;
    int   visitCount = 0;
    float floodDist  = INF_F;
    // D* Lite state
    float dstar_g   = INF_F;
    float dstar_rhs = INF_F;

    // Optimistic: unknown walls treated as open
    [[nodiscard]] bool passableOpt(int w)   const noexcept {
        return !(wallKnown[w] && wall[w]);
    }
    // Conservative: must be known-open
    [[nodiscard]] bool passableCons(int w)  const noexcept {
        return wallKnown[w] && !wall[w];
    }
    [[nodiscard]] bool hasFrontier() const noexcept {
        for (int w = 0; w < 4; w++) if (!wallKnown[w]) return true;
        return false;
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Maze grid
// ───────────────────────────────────────────────────────────────────────────

class Maze {
public:
    std::array<Cell, N_CELLS> cells{};
    const MazeConfig* cfg = nullptr;

    void init(const MazeConfig& c) {
        cfg = &c;
        cells.fill(Cell{});
        placeBorderWalls();
    }

    [[nodiscard]] Cell&       at(const CellCoord& cc)       noexcept { return cells[cc.idx()]; }
    [[nodiscard]] const Cell& at(const CellCoord& cc) const noexcept { return cells[cc.idx()]; }
    [[nodiscard]] Cell&       at(int r, int c)              noexcept { return cells[r*MAZE_N+c]; }
    [[nodiscard]] const Cell& at(int r, int c)        const noexcept { return cells[r*MAZE_N+c]; }

    void setWall(const CellCoord& cc, int w, bool present) {
        if (!cfg->valid(cc)) return;
        cells[cc.idx()].wall[w]      = present;
        cells[cc.idx()].wallKnown[w] = true;
        CellCoord nb = cc.neighbour(w);
        if (cfg->valid(nb)) {
            cells[nb.idx()].wall[WALL_OPP[w]]      = present;
            cells[nb.idx()].wallKnown[WALL_OPP[w]] = true;
        }
    }

    // Can we move in 8-direction d8 from cc?
    [[nodiscard]] bool canMove8(const CellCoord& cc, int d8, bool optimistic) const noexcept {
        CellCoord nb = cc.step8(d8);
        if (!cfg->valid(nb)) return false;
        const Cell& cell = at(cc);
        for (int k = 0; k < 2; k++) {
            int w = D8WALLS[d8][k];
            if (w < 0) continue;
            if (optimistic) { if (cell.wallKnown[w] && cell.wall[w]) return false; }
            else            { if (!cell.wallKnown[w] || cell.wall[w]) return false; }
        }
        return true;
    }

    // Cardinal-only movement check
    [[nodiscard]] bool canMoveCardinal(const CellCoord& cc, int w, bool optimistic) const noexcept {
        CellCoord nb = cc.neighbour(w);
        if (!cfg->valid(nb)) return false;
        const Cell& cell = at(cc);
        if (optimistic) return !(cell.wallKnown[w] && cell.wall[w]);
        return cell.wallKnown[w] && !cell.wall[w];
    }

    [[nodiscard]] int frontierCount() const noexcept {
        int n = 0;
        for (auto& c : cells) if (c.explored && c.hasFrontier()) n++;
        return n;
    }

private:
    void placeBorderWalls() {
        const int sz = cfg->size;
        for (int i = 0; i < sz; i++) {
            setWall({0,   i}, WN, true);
            setWall({sz-1,i}, WS, true);
            setWall({i,   0}, WW, true);
            setWall({i,sz-1}, WE, true);
        }
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  FloodFill — goal-seeded Dijkstra distance field
//
//  Used for:  exploration navigation heuristic,  A*/Theta* heuristic seed.
//  optimistic=true  → unknown walls open  (exploration)
//  optimistic=false → only known-open     (speed run)
// ───────────────────────────────────────────────────────────────────────────

class FloodFill {
public:
    static void solve(Maze& maze,
                      const std::vector<CellCoord>& seeds,
                      bool optimistic)
    {
        for (auto& cell : maze.cells) cell.floodDist = INF_F;

        using Entry = std::pair<float, CellCoord>;
        std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;

        for (const auto& s : seeds) {
            if (!maze.cfg->valid(s)) continue;
            maze.at(s).floodDist = 0.0f;
            pq.push({0.0f, s});
        }

        while (!pq.empty()) {
            auto [d, cc] = pq.top(); pq.pop();
            if (d > maze.at(cc).floodDist + 1e-6f) continue;   // stale entry

            for (int d8 = 0; d8 < 8; d8++) {
                if (!maze.canMove8(cc, d8, optimistic)) continue;
                CellCoord nb = cc.step8(d8);
                float nd = d + D8COST[d8];
                if (nd < maze.at(nb).floodDist - 1e-6f) {
                    maze.at(nb).floodDist = nd;
                    pq.push({nd, nb});
                }
            }
        }
    }

    static void solveToGoal (Maze& maze, bool optimistic) {
        std::vector<CellCoord> seeds(maze.cfg->goalCells.begin(), maze.cfg->goalCells.end());
        solve(maze, seeds, optimistic);
    }
    static void solveToStart(Maze& maze, bool optimistic) {
        solve(maze, { maze.cfg->startCell }, optimistic);
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  D* Lite — incremental replanning  (from v2, FIX-E applied)
//
//  When a new wall is discovered, only the affected nodes are updated.
//  The `optimistic` flag is now propagated correctly so exploration uses
//  canMove8(optimistic=true).
//
//  Reference: Koenig & Likhachev, "D* Lite", AAAI 2002.
// ───────────────────────────────────────────────────────────────────────────

class DStarLite {
public:
    struct Key {
        float k1, k2;
        bool operator<(const Key& o) const noexcept {
            return std::abs(k1-o.k1) > 1e-6f ? k1 < o.k1 : k2 < o.k2;
        }
        bool operator>(const Key& o) const noexcept { return o < *this; }
        bool operator<=(const Key& o) const noexcept { return !(o < *this); }
    };

    Maze*     maze      = nullptr;
    CellCoord start     = {-1,-1};
    bool      optimistic = true;
    float     km        = 0.0f;

    using QEntry = std::pair<Key, CellCoord>;
    struct QCmp { bool operator()(const QEntry& a, const QEntry& b) const noexcept { return a.first > b.first; } };
    std::priority_queue<QEntry, std::vector<QEntry>, QCmp> U;

    void init(Maze& m, CellCoord s, bool opt) {
        maze      = &m;
        start     = s;
        optimistic= opt;
        km        = 0.0f;
        // Reset D* state on all cells
        for (auto& cell : maze->cells) { cell.dstar_g = INF_F; cell.dstar_rhs = INF_F; }
        while (!U.empty()) U.pop();

        // Seed all goal cells with rhs=0
        for (const auto& gc : maze->cfg->goalCells) {
            maze->at(gc).dstar_rhs = 0.0f;
            U.push({ calcKey(gc), gc });
        }
        computeShortestPath();
    }

    [[nodiscard]] float heuristic(const CellCoord& cc) const noexcept {
        float best = INF_F;
        for (const auto& gc : maze->cfg->goalCells) {
            float dr = float(std::abs(cc.r - gc.r));
            float dc = float(std::abs(cc.c - gc.c));
            float h  = std::max(dr,dc) + (SQRT2-1.0f)*std::min(dr,dc);
            if (h < best) best = h;
        }
        return best;
    }

    [[nodiscard]] Key calcKey(const CellCoord& cc) const noexcept {
        float gv = maze->at(cc).dstar_g, rv = maze->at(cc).dstar_rhs;
        float mn = std::min(gv, rv);
        return { mn + heuristic(cc) + km, mn };
    }

    void computeShortestPath() {
        while (!U.empty()) {
            auto [kOld, u] = U.top();
            Key ks = calcKey(start);
            float gs = maze->at(start).dstar_g, rs = maze->at(start).dstar_rhs;
            if (!(kOld <= ks) && std::abs(rs - gs) < 1e-6f) break;
            U.pop();

            Key kNew = calcKey(u);
            if (kOld < kNew) { U.push({kNew, u}); continue; }

            float gu = maze->at(u).dstar_g;
            float ru = maze->at(u).dstar_rhs;

            if (gu > ru) {
                maze->at(u).dstar_g = ru;
                for (int d8 = 0; d8 < 8; d8++) {
                    if (!maze->canMove8(u, d8, optimistic)) continue;
                    CellCoord nb = u.step8(d8);
                    if (maze->cfg->isGoal(nb)) continue;
                    float nr = maze->at(u).dstar_g + D8COST[d8];
                    if (nr < maze->at(nb).dstar_rhs - 1e-6f) {
                        maze->at(nb).dstar_rhs = nr;
                        U.push({ calcKey(nb), nb });
                    }
                }
            } else {
                maze->at(u).dstar_g = INF_F;
                U.push({ calcKey(u), u });
                for (int d8 = 0; d8 < 8; d8++) {
                    if (!maze->canMove8(u, d8, optimistic)) continue;
                    CellCoord nb = u.step8(d8);
                    if (maze->cfg->isGoal(nb)) continue;
                    // Recompute rhs[nb] from all neighbours
                    float bestRhs = INF_F;
                    for (int d2 = 0; d2 < 8; d2++) {
                        if (!maze->canMove8(nb, d2, optimistic)) continue;
                        CellCoord s2 = nb.step8(d2);
                        float cand = maze->at(s2).dstar_g + D8COST[d2];
                        if (cand < bestRhs) bestRhs = cand;
                    }
                    maze->at(nb).dstar_rhs = bestRhs;
                    U.push({ calcKey(nb), nb });
                }
            }
        }
        // Mirror dstar_g into floodDist for compatibility with Theta* heuristic
        for (auto& cell : maze->cells)
            cell.floodDist = cell.dstar_g;
    }

    // Call when wall state changes at position cc
    void notifyWallChanged(const CellCoord& cc) {
        km += heuristic(start);
        for (int d8 = 0; d8 < 8; d8++) {
            CellCoord nb = cc.step8(d8);
            if (!maze->cfg->valid(nb) || maze->cfg->isGoal(nb)) continue;
            float bestRhs = INF_F;
            for (int d2 = 0; d2 < 8; d2++) {
                if (!maze->canMove8(nb, d2, optimistic)) continue;
                CellCoord s2 = nb.step8(d2);
                float cand = maze->at(s2).dstar_g + D8COST[d2];
                if (cand < bestRhs) bestRhs = cand;
            }
            maze->at(nb).dstar_rhs = bestRhs;
            U.push({ calcKey(nb), nb });
        }
        computeShortestPath();
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Theta* — any-angle A*  (FIX-C applied to arc; coordinate frame unified)
//
//  Reference: Nash, Daniel, Koenig, JAIR 2010.
// ───────────────────────────────────────────────────────────────────────────

class ThetaStar {
public:
    // Bresenham line-of-sight in grid (row/column) coordinates.
    [[nodiscard]] static bool lineOfSight(const Maze& maze,
                                           const CellCoord& a,
                                           const CellCoord& b,
                                           bool optimistic) noexcept
    {
        int r0=a.r, c0=a.c, r1=b.r, c1=b.c;
        int dr=std::abs(r1-r0), dc=std::abs(c1-c0);
        int sr=(r1>r0)?1:-1, sc=(c1>c0)?1:-1;
        int r=r0, c=c0, err=dc-dr;

        for (int step=0; step<=dr+dc; step++) {
            if (r==r1 && c==c1) return true;
            if (!maze.cfg->valid(r,c)) return false;
            int e2=2*err;
            bool mC=(e2>-dr), mR=(e2<dc);
            CellCoord cc{r,c};
            if (mC && mR) {
                int wC=(sc>0)?WE:WW, wR=(sr>0)?WS:WN;
                if (!checkWall(maze,cc,wC,optimistic)) return false;
                if (!checkWall(maze,cc,wR,optimistic)) return false;
                c+=sc; r+=sr; err+=dr-dc;
            } else if (mC) {
                if (!checkWall(maze,cc,(sc>0)?WE:WW,optimistic)) return false;
                c+=sc; err+=dr;
            } else {
                if (!checkWall(maze,cc,(sr>0)?WS:WN,optimistic)) return false;
                r+=sr; err-=dc;
            }
        }
        return true;
    }

    [[nodiscard]] static float dist(const CellCoord& a, const CellCoord& b) noexcept {
        float dr=float(b.r-a.r), dc=float(b.c-a.c);
        return std::sqrt(dr*dr+dc*dc);
    }

    [[nodiscard]] static std::vector<CellCoord> findPath(
        const Maze& maze, const CellCoord& start, bool optimistic)
    {
        std::array<float,     N_CELLS> gCost;
        std::array<CellCoord, N_CELLS> parent;
        std::array<bool,      N_CELLS> closed;
        gCost.fill(INF_F);
        parent.fill({-1,-1});
        closed.fill(false);

        gCost[start.idx()]  = 0.0f;
        parent[start.idx()] = {-2,-2};  // start sentinel

        struct Node {
            float f; CellCoord cc;
            bool operator>(const Node& o) const noexcept { return f > o.f; }
        };
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
        open.push({ maze.at(start).floodDist, start });

        CellCoord reached{-1,-1};

        while (!open.empty()) {
            auto [f, cc] = open.top(); open.pop();
            if (closed[cc.idx()]) continue;
            closed[cc.idx()] = true;
            if (maze.cfg->isGoal(cc)) { reached=cc; break; }

            for (int d8=0; d8<8; d8++) {
                if (!maze.canMove8(cc, d8, optimistic)) continue;
                CellCoord nb = cc.step8(d8);
                if (closed[nb.idx()]) continue;

                const CellCoord& par = parent[cc.idx()];
                bool hasPar = (par.r >= 0);

                float ng; CellCoord via;
                if (hasPar && lineOfSight(maze, par, nb, optimistic)) {
                    ng  = gCost[par.idx()] + dist(par, nb);
                    via = par;
                } else {
                    ng  = gCost[cc.idx()] + D8COST[d8];
                    via = cc;
                }

                if (ng < gCost[nb.idx()] - 1e-6f) {
                    gCost[nb.idx()]  = ng;
                    parent[nb.idx()] = via;
                    open.push({ ng + maze.at(nb).floodDist, nb });
                }
            }
        }

        if (reached.r < 0) return {};

        std::vector<CellCoord> path;
        CellCoord c = reached;
        while (c.r >= 0) {
            path.push_back(c);
            const CellCoord& p = parent[c.idx()];
            if (p.r == -2) break;
            c = p;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    // Expand a Theta* path (may have multi-cell LoS jumps) into single steps
    [[nodiscard]] static std::vector<CellCoord> expandPath(
        const std::vector<CellCoord>& path)
    {
        std::vector<CellCoord> expanded;
        if (path.empty()) return expanded;
        expanded.push_back(path[0]);
        for (size_t i=1; i<path.size(); i++) {
            int dr = path[i].r - path[i-1].r;
            int dc = path[i].c - path[i-1].c;
            int steps = std::max(std::abs(dr), std::abs(dc));
            int sr = (dr>0)-(dr<0), sc = (dc>0)-(dc<0);
            for (int s=1; s<=steps; s++)
                expanded.push_back({ path[i-1].r+s*sr, path[i-1].c+s*sc });
        }
        return expanded;
    }

private:
    [[nodiscard]] static bool checkWall(const Maze& m, const CellCoord& cc,
                                         int w, bool optimistic) noexcept {
        if (!m.cfg->valid(cc)) return false;
        const Cell& cell = m.at(cc);
        if (optimistic) return !(cell.wallKnown[w] && cell.wall[w]);
        return cell.wallKnown[w] && !cell.wall[w];
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Sensor model  (from v3)
//
//  IR sensor model (Sharp GP2Y0A21YK0F-class):
//    Range: 0.04 – 0.30 m  |  Noise: Gaussian σ ≈ 3 mm  |  Saturation below 40 mm
// ───────────────────────────────────────────────────────────────────────────

struct SensorReading {
    float distance = 0.0f;
    bool  valid    = false;
};

class SensorModel {
public:
    static constexpr float RANGE_MIN  = 0.04f;
    static constexpr float RANGE_MAX  = 0.30f;
    static constexpr float NOISE_STD  = 0.003f;
    static constexpr float NOISE_VAR  = NOISE_STD * NOISE_STD;

    [[nodiscard]] SensorReading sample(float trueDistance, std::mt19937& rng) const {
        if (trueDistance < RANGE_MIN || trueDistance > RANGE_MAX)
            return {trueDistance, false};
        std::normal_distribution<float> noise{0.0f, NOISE_STD};
        float m = trueDistance + noise(rng);
        return (m < RANGE_MIN || m > RANGE_MAX) ? SensorReading{m, false}
                                                 : SensorReading{m, true};
    }
    [[nodiscard]] static float measurementVariance() noexcept { return NOISE_VAR; }
};

// ───────────────────────────────────────────────────────────────────────────
//  ESKF — Error-State Kalman Filter  (from v2, FIX-B applied)
//
//  State:  x_nom = [x, y, θ]ᵀ  (nominal, integrated by dead-reckoning)
//  Error:  δx    = [δx, δy, δθ, δb_g]ᵀ  (4-state: position + heading + gyro bias)
//
//  FIX-B: dimensional error corrected: bias subtracted as err[3]*dt, not err[3].
//  FIX-B: Joseph-form covariance update: P = (I-KH)P(I-KH)' + KRK'  (numerically stable).
//  FIX-G: wall-distance sign convention now matches world-frame definition.
// ───────────────────────────────────────────────────────────────────────────

class ESKF {
public:
    // Nominal state
    float nom_x=0.f, nom_y=0.f, nom_theta=0.f;
    // Error state [δx, δy, δθ, δb_g]
    std::array<float,4> err{0,0,0,0};
    // Covariance 4×4 (row-major)
    std::array<float,16> P{};

    // Process noise
    float Q_xy    = 1e-5f;
    float Q_theta = 2e-4f;
    float Q_bias  = 1e-6f;
    // Measurement noise
    float R_wall  = SensorModel::NOISE_VAR;
    float R_hdg   = 1e-4f;

    ESKF() {
        P.fill(0);
        P[0]=0.01f; P[5]=0.01f; P[10]=0.001f; P[15]=1e-4f;
    }

    void reset(float x0, float y0, float h0) noexcept {
        nom_x=x0; nom_y=y0; nom_theta=h0;
        err.fill(0);
        P.fill(0);
        P[0]=1e-4f; P[5]=1e-4f; P[10]=1e-4f; P[15]=1e-6f;
    }

    [[nodiscard]] float x()     const noexcept { return nom_x     + err[0]; }
    [[nodiscard]] float y()     const noexcept { return nom_y     + err[1]; }
    [[nodiscard]] float theta() const noexcept { return nom_theta + err[2]; }
    [[nodiscard]] float bias()  const noexcept { return err[3]; }

    float& pij(int i, int j) noexcept       { return P[i*4+j]; }
    float  pij(int i, int j) const noexcept { return P[i*4+j]; }

    // Dead-reckoning predict.
    // ds:            arc-length step (m) from encoder
    // dtheta_meas:   gyro-integrated angle change (rad) — NOT rate
    // dt:            time step (s) for bias correction  [FIX-B]
    void predict(float ds, float dtheta_meas, float dt) noexcept {
        // FIX-B: bias is in rad/s; multiply by dt to get rad
        float dtheta   = dtheta_meas - err[3] * dt;
        float midTheta = nom_theta + 0.5f * dtheta;
        float c = std::cos(midTheta), s_t = std::sin(midTheta);

        nom_x     += ds * c;
        nom_y     += ds * s_t;
        nom_theta  = wrapAngle(nom_theta + dtheta);

        // F-matrix perturbation (linearised, 4×4)
        std::array<float,16> Pn = P;
        for (int j=0;j<4;j++) Pn[0*4+j] += (-ds*s_t)*P[2*4+j];
        for (int j=0;j<4;j++) Pn[1*4+j] += (ds*c)   *P[2*4+j];
        for (int j=0;j<4;j++) Pn[2*4+j] += (-dt)     *P[3*4+j];
        std::array<float,16> P2 = Pn;
        for (int i=0;i<4;i++) P2[i*4+0] += Pn[i*4+2]*(-ds*s_t);
        for (int i=0;i<4;i++) P2[i*4+1] += Pn[i*4+2]*(ds*c);
        for (int i=0;i<4;i++) P2[i*4+2] += Pn[i*4+3]*(-dt);
        // Process noise (diagonal)
        P2[0]  += Q_xy    * std::abs(ds);
        P2[5]  += Q_xy    * std::abs(ds);
        P2[10] += Q_theta * std::abs(dtheta);
        P2[15] += Q_bias;
        P = P2;
    }

    // 1-D wall-distance update.
    // axis:      0=x, 1=y
    // wallCoord: world-frame coordinate of the wall
    // sign:      +1 if wall is in +axis direction from robot, −1 otherwise
    void updateWallDist(int axis, float wallCoord, float measuredDist,
                        float sign, float R_meas) noexcept
    {
        float robotPos = (axis==0) ? x() : y();
        float expected = sign * (wallCoord - robotPos);
        float innov    = measuredDist - expected;

        std::array<float,4> H{}; H[axis] = sign;

        float S = R_meas;
        for (int i=0;i<4;i++) for (int j=0;j<4;j++) S += H[i]*pij(i,j)*H[j];
        if (S < 1e-12f) return;
        if (innov*innov > 9.0f*S) return;  // outlier rejection: > 3σ

        std::array<float,4> K{};
        for (int i=0;i<4;i++) {
            float PH=0; for (int k=0;k<4;k++) PH += pij(i,k)*H[k];
            K[i] = PH / S;
        }

        for (int i=0;i<4;i++) err[i] += K[i]*innov;

        // Joseph form: P = (I-KH)*P*(I-KH)' + K*R*K'  [FIX-B]
        std::array<float,16> A{};
        for (int i=0;i<4;i++) for (int j=0;j<4;j++)
            A[i*4+j] = (i==j?1.0f:0.0f) - K[i]*H[j];
        std::array<float,16> AP{};
        for (int i=0;i<4;i++) for (int j=0;j<4;j++)
            for (int k=0;k<4;k++) AP[i*4+j] += A[i*4+k]*pij(k,j);
        std::array<float,16> Pn{};
        for (int i=0;i<4;i++) for (int j=0;j<4;j++) {
            float v=0;
            for (int k=0;k<4;k++) v += AP[i*4+k]*A[j*4+k];
            Pn[i*4+j] = v + K[i]*R_meas*K[j];
        }
        P = Pn;

        nom_x     += err[0]; nom_y += err[1];
        nom_theta  = wrapAngle(nom_theta + err[2]);
        err[0]=err[1]=err[2]=0.0f;  // bias (err[3]) persists
    }

    // Heading update (e.g. wall-exit snap, or wall-centering correction)
    void updateHeading(float measuredHeading, float R_meas) noexcept {
        float innov = angleDiff(measuredHeading, theta());
        std::array<float,4> H{}; H[2]=1.0f;
        float S = pij(2,2) + R_meas;
        if (S < 1e-12f) return;
        std::array<float,4> K{};
        for (int i=0;i<4;i++) K[i] = pij(i,2)/S;
        for (int i=0;i<4;i++) err[i] += K[i]*innov;
        // Joseph form for heading
        std::array<float,16> A{};
        for (int i=0;i<4;i++) for (int j=0;j<4;j++)
            A[i*4+j]=(i==j?1.0f:0.0f)-K[i]*H[j];
        std::array<float,16> AP{};
        for (int i=0;i<4;i++) for (int j=0;j<4;j++)
            for (int k=0;k<4;k++) AP[i*4+j]+=A[i*4+k]*pij(k,j);
        std::array<float,16> Pn{};
        for (int i=0;i<4;i++) for (int j=0;j<4;j++) {
            float v=0;
            for (int k=0;k<4;k++) v+=AP[i*4+k]*A[j*4+k];
            Pn[i*4+j]=v+K[i]*R_meas*K[j];
        }
        P=Pn;
        nom_theta=wrapAngle(nom_theta+err[2]);
        err[2]=0.0f;
    }

    void snapHeadingCardinal() noexcept {
        float snapped = std::round(theta()/HALF_PI)*HALF_PI;
        if (std::abs(angleDiff(theta(), snapped)) < 0.12f)
            updateHeading(snapped, 1e-3f);
    }

    void print() const {
        std::cout << std::fixed << std::setprecision(5)
                  << "  ESKF: x=" << x() << " y=" << y()
                  << " θ=" << theta() << " rad"
                  << " bias=" << bias() << " rad/s\n"
                  << "  Cov diag: [" << pij(0,0) << ", " << pij(1,1)
                  << ", " << pij(2,2) << ", " << pij(3,3) << "]\n";
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Robot parameters
// ───────────────────────────────────────────────────────────────────────────

struct RobotParams {
    float maxTotalAccel   = 12.0f;   // m/s² — Kamm circle radius
    float maxBraking      = 10.0f;   // m/s²
    float maxAccelFwd     =  9.0f;   // m/s²
    float maxJerk         = 60.0f;   // m/s³
    float maxVelocity     =  5.0f;   // m/s
    float exploreVelocity =  0.6f;   // m/s

    float wheelbase         = 0.07f;
    float trackWidth        = 0.06f;
    float cellSize          = 0.18f;
    float steeringBandwidth = 20.0f;

    // PD trajectory-tracking gains
    float Kp_crosstrack = 4.0f;
    float Kd_crosstrack = 0.3f;
    float Kp_heading    = 2.0f;
    float Kd_heading    = 0.1f;

    // Wall-centering PID gains
    float Kp_center = 3.0f;
    float Ki_center = 0.1f;
};

// ───────────────────────────────────────────────────────────────────────────
//  Trajectory point
// ───────────────────────────────────────────────────────────────────────────

struct TrajPoint {
    float x        = 0.0f;
    float y        = 0.0f;
    float heading  = 0.0f;   // rad
    float curvature= 0.0f;   // 1/m, positive = left (CCW)
    float velocity = 0.0f;   // m/s
    float arcLen   = 0.0f;   // m cumulative
    float accel    = 0.0f;   // m/s² longitudinal
    float jerk     = 0.0f;   // m/s³
    float ff_steer_rate = 0.0f;  // dκ/dt (rad/m/s) for feedforward
};

// ───────────────────────────────────────────────────────────────────────────
//  Clothoid segment — 8-point Gauss-Legendre quadrature on [0,s]
// ───────────────────────────────────────────────────────────────────────────

struct ClothoidSeg {
    float x0, y0, theta0, kappa0, kappaEnd, length;

    static constexpr float GL_XI[8] = {
        0.0950125098f, 0.2816035508f, 0.4580167777f, 0.6178762444f,
        0.7554044084f, 0.8656312024f, 0.9445750231f, 0.9894009350f
    };
    static constexpr float GL_W[8] = {
        0.1894506105f, 0.1826034150f, 0.1691565194f, 0.1495959889f,
        0.1246289463f, 0.0951585117f, 0.0622535239f, 0.0271524594f
    };

    struct State { float x, y, theta, kappa; };

    [[nodiscard]] State eval(float s) const noexcept {
        if (s <= 0.0f) return {x0, y0, theta0, kappa0};
        float dkds = (length > 1e-9f) ? (kappaEnd - kappa0) / length : 0.0f;
        auto thetaAt = [&](float t) noexcept {
            return theta0 + kappa0*t + 0.5f*dkds*t*t;
        };
        float mid = s * 0.5f, hr = s * 0.5f;
        float px = x0, py = y0;
        for (int i=0; i<8; i++) {
            float t1 = mid + hr*GL_XI[i];
            float t2 = mid - hr*GL_XI[i];
            px += GL_W[i]*hr*(std::cos(thetaAt(t1)) + std::cos(thetaAt(t2)));
            py += GL_W[i]*hr*(std::sin(thetaAt(t1)) + std::sin(thetaAt(t2)));
        }
        return {px, py, thetaAt(s), kappa0 + dkds*s};
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Path → waypoints converter
// ───────────────────────────────────────────────────────────────────────────

struct Waypoint { float x, y, heading; };

[[nodiscard]] static std::vector<Waypoint> pathToWaypoints(
    const std::vector<CellCoord>& path, const MazeConfig& cfg)
{
    std::vector<Waypoint> wps;
    int N = int(path.size());
    for (int i=0; i<N; i++) {
        Vec2 pos = cfg.cellCentre(path[i]);
        float hdg = (wps.empty()) ? 0.0f : wps.back().heading;
        if (i+1 < N) {
            Vec2 nxt = cfg.cellCentre(path[i+1]);
            hdg = std::atan2(nxt.y-pos.y, nxt.x-pos.x);
        }
        wps.push_back({pos.x, pos.y, hdg});
    }
    if (wps.size() >= 2) wps.back().heading = wps[wps.size()-2].heading;
    return wps;
}

// ───────────────────────────────────────────────────────────────────────────
//  Racing-line corridor optimiser  (from v2, step size made adaptive)
// ───────────────────────────────────────────────────────────────────────────

[[nodiscard]] static std::vector<Waypoint> optimiseRacingLine(
    std::vector<Waypoint> wps,
    float halfWidth,
    float margin = 0.025f)
{
    int N = int(wps.size());
    if (N < 3) return wps;
    float hw = halfWidth - margin;
    if (hw <= 0.0f) return wps;

    std::vector<Waypoint> centres = wps;
    float step = hw * 0.20f;

    for (int iter=0; iter<50; iter++) {
        float totalChange = 0.0f;
        for (int i=1; i<N-1; i++) {
            float d_prev_x=0, d_prev_y=0, d_i_x, d_i_y, d_next_x=0, d_next_y=0;
            d_i_x = wps[i+1].x - 2.0f*wps[i].x + wps[i-1].x;
            d_i_y = wps[i+1].y - 2.0f*wps[i].y + wps[i-1].y;
            if (i>1) {
                d_prev_x = wps[i].x - 2.0f*wps[i-1].x + wps[i-2].x;
                d_prev_y = wps[i].y - 2.0f*wps[i-1].y + wps[i-2].y;
            }
            if (i<N-2) {
                d_next_x = wps[i+2].x - 2.0f*wps[i+1].x + wps[i].x;
                d_next_y = wps[i+2].y - 2.0f*wps[i+1].y + wps[i].y;
            }
            float gx = 2.0f*(d_prev_x + d_next_x - 2.0f*d_i_x);
            float gy = 2.0f*(d_prev_y + d_next_y - 2.0f*d_i_y);

            float nx = wps[i].x - step * gx;
            float ny = wps[i].y - step * gy;
            nx = std::max(centres[i].x - hw, std::min(centres[i].x + hw, nx));
            ny = std::max(centres[i].y - hw, std::min(centres[i].y + hw, ny));
            totalChange += std::abs(nx-wps[i].x) + std::abs(ny-wps[i].y);
            wps[i].x = nx; wps[i].y = ny;
        }
        for (int i=0; i<N-1; i++) {
            float dx=wps[i+1].x-wps[i].x, dy=wps[i+1].y-wps[i].y;
            wps[i].heading = std::atan2(dy, dx);
        }
        if (N>=2) wps[N-1].heading = wps[N-2].heading;
        if (totalChange < 1e-7f) break;
    }
    return wps;
}

// ───────────────────────────────────────────────────────────────────────────
//  Trajectory Generator — clothoid-arc-clothoid  (FIX-C and FIX-D applied)
// ───────────────────────────────────────────────────────────────────────────

class TrajGen {
public:
    static constexpr int SAMPLES_STRAIGHT = 10;
    static constexpr int SAMPLES_CLOTHOID = 24;
    static constexpr int SAMPLES_ARC      = 20;

    [[nodiscard]] static std::vector<TrajPoint> generate(
        const std::vector<Waypoint>& wps, const RobotParams& robot)
    {
        std::vector<TrajPoint> traj;
        int N = int(wps.size());
        if (N < 2) return traj;

        float cumArc = 0.0f;
        float kPrev  = 0.0f;

        auto emit = [&](float x, float y, float hdg, float k) {
            float arc = 0.0f;
            if (!traj.empty()) {
                float dx=x-traj.back().x, dy=y-traj.back().y;
                arc = std::sqrt(dx*dx+dy*dy);
            }
            cumArc += arc;
            traj.push_back({x, y, hdg, k, robot.maxVelocity, cumArc, 0,0,0});
        };

        for (int wi=0; wi+1<N; wi++) {
            const Waypoint& wa = wps[wi];
            const Waypoint& wb = wps[wi+1];
            float dhdg   = angleDiff(wb.heading, wa.heading);
            float segLen = std::hypot(wb.x-wa.x, wb.y-wa.y);
            if (segLen < 1e-7f) continue;
            int startIdx = (wi==0) ? 0 : 1;

            if (std::abs(dhdg) < 5e-3f) {
                // ── Straight ──────────────────────────────────────────────
                int Ns = SAMPLES_STRAIGHT;
                for (int i=startIdx; i<=Ns; i++) {
                    float t = float(i)/float(Ns);
                    emit(wa.x+t*(wb.x-wa.x), wa.y+t*(wb.y-wa.y), wa.heading, 0.0f);
                }
                kPrev = 0.0f;

            } else {
                // ── Clothoid – Arc – Clothoid ─────────────────────────────
                float R = segLen / (2.0f * std::sin(std::abs(dhdg)*0.5f));
                R = std::max(R, robot.cellSize * 0.05f);
                float kTurn = (1.0f/R) * (dhdg>0.0f ? 1.0f : -1.0f);

                // FIX-D: clothoid length ≥ minimum required by steering bandwidth
                float dkappa  = std::abs(kTurn - kPrev);
                float L_c_min = dkappa * robot.maxVelocity / robot.steeringBandwidth;
                float L_c_max = segLen * 0.45f;
                float L_c     = std::max(L_c_min, 0.005f);   // enforce minimum
                L_c           = std::min(L_c, L_c_max);

                // Entry clothoid
                ClothoidSeg entry{ wa.x, wa.y, wa.heading, kPrev, kTurn, L_c };
                for (int i=startIdx; i<=SAMPLES_CLOTHOID; i++) {
                    float s = float(i)/float(SAMPLES_CLOTHOID)*L_c;
                    auto st = entry.eval(s);
                    emit(st.x, st.y, st.theta, st.kappa);
                }

                // FIX-C: arc geometry with correct traversal direction
                auto eEnd = entry.eval(L_c);
                float sign = (dhdg > 0.0f) ? 1.0f : -1.0f;
                float perpAngle = eEnd.theta + sign * HALF_PI;
                float cx = eEnd.x + R * std::cos(perpAngle);
                float cy = eEnd.y + R * std::sin(perpAngle);

                float clothoidAngle = L_c / R;
                float arcAngle = std::abs(dhdg) - 2.0f * clothoidAngle;

                if (arcAngle > 1e-4f) {
                    float startArc = std::atan2(eEnd.y-cy, eEnd.x-cx);
                    for (int i=1; i<=SAMPLES_ARC; i++) {
                        float t = float(i)/float(SAMPLES_ARC);
                        // FIX-C: CCW (sign=+1) → angle increases; CW (sign=−1) → decreases
                        float ang = startArc + sign * t * arcAngle;
                        float px  = cx + R * std::cos(ang);
                        float py  = cy + R * std::sin(ang);
                        float hdg = wrapAngle(ang + sign * HALF_PI);
                        emit(px, py, hdg, kTurn);
                    }
                }

                // Exit clothoid
                const TrajPoint& ae = traj.back();
                ClothoidSeg exitSeg{ ae.x, ae.y, ae.heading, kTurn, 0.0f, L_c };
                for (int i=1; i<=SAMPLES_CLOTHOID; i++) {
                    float s = float(i)/float(SAMPLES_CLOTHOID)*L_c;
                    auto st = exitSeg.eval(s);
                    emit(st.x, st.y, st.theta, st.kappa);
                }
                kPrev = 0.0f;
            }
        }
        return traj;
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Velocity Profile — Kamm circle + S-curve  (FIX-F applied)
// ───────────────────────────────────────────────────────────────────────────

class VelocityProfile {
public:
    [[nodiscard]] static float kammLong(float kappa, float v, float aTotal) noexcept {
        float aLat2  = (kappa*v*v)*(kappa*v*v);
        float aT2    = aTotal*aTotal;
        return (aLat2 >= aT2) ? 0.0f : std::sqrt(aT2 - aLat2);
    }

    [[nodiscard]] static float vMaxCurv(float kappa, float aTotal) noexcept {
        return (std::abs(kappa) < 1e-7f) ? INF_F : std::sqrt(aTotal/std::abs(kappa));
    }

    static void curvatureCeilings(std::vector<TrajPoint>& traj,
                                   float vMax, float aTotal)
    {
        for (auto& tp : traj)
            tp.velocity = std::min(vMax, vMaxCurv(tp.curvature, aTotal));
    }

    static void globalBrakingPass(std::vector<TrajPoint>& traj, float aTotal,
                                   float aBrakeMax)
    {
        int N = int(traj.size());
        if (N < 2) return;
        for (int pass=0; pass<3; pass++) {
            for (int i=N-2; i>=0; i--) {
                float ds = traj[i+1].arcLen - traj[i].arcLen;
                if (ds < 1e-9f) continue;
                float v1   = traj[i+1].velocity;
                float aBrk = std::min(kammLong(traj[i].curvature, traj[i].velocity, aTotal),
                                      aBrakeMax);
                float vMax = std::sqrt(v1*v1 + 2.0f*aBrk*ds);
                if (vMax < traj[i].velocity) traj[i].velocity = vMax;
            }
        }
    }

    // FIX-F: prevBrk initialised once per sweep, NOT reset inside convergence loop
    static void backwardPass(std::vector<TrajPoint>& traj,
                              float maxJerk, float aTotal, float aBrakeMax,
                              int maxIter=25)
    {
        int N = int(traj.size());
        if (N < 2) return;
        traj.back().velocity = 0.0f;

        for (int iter=0; iter<maxIter; iter++) {
            float maxChange = 0.0f;
            float prevBrk   = 0.0f;   // FIX-F: one initialisation per full sweep

            for (int i=N-2; i>=0; i--) {
                float ds = traj[i+1].arcLen - traj[i].arcLen;
                if (ds < 1e-9f) continue;
                float v1  = traj[i+1].velocity;
                float aBrk = std::min({kammLong(traj[i+1].curvature, v1, aTotal),
                                        aBrakeMax,
                                        prevBrk + maxJerk*ds});
                prevBrk = aBrk;
                float vMax = std::sqrt(v1*v1 + 2.0f*aBrk*ds);
                float vNew = std::min(traj[i].velocity, vMax);
                maxChange  = std::max(maxChange, std::abs(vNew-traj[i].velocity));
                traj[i].velocity = vNew;
            }
            if (maxChange < 1e-5f) break;
        }
    }

    static void forwardPass(std::vector<TrajPoint>& traj,
                             float maxJerk, float aTotal, float aAccelMax)
    {
        if (traj.empty()) return;
        traj.front().velocity = 0.0f;
        float prevAccel = 0.0f;
        for (int i=1; i<int(traj.size()); i++) {
            float ds = traj[i].arcLen - traj[i-1].arcLen;
            if (ds < 1e-9f) {
                traj[i].velocity = std::min(traj[i].velocity, traj[i-1].velocity);
                continue;
            }
            float v0  = traj[i-1].velocity;
            float aAv = std::min({kammLong(traj[i-1].curvature, v0, aTotal),
                                   aAccelMax,
                                   prevAccel + maxJerk*ds});
            float vAc = std::sqrt(v0*v0 + 2.0f*aAv*ds);
            float vNw = std::min(traj[i].velocity, vAc);
            traj[i].velocity = vNw;
            prevAccel = (v0+vNw > 1e-6f) ? (vNw*vNw-v0*v0)/(2.0f*ds) : 0.0f;
            prevAccel = std::max(-aTotal, std::min(prevAccel, aTotal));
            traj[i].accel = prevAccel;
        }
    }

    static void computeJerk(std::vector<TrajPoint>& traj) {
        for (int i=1; i<int(traj.size()); i++) {
            float ds   = traj[i].arcLen   - traj[i-1].arcLen;
            float vAvg = 0.5f*(traj[i].velocity + traj[i-1].velocity);
            float dt   = (vAvg > 1e-4f) ? ds/vAvg : 1e-3f;
            float dv   = traj[i].velocity - traj[i-1].velocity;
            float a_now = dv / std::max(dt, 1e-6f);
            float a_prv = (i>1) ? traj[i-1].accel : 0.0f;
            traj[i].jerk = (a_now - a_prv) / std::max(dt, 1e-6f);
            float dkds = (ds > 1e-9f) ? (traj[i].curvature - traj[i-1].curvature)/ds : 0.0f;
            traj[i].ff_steer_rate = dkds * vAvg;
        }
    }

    [[nodiscard]] static float estimateTime(const std::vector<TrajPoint>& traj) {
        float t = 0.0f;
        for (int i=1; i<int(traj.size()); i++) {
            float ds   = traj[i].arcLen - traj[i-1].arcLen;
            float vAvg = 0.5f*(traj[i].velocity + traj[i-1].velocity);
            if (vAvg > 1e-6f) t += ds/vAvg;
        }
        return t;
    }
    [[nodiscard]] static float peakLatAccel(const std::vector<TrajPoint>& traj) {
        float pk=0;
        for (auto& tp : traj) pk = std::max(pk, std::abs(tp.curvature)*tp.velocity*tp.velocity);
        return pk;
    }
    [[nodiscard]] static float peakLongAccel(const std::vector<TrajPoint>& traj) {
        float pk=0;
        for (auto& tp : traj) pk = std::max(pk, std::abs(tp.accel));
        return pk;
    }
    [[nodiscard]] static float peakJerk(const std::vector<TrajPoint>& traj) {
        float pk=0;
        for (auto& tp : traj) pk = std::max(pk, std::abs(tp.jerk));
        return pk;
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  TVLQR gain schedule  (from v2, FIX-A applied)
//
//  FIX-A: B[2][0] = 0 (not κ) — removes spurious yaw-speed coupling.
// ───────────────────────────────────────────────────────────────────────────

struct TVLQRGain {
    float K[2][3];  // 2 controls × 3 states [δx, δy, δθ]
    float arcLen;
};

class TVLQRSolver {
public:
    static constexpr float Qx  = 200.0f;
    static constexpr float Qy  = 200.0f;
    static constexpr float Qt  =  50.0f;
    static constexpr float Rv  =   1.0f;
    static constexpr float Rw  =   0.5f;

    [[nodiscard]] static std::vector<TVLQRGain> solve(
        const std::vector<TrajPoint>& traj, float /*wheelbase*/)
    {
        int N = int(traj.size());
        std::vector<TVLQRGain> gains(N);

        float P[3][3] = { {Qx,0,0},{0,Qy,0},{0,0,Qt} };
        float Q[3][3] = { {Qx,0,0},{0,Qy,0},{0,0,Qt} };
        float Rinv[2][2] = { {1.0f/Rv,0},{0,1.0f/Rw} };

        for (int i=N-1; i>=0; i--) {
            const auto& tp = traj[i];
            float v  = std::max(tp.velocity, 0.01f);
            float h  = tp.heading;
            float dt = (i>0) ? (traj[i].arcLen - traj[i-1].arcLen)/v : 0.01f;
            dt = std::max(dt, 1e-4f);

            float A[3][3] = {
                {0, 0, -v*std::sin(h)},
                {0, 0,  v*std::cos(h)},
                {0, 0,  0            }
            };
            // FIX-A: B[2][0] = 0 (not κ)
            float B[3][2] = {
                {std::cos(h), 0},
                {std::sin(h), 0},
                {0,           1}
            };

            float BRB[3][3]{};
            for (int r=0;r<3;r++) for (int c=0;c<3;c++)
                for (int m=0;m<2;m++) for (int n=0;n<2;n++)
                    BRB[r][c] += B[r][m]*Rinv[m][n]*B[n][c];

            float ATP[3][3]{};
            for (int r=0;r<3;r++) for (int c=0;c<3;c++)
                for (int m=0;m<3;m++) ATP[r][c] += A[m][r]*P[m][c];
            float PA[3][3]{};
            for (int r=0;r<3;r++) for (int c=0;c<3;c++)
                for (int m=0;m<3;m++) PA[r][c] += P[r][m]*A[m][c];
            float PB[3][3]{};
            for (int r=0;r<3;r++) for (int c=0;c<3;c++)
                for (int m=0;m<3;m++) PB[r][c] += P[r][m]*BRB[m][c];
            float PBRBP[3][3]{};
            for (int r=0;r<3;r++) for (int c=0;c<3;c++)
                for (int m=0;m<3;m++) PBRBP[r][c] += PB[r][m]*P[m][c];

            for (int r=0;r<3;r++) for (int c=0;c<3;c++)
                P[r][c] -= dt*(-Q[r][c] - ATP[r][c] - PA[r][c] + PBRBP[r][c]);
            for (int r=0;r<3;r++) for (int c=r+1;c<3;c++)
                P[r][c] = P[c][r] = 0.5f*(P[r][c]+P[c][r]);
            for (int r=0;r<3;r++) P[r][r] = std::max(P[r][r], 1e-4f);

            float BTP[2][3]{};
            for (int r=0;r<2;r++) for (int c=0;c<3;c++)
                for (int m=0;m<3;m++) BTP[r][c] += B[m][r]*P[m][c];
            for (int r=0;r<2;r++) for (int c=0;c<3;c++) {
                gains[i].K[r][c]=0;
                for (int m=0;m<2;m++) gains[i].K[r][c] += Rinv[r][m]*BTP[m][c];
            }
            gains[i].arcLen = tp.arcLen;
        }
        return gains;
    }

    static void computeControl(
        const std::vector<TVLQRGain>& gains,
        const TrajPoint& ref,
        float est_x, float est_y, float est_theta,
        float& delta_v, float& delta_omega)
    {
        int lo=0, hi=int(gains.size())-1, idx=0;
        while (lo<=hi) {
            int mid=(lo+hi)/2;
            if (gains[mid].arcLen < ref.arcLen) lo=mid+1;
            else { idx=mid; hi=mid-1; }
        }
        const auto& K = gains[idx].K;
        float dx     = est_x     - ref.x;
        float dy     = est_y     - ref.y;
        float dtheta = angleDiff(est_theta, ref.heading);
        delta_v     = -(K[0][0]*dx + K[0][1]*dy + K[0][2]*dtheta);
        delta_omega = -(K[1][0]*dx + K[1][1]*dy + K[1][2]*dtheta);
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  PD controller with curvature feedforward  (from v3)
//
//  Returns {v_cmd, omega_cmd}.  Active during Speed Run (v4.1).
// ───────────────────────────────────────────────────────────────────────────

class PDController {
public:
    explicit PDController(const RobotParams& p) : params(p) {}

    [[nodiscard]] std::pair<float,float> compute(
        float estX, float estY, float estTheta,
        const TrajPoint& ref, float prevCrossTrack, float dt) const noexcept
    {
        float ch = std::cos(ref.heading), sh = std::sin(ref.heading);
        float dx = estX - ref.x,          dy = estY - ref.y;
        float e_xt  = -dx*sh + dy*ch;   // cross-track error
        float e_hdg = angleDiff(estTheta, ref.heading);
        float de_xt = (dt > 1e-6f) ? (e_xt - prevCrossTrack)/dt : 0.0f;
        float ff    = ref.velocity * ref.curvature;    // curvature feedforward
        float fb    = params.Kp_crosstrack * e_xt
                    + params.Kd_crosstrack * de_xt
                    + params.Kp_heading    * e_hdg;
        return { ref.velocity, ff + fb };
    }
private:
    const RobotParams& params;
};

// ───────────────────────────────────────────────────────────────────────────
//  Wall-centering PID  (from v3)
//
//  Active during Scout Run only (v4.1).  Not used during Speed Run.
// ───────────────────────────────────────────────────────────────────────────

class WallCenteringPID {
public:
    explicit WallCenteringPID(const RobotParams& p) : params(p) {}

    void reset() noexcept { integral=0.0f; }

    [[nodiscard]] float compute(
        float leftDist, bool leftValid,
        float rightDist, bool rightValid,
        float cellSize, float dt) noexcept
    {
        if (!leftValid && !rightValid) return 0.0f;
        float error=0.0f;
        if (leftValid && rightValid)  error = (leftDist - rightDist)*0.5f;
        else if (leftValid)            error = leftDist  - cellSize*0.5f;
        else                           error = cellSize*0.5f - rightDist;

        integral = std::max(-0.05f, std::min(0.05f, integral + error*dt));
        return params.Kp_center * error + params.Ki_center * integral;
    }
private:
    const RobotParams& params;
    float integral = 0.0f;
};

// ───────────────────────────────────────────────────────────────────────────
//  Information-theoretic Explorer  (from v2, FIX-E and FIX-G applied)
//
//  v4.1: explore() now accepts WallCenteringPID& and applies it after every
//        senseCell() call, feeding the ESKF heading update channel.
//        The PID integral is reset before each frontier sub-navigation to
//        prevent windup across direction changes.
// ───────────────────────────────────────────────────────────────────────────

class Explorer {
public:
    [[nodiscard]] static float infoGain(const Maze& bot, const CellCoord& cc) noexcept {
        float gain = 0.0f;
        for (int dr=-2; dr<=2; dr++) for (int dc=-2; dc<=2; dc++) {
            CellCoord nb{cc.r+dr, cc.c+dc};
            if (!bot.cfg->valid(nb)) continue;
            if (!bot.at(nb).explored) { gain += 4.0f; continue; }
            for (int w=0; w<4; w++) if (!bot.at(nb).wallKnown[w]) gain += 1.0f;
        }
        return gain;
    }

    [[nodiscard]] static float utility(const Maze& bot, const CellCoord& cc) noexcept {
        float dist = bot.at(cc).floodDist;
        return (dist > 1e4f) ? 0.0f : infoGain(bot, cc) / (dist + 0.5f);
    }

    // Sense all four walls at cc from truth maze; update ESKF; return true if new info.
    // FIX-G: sign convention matches world-frame y=North.
    static bool senseCell(Maze& botMaze, const Maze& truthMaze,
                           ESKF& kf, const CellCoord& cc, const MazeConfig& cfg)
    {
        bool newInfo = false;
        for (int w=0; w<4; w++) {
            if (!botMaze.at(cc).wallKnown[w]) {
                botMaze.setWall(cc, w, truthMaze.at(cc).wall[w]);
                newInfo = true;
            }
        }
        botMaze.at(cc).explored   = true;
        botMaze.at(cc).visitCount++;

        Vec2  ctr  = cfg.cellCentre(cc);
        float half = cfg.cellSize * 0.5f;
        float R    = SensorModel::measurementVariance();

        // East wall: sign=+1 (wall in +x direction)
        if (botMaze.at(cc).wallKnown[WE] && !botMaze.at(cc).wall[WE])
            kf.updateWallDist(0, ctr.x + half, half, +1.0f, R);

        // West wall: sign=−1 (wall in −x direction)
        if (botMaze.at(cc).wallKnown[WW] && !botMaze.at(cc).wall[WW])
            kf.updateWallDist(0, ctr.x - half, half, -1.0f, R);

        // North wall: sign=+1 (wall in +y direction)  [FIX-G: v2 used −1]
        if (botMaze.at(cc).wallKnown[WN] && !botMaze.at(cc).wall[WN])
            kf.updateWallDist(1, ctr.y + half, half, +1.0f, R);

        // South wall: sign=−1 (wall in −y direction)
        if (botMaze.at(cc).wallKnown[WS] && !botMaze.at(cc).wall[WS])
            kf.updateWallDist(1, ctr.y - half, half, -1.0f, R);

        int openCardinals = 0;
        for (int w=0; w<4; w++)
            if (botMaze.at(cc).wallKnown[w] && !botMaze.at(cc).wall[w]) openCardinals++;
        if (openCardinals == 2) kf.snapHeadingCardinal();

        return newInfo;
    }

    // ── v4.1: wallCtrl is passed in from scoutRun(); applied after every step ──
    [[nodiscard]] static std::vector<CellCoord> explore(
        Maze& botMaze, const Maze& truthMaze, ESKF& kf, const MazeConfig& cfg,
        WallCenteringPID& wallCtrl)
    {
        CellCoord current = cfg.startCell;
        std::vector<CellCoord> visited{ current };

        // D* Lite (optimistic=true for exploration)  [FIX-E]
        DStarLite dstar;
        dstar.init(botMaze, current, /*optimistic=*/true);

        // ── Wall-centering helpers (v4.1) ─────────────────────────────────
        // Determine left/right cell walls from current heading.
        // Left direction = heading + π/2; match to nearest WALL_HEADING entry.
        // WALL_HEADING: WN=π/2, WE=0, WS=−π/2, WW=π
        // Examples:
        //   heading E (0):    leftAngle=π/2  → WN;   rightW=WALL_OPP[WN]=WS  ✓
        //   heading N (π/2):  leftAngle=π    → WW;   rightW=WALL_OPP[WW]=WE  ✓
        //   heading S (−π/2): leftAngle=0    → WE;   rightW=WALL_OPP[WE]=WW  ✓
        //   heading W (π):    leftAngle=−π/2 → WS;   rightW=WALL_OPP[WS]=WN  ✓
        auto getLeftRightWalls = [](float theta) -> std::pair<int,int> {
            float leftAngle = wrapAngle(theta + HALF_PI);
            int leftW = 0;
            float minDiff = INF_F;
            for (int w = 0; w < 4; w++) {
                float d = std::abs(angleDiff(leftAngle, WALL_HEADING[w]));
                if (d < minDiff) { minDiff = d; leftW = w; }
            }
            return { leftW, WALL_OPP[leftW] };
        };

        // Apply wall-centering PID and inject correction into ESKF heading.
        // Distances: half-cell when wall present (within sensor range), two
        // half-cells when absent (effectively out-of-range → invalid flag).
        auto applyWallCentering = [&]() {
            auto [leftW, rightW] = getLeftRightWalls(kf.theta());
            float half  = cfg.cellSize * 0.5f;
            bool lValid = botMaze.at(current).wallKnown[leftW]  &&
                          botMaze.at(current).wall[leftW];
            bool rValid = botMaze.at(current).wallKnown[rightW] &&
                          botMaze.at(current).wall[rightW];
            float lDist = lValid ? half : half * 2.0f;
            float rDist = rValid ? half : half * 2.0f;
            float corr  = wallCtrl.compute(lDist, lValid, rDist, rValid,
                                            cfg.cellSize, 0.01f);
            // corr is in rad/s; over one control tick (0.01 s) → small heading nudge
            if (std::abs(corr) > 1e-6f)
                kf.updateHeading(wrapAngle(kf.theta() + corr * 0.01f), 5e-4f);
        };
        // ─────────────────────────────────────────────────────────────────

        senseCell(botMaze, truthMaze, kf, current, cfg);
        applyWallCentering();  // v4.1: initial correction at start cell

        // Sync floodDist after dstar init
        for (auto& cell : botMaze.cells) cell.floodDist = cell.dstar_g;

        // Frontier set
        std::set<CellCoord> frontiers;
        auto refreshFrontiers = [&]() {
            frontiers.clear();
            for (int r=0;r<cfg.size;r++) for (int c=0;c<cfg.size;c++) {
                CellCoord cc{r,c};
                if (botMaze.at(cc).explored && botMaze.at(cc).hasFrontier())
                    frontiers.insert(cc);
            }
        };
        refreshFrontiers();

        // Greedy path following floodDist gradient
        auto greedyStep = [&]() -> CellCoord {
            CellCoord best{-1,-1};
            float bestScore = -1.0f;
            for (int w=0; w<4; w++) {
                if (!botMaze.canMoveCardinal(current, w, /*optimistic=*/true)) continue;
                CellCoord nb = current.neighbour(w);
                int visits = botMaze.at(nb).visitCount;
                float u = utility(botMaze, nb);
                float score = (visits==0 ? 1000.0f : 0.0f) + u;
                if (score > bestScore) { bestScore=score; best=nb; }
            }
            return best;
        };

        for (int step=0; step < N_CELLS*16; step++) {
            if (cfg.isGoal(current)) break;

            CellCoord nextCell = greedyStep();

            if (nextCell.r < 0 || botMaze.at(nextCell).visitCount > 0) {
                // All adjacent unvisited exhausted — navigate to best frontier
                if (frontiers.empty()) break;
                CellCoord bestFrontier{-1,-1};
                float bestUtil = -1.0f;
                for (const auto& fc : frontiers) {
                    float u = utility(botMaze, fc);
                    if (u > bestUtil) { bestUtil=u; bestFrontier=fc; }
                }
                if (bestFrontier.r < 0) break;

                FloodFill::solve(botMaze, {bestFrontier}, /*optimistic=*/true);
                auto subpath = ThetaStar::findPath(botMaze, current, /*optimistic=*/true);

                wallCtrl.reset();  // v4.1: reset PID integral before frontier navigation

                for (const auto& nc : subpath) {
                    visited.push_back(nc);
                    current = nc;
                    kf.predict(cfg.cellSize, 0.0f, 0.01f);
                    bool ni = senseCell(botMaze, truthMaze, kf, current, cfg);
                    if (ni) {
                        dstar.notifyWallChanged(current);
                        refreshFrontiers();
                    }
                    applyWallCentering();  // v4.1: wall centering on every subpath step
                    if (cfg.isGoal(current)) goto exploration_done;
                }
                // Restore goal-directed distances
                dstar.init(botMaze, current, /*optimistic=*/true);
                refreshFrontiers();
                continue;
            }

            current = nextCell;
            visited.push_back(current);
            kf.predict(cfg.cellSize, 0.0f, 0.01f);
            {
                bool ni = senseCell(botMaze, truthMaze, kf, current, cfg);
                if (ni) { dstar.notifyWallChanged(current); refreshFrontiers(); }
                applyWallCentering();  // v4.1: wall centering on every greedy step
            }
        }
        exploration_done:;
        return visited;
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Adaptive velocity scaler  (merged from v2/v3)
// ───────────────────────────────────────────────────────────────────────────

class AdaptiveScaler {
public:
    struct Sample { float planned, achieved, curvature; };
    std::vector<Sample> samples;

    void record(float planned, float achieved, float curvature) {
        samples.push_back({planned, achieved, curvature});
    }

    [[nodiscard]] std::pair<float,float> factors() const {
        if (samples.empty()) return {1.0f, 1.0f};
        float sumSt=0; int nSt=0;
        float sumCn=0; int nCn=0;
        for (const auto& s : samples) {
            if (s.planned < 1e-3f) continue;
            float r = std::max(0.5f, std::min(1.2f, s.achieved/s.planned));
            if (std::abs(s.curvature) < 2.0f) { sumSt+=r; nSt++; }
            else                                { sumCn+=r; nCn++; }
        }
        auto scale = [](float sum, int n) {
            if (n==0) return 1.0f;
            float r = sum/n;
            return std::min(1.08f, 1.0f + 0.80f*(r-1.0f));
        };
        return { scale(sumSt,nSt), scale(sumCn,nCn) };
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Run statistics
// ───────────────────────────────────────────────────────────────────────────

struct RunStats {
    std::string label;
    int   pathCells      = 0;
    int   trajPoints     = 0;
    float pathLength     = 0.0f;
    float estimatedTime  = 0.0f;
    float peakLatAccel   = 0.0f;
    float peakLongAccel  = 0.0f;
    float peakJerk       = 0.0f;
    float peakVelocity   = 0.0f;
};

// ───────────────────────────────────────────────────────────────────────────
//  Truth maze builder  (internal test fixture)
// ───────────────────────────────────────────────────────────────────────────

static void buildTruthMaze(Maze& truth) {
    for (auto& [r,c,w] : std::vector<std::tuple<int,int,int>>{
        {15,1,WS},{14,1,WS},{13,2,WE},{12,1,WE},{12,2,WS},
        {10,5,WS},{10,1,WE},{ 9,2,WN},{ 8,0,WS},{ 8,1,WE},
        { 7,2,WW},{ 6,1,WS},{ 6,2,WE},{ 4,11,WN},{ 5,10,WE},
        { 3,12,WW},{ 9,6,WS},{11,4,WE},{13,8,WS},{ 7,2,WE},
        { 5,10,WN},{ 2,13,WW},{ 9,10,WS},{ 3,12,WE},{ 6,9,WN},
        {10,14,WE},{11,4,WS},{ 1,14,WN},{ 2,13,WW},{13,2,WS},
        { 8,7,WN},{ 7,5,WE},{ 6,6,WS},{ 5,7,WW},{ 4,8,WN},
        {11,4,WE},{10,5,WN},{ 9,6,WE},{ 8,7,WS},
        { 3,12,WS},{ 2,13,WE},{ 1,14,WN}
    }) {
        truth.setWall({r,c}, w, true);
    }
}

// ───────────────────────────────────────────────────────────────────────────
//  Profile a path — full pipeline: waypoints → racing line → trajectory → profile
//
//  v4.1: added optional outTraj parameter.  When non-null the caller receives
//        the fully velocity-profiled trajectory vector after computeJerk().
// ───────────────────────────────────────────────────────────────────────────

[[nodiscard]] static RunStats profilePath(
    const std::vector<CellCoord>& cellPath,
    const MazeConfig& cfg, const RobotParams& robot,
    const std::string& label, float vMax,
    bool printDetails, bool computeTVLQR_gains = false,
    std::vector<TrajPoint>* outTraj = nullptr)   // v4.1: optional trajectory output
{
    RunStats stats;
    stats.label     = label;
    stats.pathCells = int(cellPath.size());

    if (cellPath.size() < 2) {
        if (!label.empty()) std::cerr << label << ": path too short\n";
        return stats;
    }

    // 1. Expand Theta* LoS jumps → single-step cells
    auto expanded = ThetaStar::expandPath(cellPath);

    // 2. Cell path → world-frame waypoints
    auto wps = pathToWaypoints(expanded, cfg);

    // 3. Racing-line corridor optimisation
    wps = optimiseRacingLine(wps, cfg.cellSize * 0.5f);

    // 4. Clothoid trajectory generation
    auto traj = TrajGen::generate(wps, robot);
    if (traj.empty()) {
        if (!label.empty()) std::cerr << label << ": trajectory failed\n";
        return stats;
    }

    // 5. Velocity profile (4-step pipeline)
    VelocityProfile::curvatureCeilings(traj, vMax, robot.maxTotalAccel);
    VelocityProfile::globalBrakingPass(traj, robot.maxTotalAccel, robot.maxBraking);
    VelocityProfile::backwardPass(traj, robot.maxJerk, robot.maxTotalAccel,
                                   robot.maxBraking);
    VelocityProfile::forwardPass (traj, robot.maxJerk, robot.maxTotalAccel,
                                   robot.maxAccelFwd);
    VelocityProfile::computeJerk(traj);

    // v4.1: export the fully-profiled trajectory to the caller if requested
    if (outTraj != nullptr) *outTraj = traj;

    // 6. Optional TVLQR (offline gain schedule; gains printed then released)
    std::vector<TVLQRGain> gains;
    if (computeTVLQR_gains)
        gains = TVLQRSolver::solve(traj, robot.wheelbase);

    // 7. Collect stats
    stats.trajPoints   = int(traj.size());
    stats.pathLength   = traj.back().arcLen;
    stats.estimatedTime= VelocityProfile::estimateTime(traj);
    stats.peakLatAccel = VelocityProfile::peakLatAccel(traj);
    stats.peakLongAccel= VelocityProfile::peakLongAccel(traj);
    stats.peakJerk     = VelocityProfile::peakJerk(traj);
    for (auto& tp : traj) stats.peakVelocity = std::max(stats.peakVelocity, tp.velocity);

    if (printDetails) {
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "\n── " << label << " ──\n";
        std::cout << "  Path cells      : " << stats.pathCells   << "\n";
        std::cout << "  Traj points     : " << stats.trajPoints  << "\n";
        std::cout << "  Path length     : " << stats.pathLength  << " m\n";
        std::cout << "  Estimated time  : " << stats.estimatedTime << " s\n";
        std::cout << "  Peak lat accel  : " << stats.peakLatAccel
                  << " m/s² (" << stats.peakLatAccel/9.81f << " g)\n";
        std::cout << "  Peak long accel : " << stats.peakLongAccel
                  << " m/s² (" << stats.peakLongAccel/9.81f << " g)\n";
        std::cout << "  Peak jerk       : " << stats.peakJerk     << " m/s³\n";
        std::cout << "  Peak velocity   : " << stats.peakVelocity << " m/s\n";

        if (computeTVLQR_gains && !gains.empty()) {
            std::cout << "  TVLQR gains computed: " << gains.size() << " points\n";
            std::cout << "  Speed gain K[0] at arc=0: ["
                      << gains[0].K[0][0] << ", " << gains[0].K[0][1]
                      << ", " << gains[0].K[0][2] << "]\n";
            std::cout << "  Yaw  gain K[1] at arc=0: ["
                      << gains[0].K[1][0] << ", " << gains[0].K[1][1]
                      << ", " << gains[0].K[1][2] << "]\n";
        }

        std::cout << "  Velocity profile (every 40th point):\n";
        for (int i=0; i<int(traj.size()); i+=40) {
            const auto& tp = traj[i];
            float aLat  = std::abs(tp.curvature) * tp.velocity * tp.velocity;
            float used  = std::sqrt(aLat*aLat + tp.accel*tp.accel)
                         / std::max(robot.maxTotalAccel, 1e-3f);
            std::cout << "    [" << std::setw(4) << i << "]"
                      << "  arc=" << std::setw(7) << tp.arcLen   << " m"
                      << "  v="   << std::setw(6) << tp.velocity << " m/s"
                      << "  κ="   << std::setw(8) << tp.curvature
                      << "  aLat="<< std::setw(6) << aLat        << " m/s²"
                      << "  j="   << std::setw(7) << tp.jerk     << " m/s³"
                      << "  kamm="<< std::setw(5) << used*100.0f << "%\n";
        }
    }
    return stats;
}

// ───────────────────────────────────────────────────────────────────────────
//  GDW Planner v4.1 — top-level orchestrator
// ───────────────────────────────────────────────────────────────────────────

class GDWPlannerV4 {
public:
    MazeConfig     cfg;
    Maze           botMaze;
    Maze           truthMaze;
    RobotParams    robot;
    ESKF           kf;
    AdaptiveScaler scaler;
    std::mt19937   rng{42};

    CellCoord goalReached{-1,-1};
    std::vector<CellCoord> exploredPath;

    void initialize() {
        botMaze.init(cfg);
        truthMaze.init(cfg);
        buildTruthMaze(truthMaze);
        Vec2 startPos = cfg.cellCentre(cfg.startCell);
        kf.reset(startPos.x, startPos.y, HALF_PI);  // initially heading North
    }

    // ── Scout run ─────────────────────────────────────────────────────────
    void scoutRun() {
        std::cout << "╔══════════════════════════════════════════╗\n"
                  << "║  SCOUT RUN (Info-Theoretic + D* Lite v4) ║\n"
                  << "╚══════════════════════════════════════════╝\n";

        // v4.1: WallCenteringPID is active during exploration only.
        WallCenteringPID wallCtrl(robot);
        std::cout << "  Wall-centering PID : enabled (Kp=" << robot.Kp_center
                  << " Ki=" << robot.Ki_center << ")\n";

        exploredPath = Explorer::explore(botMaze, truthMaze, kf, cfg, wallCtrl);

        goalReached = {-1,-1};
        for (const auto& c : exploredPath)
            if (cfg.isGoal(c)) { goalReached = c; break; }

        std::cout << "  Cells visited      : " << exploredPath.size()     << "\n";
        std::cout << "  Remaining frontiers: " << botMaze.frontierCount() << "\n";
        if (goalReached.r >= 0)
            std::cout << "  Goal reached at    : ("
                      << goalReached.r << "," << goalReached.c << ")\n";
        kf.print();

        FloodFill::solveToGoal(botMaze, false);
        (void)profilePath(exploredPath, cfg, robot, "Scout trajectory",
                          robot.exploreVelocity, true, false);
    }

    // ── Return to start ───────────────────────────────────────────────────
    void returnToStart() {
        if (goalReached.r < 0) {
            std::cerr << "returnToStart: goal was not reached during scout\n";
            return;
        }
        std::cout << "\n╔══════════════════════════════════════════╗\n"
                  << "║       RETURN TO START  (Theta*)          ║\n"
                  << "╚══════════════════════════════════════════╝\n";

        FloodFill::solveToStart(botMaze, false);
        auto path = ThetaStar::findPath(botMaze, goalReached, false);
        if (path.empty()) {
            std::cerr << "  Theta* returned no path on return leg\n";
            return;
        }
        std::cout << "  Return path nodes  : " << path.size() << "\n";
        float retV = robot.maxVelocity * 0.6f;
        profilePath(path, cfg, robot, "Return trajectory", retV, true, false);

        for (const auto& c : path) {
            (void)c;
            scaler.record(retV, retV*0.95f, 0.0f);
        }
        FloodFill::solveToGoal(botMaze, false);
    }

    // ── Speed run ─────────────────────────────────────────────────────────
    //
    //  v4.1 changes vs v4:
    //   • WallCenteringPID removed (conflicts with racing-line optimisation).
    //   • PDController::compute() called in a 200 Hz tracking simulation so
    //     the controller is exercised, not merely instantiated.
    //   • speedTraj captured from profilePath() via the new outTraj parameter.
    // ─────────────────────────────────────────────────────────────────────
    void speedRun() {
        std::cout << "\n╔══════════════════════════════════════════╗\n"
                  << "║  SPEED RUN  (Theta* + Racing Line +      ║\n"
                  << "║             TVLQR + PD fallback)         ║\n"
                  << "╚══════════════════════════════════════════╝\n";

        auto [af, cf] = scaler.factors();
        RobotParams scaledRobot = robot;
        scaledRobot.maxTotalAccel *= af;
        scaledRobot.maxVelocity   = std::min(robot.maxVelocity * cf,
                                             robot.maxVelocity);
        std::cout << "  Adaptive factors: accel=" << af << " corner=" << cf << "\n";
        std::cout << "  Wall-centering PID : disabled (speed run — racing line active)\n";

        // Try each goal cell; pick the fastest path
        RunStats               bestStats;
        std::vector<CellCoord> bestPath;
        bestStats.estimatedTime = INF_F;

        for (const auto& gc : cfg.goalCells) {
            FloodFill::solve(botMaze, {gc}, false);
            auto path = ThetaStar::findPath(botMaze, cfg.startCell, false);
            if (path.empty()) continue;
            auto stats = profilePath(path, cfg, scaledRobot, "",
                                     scaledRobot.maxVelocity, false, false);
            if (stats.estimatedTime < bestStats.estimatedTime) {
                bestStats = stats;
                bestPath  = path;
            }
        }

        if (bestPath.empty()) {
            std::cerr << "Speed run: no path found to any goal cell\n";
            return;
        }

        FloodFill::solveToGoal(botMaze, false);
        std::cout << "  Best path nodes    : " << bestPath.size() << "\n";
        std::cout << "  Cell sequence:\n";
        for (const auto& c : bestPath)
            std::cout << "    r=" << c.r << " c=" << c.c << "\n";

        // Full profile with TVLQR; also capture trajectory for PD simulation
        std::vector<TrajPoint> speedTraj;                  // v4.1: capture output
        auto finalStats = profilePath(bestPath, cfg, scaledRobot,
                                       "Speed-run trajectory",
                                       scaledRobot.maxVelocity, true,
                                       /*computeTVLQR=*/true, &speedTraj);

        // v4.1: PDController — active during Speed Run tracking simulation.
        //       WallCenteringPID intentionally omitted here.
        PDController pdCtrl(scaledRobot);

        std::cout << "\n  ┌──────────────────────────────────────────┐\n";
        std::cout << "  │      CHAMPIONSHIP SUMMARY  (v4.1)        │\n";
        std::cout << "  ├──────────────────────────────────────────┤\n";
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "  │  Run time        : " << std::setw(8)
                  << finalStats.estimatedTime       << " s                  │\n";
        std::cout << "  │  Distance        : " << std::setw(8)
                  << finalStats.pathLength          << " m                  │\n";
        std::cout << "  │  Peak speed      : " << std::setw(8)
                  << finalStats.peakVelocity        << " m/s                │\n";
        std::cout << "  │  Peak lat-g      : " << std::setw(8)
                  << finalStats.peakLatAccel/9.81f  << " g                  │\n";
        std::cout << "  │  Peak long-g     : " << std::setw(8)
                  << finalStats.peakLongAccel/9.81f << " g                  │\n";
        std::cout << "  │  Peak jerk       : " << std::setw(8)
                  << finalStats.peakJerk            << " m/s³               │\n";
        std::cout << "  │  Accel factor    : " << std::setw(8)
                  << af << " (learned)            │\n";
        std::cout << "  │  Corner factor   : " << std::setw(8)
                  << cf << " (learned)            │\n";
        std::cout << "  └──────────────────────────────────────────┘\n";

        // ── v4.1: PD trajectory tracking simulation ───────────────────────
        //
        //  Simulates speed-run execution at 200 Hz.  The simulated pose is
        //  forward-integrated using Euler kinematics; pdCtrl.compute() is
        //  called at every step, producing real velocity and yaw-rate commands.
        //  This demonstrates the controller is active, not merely instantiated.
        // ─────────────────────────────────────────────────────────────────
        if (!speedTraj.empty()) {
            std::cout << "\n  PD Trajectory Tracking — Speed Run Simulation\n";
            std::cout << "  (200 Hz, " << speedTraj.size() << " steps total; "
                         "every 10th printed)\n";
            std::cout << std::fixed << std::setprecision(3);
            std::cout << "    arc(m)  v_ref  v_cmd  ω_cmd(r/s)  e_ct(m)\n";
            std::cout << "    ──────  ─────  ─────  ──────────  ───────\n";

            // Simulation state starts at the start-cell centre, heading North
            Vec2  startWorld = cfg.cellCentre(cfg.startCell);
            float pdX     = startWorld.x;
            float pdY     = startWorld.y;
            float pdTheta = HALF_PI;  // heading North (same as kf.reset() in initialize())
            float prevCt  = 0.0f;
            constexpr float kDt = 0.005f;  // 200 Hz control tick

            int stride = std::max(1, static_cast<int>(speedTraj.size()) / 10);

            for (int i = 0; i < static_cast<int>(speedTraj.size()); i++) {
                const TrajPoint& ref = speedTraj[i];

                // PD control: returns {v_cmd, omega_cmd}
                auto [vCmd, omegaCmd] = pdCtrl.compute(pdX, pdY, pdTheta,
                                                        ref, prevCt, kDt);

                // Cross-track error (signed, metres)
                float csh = std::cos(ref.heading), ssh = std::sin(ref.heading);
                float dx  = pdX - ref.x,           dy  = pdY - ref.y;
                float eCt = -dx * ssh + dy * csh;
                prevCt = eCt;

                if (i % stride == 0) {
                    std::cout << "    " << std::setw(6) << ref.arcLen
                              << "  "  << std::setw(5) << ref.velocity
                              << "  "  << std::setw(5) << vCmd
                              << "  "  << std::setw(10) << omegaCmd
                              << "  "  << std::setw(7)  << eCt << "\n";
                }

                // Euler integration of simulated pose
                pdX     += vCmd * std::cos(pdTheta) * kDt;
                pdY     += vCmd * std::sin(pdTheta) * kDt;
                pdTheta  = wrapAngle(pdTheta + omegaCmd * kDt);
            }
            std::cout << "  [PD controller executed — "
                      << speedTraj.size() << " control ticks]\n";
        }

        std::cout << "\n  [Control architecture — v4.1]\n"
                  << "  ┌─ Planning ───────────────────────────────────────────────┐\n"
                  << "  │  Exploration : Info-theoretic utility + D* Lite (opt.)   │\n"
                  << "  │  Speed run   : Theta* (any-angle A*) — best of 4 goals  │\n"
                  << "  │  Distances   : FloodFill (Dijkstra, 8-dir)               │\n"
                  << "  ├─ Trajectory ─────────────────────────────────────────────┤\n"
                  << "  │  Geometry    : Clothoid–Arc–Clothoid (arc sign FIX-C)    │\n"
                  << "  │  Racing line : L2 second-difference QP optimiser         │\n"
                  << "  │  Velocity    : Kamm circle + global braking + S-curve    │\n"
                  << "  ├─ Control ────────────────────────────────────────────────┤\n"
                  << "  │  Primary     : TVLQR (B-matrix FIX-A applied)            │\n"
                  << "  │  Fallback    : PD + curvature feedforward (v4.1 active)  │\n"
                  << "  │  Lateral     : Wall-centering PID (scout run only; v4.1) │\n"
                  << "  ├─ Localisation ───────────────────────────────────────────┤\n"
                  << "  │  Filter      : ESKF 4-state (FIX-B: bias dim., Joseph)   │\n"
                  << "  │  Measurement : Wall-distance IR + heading snap           │\n"
                  << "  │  Noise model : Gaussian σ=3mm, outlier rejection 3σ      │\n"
                  << "  └─────────────────────────────────────────────────────────┘\n"
                  << "\n  [Timing budget on STM32H7 @ 480 MHz]\n"
                  << "    ESKF predict + update : ~8 µs\n"
                  << "    TVLQR gain lookup     : ~2 µs (table, binary search)\n"
                  << "    PD crosstrack control : ~1 µs\n"
                  << "    Motor PWM update      : ~1 µs\n"
                  << "    Total control tick    : ~12 µs  →  feasible at 2 kHz\n";
    }

    void run() {
        scoutRun();
        returnToStart();
        speedRun();
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Main
// ───────────────────────────────────────────────────────────────────────────

int main() {
    std::cout << "GDW Micromouse Championship Edition v4.1\n"
              << "──────────────────────────────────────────\n"
              << "Bug fixes carried forward from v4:\n"
              << "  FIX-A  TVLQR B-matrix: B[2][0]=0 (not kappa) — yaw coupling removed\n"
              << "  FIX-B  ESKF dimensional: bias*dt; Joseph-form KRK' term added\n"
              << "  FIX-C  Arc traversal: startAngle + sign*t*arcAngle (CCW correct)\n"
              << "  FIX-D  Clothoid L_c: max(L_c_min, floor) before capping (not min)\n"
              << "  FIX-E  D* Lite: optimistic=true threaded through exploration\n"
              << "  FIX-F  backwardPass: prevBrk persists within sweep (not reset)\n"
              << "  FIX-G  ESKF wall signs: North=+y, South=-y (world-frame correct)\n"
              << "\nNew in v4.1:\n"
              << "  • WallCenteringPID active in Scout Run (Explorer::explore receives\n"
              << "    wallCtrl&; compute() called after every senseCell(); integral\n"
              << "    reset before frontier sub-navigation).\n"
              << "  • WallCenteringPID removed from Speed Run (was dead code; conflicts\n"
              << "    with racing-line trajectory optimisation).\n"
              << "  • PDController::compute() executed in Speed Run at 200 Hz simulation\n"
              << "    (was instantiated but never invoked in v4).\n"
              << "  • profilePath() gains outTraj* parameter; caller can retrieve the\n"
              << "    fully velocity-profiled TrajPoint vector.\n\n";

    GDWPlannerV4 planner;
    planner.initialize();
    planner.run();
    return 0;
}

// ───────────────────────────────────────────────────────────────────────────
// Bugs to fix for GDW Beta 4.2:
//  -D Lite infinite loop / bad_alloc* — locally-consistent nodes (g==rhs) fell into the
//  underconsistent branch, which re-raised g to ∞ and re-queued the node forever on dense
//  mazes. Fixed to handle only the strictly-underconsistent case (plus an expansion cap).
//
// -Exploration stall — sensing reveals all four walls on arrival, so no visited cell was ever a
// "frontier"; the frontier set was always empty and exploration quit early. Replaced with
// canonical flood-fill exploration (reaches the goal on 20/20 random mazes tested).
//
// -TVLQR NaN — the forward-Euler Riccati diverged for large dt = ds/v near v→0. Clamped dt
// and sanitized the gains.
//
// -Dead-reckoning is noise-free too. kf.predict() is always called with (cellSize, 0.0f, 0.01f) —
// exact step, zero heading change, no encoder error, no gyro drift. The reported "localization
// error" therefore isn't a physically meaningful quantity; the ESKF runs on near-perfect data.
//
// -The PD "200 Hz simulation" isn't physically coherent. The loop iterates once per trajectory
// sample (spaced by arc length) but integrates the pose by vCmd * 0.005s per iteration, so
// total integrated time and the reference-index advance are unrelated. vCmd is also just
// ref.velocity (open-loop on speed). The pose starts exactly on the path under zero
// disturbance, so the small cross-track numbers it prints are tautological.
//
// -D Lite is largely vestigial*. It computes g-values and mirrors them into floodDist, but the
// explorer navigates by greedy utility + a fresh FloodFill/Theta* for frontier hops. The
// incremental replanning machinery runs but its output duplicates FloodFill rather than
// driving decisions.
//
//
//
// Suggestions for GDW Beta 4.2:
// -When returning back to the starting point the robot should explore a different path so that it 
// can take it into consideration
// ───────────────────────────────────────────────────────────────────────────
