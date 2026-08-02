// ═══════════════════════════════════════════════════════════════════════════
//  GDW Micromouse Championship Edition v4.3
//  C++17  ·  Single translation unit
//
//  v4.3 is a CONSERVATIVE production-hardening pass over v4.2.  Architecture
//  unchanged: ESKF, D* Lite (module retained), FloodFill exploration, Theta*
//  speed-run planning, racing-line optimiser, clothoid trajectory generation,
//  velocity profiling, TVLQR primary controller, PD fallback, wall-centering
//  PID, existing map representation and state machine all preserved.
//
//  ── v4.3 change inventory (see audit report for severity / root cause) ───
//
//  FIX-L  expandPath() produced off-grid / non-terminal cells for Theta*
//         line-of-sight jumps that are neither pure-cardinal nor pure-diagonal
//         (e.g. dr=2, dc=1).  Replaced with a Bresenham cell walk identical to
//         the one lineOfSight() verifies.                            [BUG FIX]
//
//  FIX-M  Clothoid-arc-clothoid heading budget was wrong: each linear-curvature
//         clothoid turns by L_c/(2R), so the two clothoids together turn L_c/R,
//         but the code subtracted 2*L_c/R from the total turn, under-rotating
//         each turn.  Corrected to arcAngle = |dtheta| - L_c/R.        [BUG FIX]
//
//  FIX-N  Return leg could never be planned: ThetaStar::findPath() is hard-wired
//         to terminate on cfg.goalCells, so planning from the goal back to the
//         start terminated instantly and returned a single node.  Added
//         ThetaStar::findPathTo(start, target) and used it for the return leg
//         and per-goal speed-run searches.                            [BUG FIX]
//
//  FIX-O  Exploration committed to one optimistic full path and drove every
//         cell of it without re-checking the map, so it would command moves
//         across walls discovered mid-path.  Now the robot verifies each step
//         against the KNOWN map before moving and replans when blocked.
//                                                          [BUG FIX / HARDWARE]
//
//  FIX-P  TVLQRSolver::computeControl() indexed gains[idx] without guarding the
//         empty-schedule case (out-of-range read).               [BUG FIX]
//
//  FIX-R  TVLQR Riccati step computed B·R⁻¹·Bᵀ as B[r][m]*Rinv[m][n]*B[n][c],
//         indexing B (a 3×2) out of bounds at c=2 and forming the wrong term;
//         the transpose factor is B[c][n].  Confirmed via UBSan.    [BUG FIX]
//
//  ROB-1  Speed-run search uses findPathTo(gc) per goal cell (admissible)
//         instead of an any-goal A* seeded with a single-cell heuristic.
//                                                                  [ROBUSTNESS]
//  ROB-2  sanitizeTrajectory(): non-finite poses / velocities scrubbed and
//         arcLen forced non-decreasing before the profile is consumed.
//                                                                  [ROBUSTNESS]
//  ROB-3  ESKF::sanitize(): covariance kept finite & diagonally non-negative.
//                                                                  [ROBUSTNESS]
//  ROB-4  Explorer outer watchdog cap on total steps.              [ROBUSTNESS]
//
//  HW-1   ESKF::predict() rejects implausible encoder deltas and clamps the
//         gyro-implied rate to BMI088 full scale (±2000 dps).         [HARDWARE]
//  HW-2   PD and TVLQR outputs clamped to safe envelopes (nominal no-ops).
//                                                                    [HARDWARE]
//  HW-3   Exploration dead-reckoning uses true per-step arc length and
//         dt = ds / exploreVelocity (unit-consistent).               [HARDWARE]
//
//  NEW    Alternate Return Exploration (returnToStart): on the way home the
//         robot prefers a different feasible, information-maximising route,
//         within a bounded distance budget, using only known-open edges.
//                                                                    [FEATURE]
//
//  ── Inherited v4.2 fixes (unchanged) ────────────────────────────────────
//    FIX-H D* Lite strictly-underconsistent only + cap.  FIX-I flood-fill
//    frontier exploration.  FIX-J TVLQR dt clamp + gain sanitisation.
//    FIX-K PD simulation dt = ds / v_ref.
//
//  ── Coordinate frame (fixed, applies to ALL subsystems) ──────────────────
//    World: x → East, y → North.  Cell (r,c): row r from North, col c from West.
//    Cell centre: x=(c+0.5)*cellSize, y=-(r+0.5)*cellSize.
//    Heading: 0=East, π/2=North, −π/2=South, π=West.  +curvature = left (CCW).
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

// HW-1: BMI088 gyro full scale at ±2000 dps.
inline constexpr float GYRO_MAX_RATE = 34.9f;     // rad/s
// HW-1: a single dead-reckoning step should never advance more than this.
inline constexpr float MAX_STEP_DS   = 1.0f;      // m
// HW-2: control output safety envelopes (generous — no-ops in nominal regime).
inline constexpr float OMEGA_LIMIT   = 25.0f;     // rad/s
inline constexpr float DELTA_V_LIMIT = 6.0f;      // m/s

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

[[nodiscard]] inline float clampf(float v, float lo, float hi) noexcept {
    return std::max(lo, std::min(hi, v));
}

// ───────────────────────────────────────────────────────────────────────────
//  Wall / direction encoding
// ───────────────────────────────────────────────────────────────────────────

enum Wall : int { WN = 0, WE = 1, WS = 2, WW = 3 };

inline constexpr int   WALL_OPP[4]     = { WS, WW, WN, WE };
inline constexpr int   WALL_DC[4]      = {  0,  1,  0, -1 };
inline constexpr int   WALL_DR[4]      = { -1,  0,  1,  0 };
inline constexpr float WALL_HEADING[4] = { HALF_PI, 0.0f, -HALF_PI, PI };

inline constexpr int   D8C[8]    = {  0, 1, 1, 1,  0, -1, -1, -1 };
inline constexpr int   D8R[8]    = { -1,-1, 0, 1,  1,  1,  0, -1 };
inline constexpr float D8COST[8] = { 1.f, SQRT2, 1.f, SQRT2, 1.f, SQRT2, 1.f, SQRT2 };

inline constexpr int D8WALLS[8][2] = {
    { WN, -1 }, { WN, WE }, { WE, -1 }, { WE, WS },
    { WS, -1 }, { WS, WW }, { WW, -1 }, { WN, WW }
};

inline constexpr float D8HEADING[8] = {
    HALF_PI, PI * 0.25f, 0.0f, -PI * 0.25f,
    -HALF_PI, -(PI * 0.75f), PI, PI * 0.75f
};

// ───────────────────────────────────────────────────────────────────────────
//  CellCoord
// ───────────────────────────────────────────────────────────────────────────

struct CellCoord {
    int r = 0;
    int c = 0;

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

// FIX-O helper: 8-direction index from a to an adjacent b, or -1 if not adjacent.
[[nodiscard]] inline int dirFromDelta(const CellCoord& a, const CellCoord& b) noexcept {
    int dr = b.r - a.r, dc = b.c - a.c;
    for (int d8 = 0; d8 < 8; d8++)
        if (D8R[d8] == dr && D8C[d8] == dc) return d8;
    return -1;
}

// ───────────────────────────────────────────────────────────────────────────
//  Vec2
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
    float cellSize = 0.18f;

    std::array<CellCoord,4> goalCells = {{ {7,7},{7,8},{8,7},{8,8} }};
    CellCoord startCell = { 15, 0 };

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
    float dstar_g   = INF_F;
    float dstar_rhs = INF_F;

    [[nodiscard]] bool passableOpt(int w)   const noexcept { return !(wallKnown[w] && wall[w]); }
    [[nodiscard]] bool passableCons(int w)  const noexcept { return wallKnown[w] && !wall[w]; }
    [[nodiscard]] bool hasFrontier() const noexcept {
        for (int w = 0; w < 4; w++) if (!wallKnown[w]) return true;
        return false;
    }
    [[nodiscard]] int unknownWalls() const noexcept {
        int n = 0;
        for (int w = 0; w < 4; w++) if (!wallKnown[w]) n++;
        return n;
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

    [[nodiscard]] bool checkConsistency() const noexcept {
        for (int r = 0; r < MAZE_N; r++)
            for (int c = 0; c < MAZE_N; c++) {
                CellCoord cc{r,c};
                for (int w = 0; w < 4; w++) {
                    CellCoord nb = cc.neighbour(w);
                    if (!cfg->valid(nb)) continue;
                    const Cell& a = at(cc);
                    const Cell& b = at(nb);
                    if (a.wallKnown[w] && b.wallKnown[WALL_OPP[w]] &&
                        a.wall[w] != b.wall[WALL_OPP[w]])
                        return false;
                }
            }
        return true;
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
            if (d > maze.at(cc).floodDist + 1e-6f) continue;

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

    // ROB / NEW: capture the current floodDist field so two fields can coexist.
    static void snapshot(const Maze& maze, std::array<float,N_CELLS>& out) {
        for (int i = 0; i < N_CELLS; i++) out[i] = maze.cells[i].floodDist;
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  D* Lite — incremental replanning  (module retained; currently off-path)
//
//  NOTE (v4.3): not on any live execution path — exploration uses FloodFill +
//  Theta* (FIX-I).  Preserved per the change spec; v4.2 FIX-H kept.
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
        for (auto& cell : maze->cells) { cell.dstar_g = INF_F; cell.dstar_rhs = INF_F; }
        while (!U.empty()) U.pop();

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
        int maxIter = 20000;
        while (!U.empty() && maxIter-- > 0) {
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
            } else if (gu < ru) {
                maze->at(u).dstar_g = INF_F;
                U.push({ calcKey(u), u });
                for (int d8 = 0; d8 < 8; d8++) {
                    if (!maze->canMove8(u, d8, optimistic)) continue;
                    CellCoord nb = u.step8(d8);
                    if (maze->cfg->isGoal(nb)) continue;
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
        for (auto& cell : maze->cells)
            cell.floodDist = cell.dstar_g;
    }

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
//  Theta* — any-angle A*
// ───────────────────────────────────────────────────────────────────────────

class ThetaStar {
public:
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

    [[nodiscard]] static float octile(const CellCoord& a, const CellCoord& b) noexcept {
        float dr = float(std::abs(a.r-b.r));
        float dc = float(std::abs(a.c-b.c));
        return std::max(dr,dc) + (SQRT2-1.0f)*std::min(dr,dc);
    }

    // Existing behaviour: plan toward cfg.goalCells, heuristic = maze.floodDist.
    [[nodiscard]] static std::vector<CellCoord> findPath(
        const Maze& maze, const CellCoord& start, bool optimistic)
    {
        return search(maze, start, optimistic,
                      [&](const CellCoord& cc){ return maze.cfg->isGoal(cc); },
                      [&](const CellCoord& cc){ return maze.at(cc).floodDist; });
    }

    // FIX-N: plan toward an arbitrary target with an admissible octile heuristic.
    [[nodiscard]] static std::vector<CellCoord> findPathTo(
        const Maze& maze, const CellCoord& start,
        const CellCoord& target, bool optimistic)
    {
        return search(maze, start, optimistic,
                      [&](const CellCoord& cc){ return cc == target; },
                      [&](const CellCoord& cc){ return octile(cc, target); });
    }

    // FIX-L: Bresenham cell walk between two cells (excl. a, incl. b), identical
    // to the line lineOfSight() verifies.
    static void bresenhamCells(const CellCoord& a, const CellCoord& b,
                               std::vector<CellCoord>& out)
    {
        int r0=a.r, c0=a.c, r1=b.r, c1=b.c;
        int dr=std::abs(r1-r0), dc=std::abs(c1-c0);
        int sr=(r1>r0)?1:-1, sc=(c1>c0)?1:-1;
        int r=r0, c=c0, err=dc-dr;
        for (int step=0; step<=dr+dc; step++) {
            if (r==r1 && c==c1) { out.push_back({r,c}); break; }
            int e2=2*err;
            bool mC=(e2>-dr), mR=(e2<dc);
            if (mC && mR)      { c+=sc; r+=sr; err+=dr-dc; }
            else if (mC)       { c+=sc; err+=dr; }
            else               { r+=sr; err-=dc; }
            out.push_back({r,c});
        }
    }

    [[nodiscard]] static std::vector<CellCoord> expandPath(
        const std::vector<CellCoord>& path)
    {
        std::vector<CellCoord> expanded;
        if (path.empty()) return expanded;
        expanded.push_back(path[0]);
        for (size_t i=1; i<path.size(); i++)
            bresenhamCells(path[i-1], path[i], expanded);   // FIX-L
        return expanded;
    }

private:
    template <class GoalTest, class Heur>
    [[nodiscard]] static std::vector<CellCoord> search(
        const Maze& maze, const CellCoord& start, bool optimistic,
        GoalTest isGoal, Heur h)
    {
        std::array<float,     N_CELLS> gCost;
        std::array<CellCoord, N_CELLS> parent;
        std::array<bool,      N_CELLS> closed;
        gCost.fill(INF_F);
        parent.fill({-1,-1});
        closed.fill(false);

        if (!maze.cfg->valid(start)) return {};
        if (isGoal(start)) return { start };

        gCost[start.idx()]  = 0.0f;
        parent[start.idx()] = {-2,-2};

        struct Node {
            float f; CellCoord cc;
            bool operator>(const Node& o) const noexcept { return f > o.f; }
        };
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
        open.push({ h(start), start });

        CellCoord reached{-1,-1};

        while (!open.empty()) {
            auto [f, cc] = open.top(); open.pop();
            if (closed[cc.idx()]) continue;
            closed[cc.idx()] = true;
            if (isGoal(cc)) { reached=cc; break; }

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
                    float hv = h(nb);
                    open.push({ ng + (std::isfinite(hv) ? hv : 0.0f), nb });
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

    [[nodiscard]] static bool checkWall(const Maze& m, const CellCoord& cc,
                                         int w, bool optimistic) noexcept {
        if (!m.cfg->valid(cc)) return false;
        const Cell& cell = m.at(cc);
        if (optimistic) return !(cell.wallKnown[w] && cell.wall[w]);
        return cell.wallKnown[w] && !cell.wall[w];
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Sensor model
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
//  ESKF — Error-State Kalman Filter
//  v4.3: ROB-3 sanitize(), HW-1 encoder/gyro fault handling.
// ───────────────────────────────────────────────────────────────────────────

class ESKF {
public:
    float nom_x=0.f, nom_y=0.f, nom_theta=0.f;
    std::array<float,4> err{0,0,0,0};
    std::array<float,16> P{};

    float Q_xy    = 1e-5f;
    float Q_theta = 2e-4f;
    float Q_bias  = 1e-6f;
    float R_wall  = SensorModel::NOISE_VAR;
    float R_hdg   = 1e-4f;

    bool faultEncoder = false;
    bool faultGyro    = false;

    ESKF() {
        P.fill(0);
        P[0]=0.01f; P[5]=0.01f; P[10]=0.001f; P[15]=1e-4f;
    }

    void reset(float x0, float y0, float h0) noexcept {
        nom_x=x0; nom_y=y0; nom_theta=h0;
        err.fill(0);
        P.fill(0);
        P[0]=1e-4f; P[5]=1e-4f; P[10]=1e-4f; P[15]=1e-6f;
        faultEncoder = faultGyro = false;
    }

    [[nodiscard]] float x()     const noexcept { return nom_x     + err[0]; }
    [[nodiscard]] float y()     const noexcept { return nom_y     + err[1]; }
    [[nodiscard]] float theta() const noexcept { return nom_theta + err[2]; }
    [[nodiscard]] float bias()  const noexcept { return err[3]; }

    float& pij(int i, int j) noexcept       { return P[i*4+j]; }
    float  pij(int i, int j) const noexcept { return P[i*4+j]; }

    // ROB-3
    void sanitize() noexcept {
        if (!std::isfinite(nom_x))     nom_x     = 0.0f;
        if (!std::isfinite(nom_y))     nom_y     = 0.0f;
        if (!std::isfinite(nom_theta)) nom_theta = 0.0f;
        for (int i=0;i<4;i++)  if (!std::isfinite(err[i])) err[i]=0.0f;
        for (int i=0;i<16;i++) if (!std::isfinite(P[i]))   P[i]=0.0f;
        for (int d=0; d<4; d++) { int k=d*4+d; if (P[k] < 1e-9f) P[k] = 1e-9f; }
    }

    void predict(float ds, float dtheta_meas, float dt) noexcept {
        // HW-1: encoder fault detection
        if (!std::isfinite(ds) || std::abs(ds) > MAX_STEP_DS) {
            faultEncoder = true;
            ds = clampf(std::isfinite(ds) ? ds : 0.0f, -MAX_STEP_DS, MAX_STEP_DS);
        }
        // HW-1: gyro saturation
        if (dt > 1e-6f) {
            float maxDtheta = GYRO_MAX_RATE * dt;
            if (!std::isfinite(dtheta_meas) || std::abs(dtheta_meas) > maxDtheta) {
                faultGyro = true;
                dtheta_meas = clampf(std::isfinite(dtheta_meas) ? dtheta_meas : 0.0f,
                                     -maxDtheta, maxDtheta);
            }
        }

        float dtheta   = dtheta_meas - err[3] * dt;
        float midTheta = nom_theta + 0.5f * dtheta;
        float c = std::cos(midTheta), s_t = std::sin(midTheta);

        nom_x     += ds * c;
        nom_y     += ds * s_t;
        nom_theta  = wrapAngle(nom_theta + dtheta);

        std::array<float,16> Pn = P;
        for (int j=0;j<4;j++) Pn[0*4+j] += (-ds*s_t)*P[2*4+j];
        for (int j=0;j<4;j++) Pn[1*4+j] += (ds*c)   *P[2*4+j];
        for (int j=0;j<4;j++) Pn[2*4+j] += (-dt)     *P[3*4+j];
        std::array<float,16> P2 = Pn;
        for (int i=0;i<4;i++) P2[i*4+0] += Pn[i*4+2]*(-ds*s_t);
        for (int i=0;i<4;i++) P2[i*4+1] += Pn[i*4+2]*(ds*c);
        for (int i=0;i<4;i++) P2[i*4+2] += Pn[i*4+3]*(-dt);
        P2[0]  += Q_xy    * std::abs(ds);
        P2[5]  += Q_xy    * std::abs(ds);
        P2[10] += Q_theta * std::abs(dtheta);
        P2[15] += Q_bias;
        P = P2;
        sanitize();
    }

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
        if (innov*innov > 9.0f*S) return;  // > 3σ outlier rejection

        std::array<float,4> K{};
        for (int i=0;i<4;i++) {
            float PH=0; for (int k=0;k<4;k++) PH += pij(i,k)*H[k];
            K[i] = PH / S;
        }

        for (int i=0;i<4;i++) err[i] += K[i]*innov;

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
        err[0]=err[1]=err[2]=0.0f;
        sanitize();
    }

    void updateHeading(float measuredHeading, float R_meas) noexcept {
        float innov = angleDiff(measuredHeading, theta());
        std::array<float,4> H{}; H[2]=1.0f;
        float S = pij(2,2) + R_meas;
        if (S < 1e-12f) return;
        std::array<float,4> K{};
        for (int i=0;i<4;i++) K[i] = pij(i,2)/S;
        for (int i=0;i<4;i++) err[i] += K[i]*innov;
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
        sanitize();
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
        if (faultEncoder || faultGyro)
            std::cout << "  WARN sensor fault flags: encoder=" << faultEncoder
                      << " gyro=" << faultGyro << "\n";
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Robot parameters
// ───────────────────────────────────────────────────────────────────────────

struct RobotParams {
    float maxTotalAccel   = 12.0f;
    float maxBraking      = 10.0f;
    float maxAccelFwd     =  9.0f;
    float maxJerk         = 60.0f;
    float maxVelocity     =  5.0f;
    float exploreVelocity =  0.6f;

    float wheelbase         = 0.07f;
    float trackWidth        = 0.06f;
    float cellSize          = 0.18f;
    float steeringBandwidth = 20.0f;

    float Kp_crosstrack = 4.0f;
    float Kd_crosstrack = 0.3f;
    float Kp_heading    = 2.0f;
    float Kd_heading    = 0.1f;

    float Kp_center = 3.0f;
    float Ki_center = 0.1f;
};

// ───────────────────────────────────────────────────────────────────────────
//  Trajectory point
// ───────────────────────────────────────────────────────────────────────────

struct TrajPoint {
    float x        = 0.0f;
    float y        = 0.0f;
    float heading  = 0.0f;
    float curvature= 0.0f;
    float velocity = 0.0f;
    float arcLen   = 0.0f;
    float accel    = 0.0f;
    float jerk     = 0.0f;
    float ff_steer_rate = 0.0f;
};

// ───────────────────────────────────────────────────────────────────────────
//  Clothoid segment — 8-point Gauss-Legendre quadrature
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
//  Racing-line corridor optimiser
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
//  Trajectory Generator — clothoid-arc-clothoid  (FIX-M applied)
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
                int Ns = SAMPLES_STRAIGHT;
                for (int i=startIdx; i<=Ns; i++) {
                    float t = float(i)/float(Ns);
                    emit(wa.x+t*(wb.x-wa.x), wa.y+t*(wb.y-wa.y), wa.heading, 0.0f);
                }
                kPrev = 0.0f;

            } else {
                float R = segLen / (2.0f * std::sin(std::abs(dhdg)*0.5f));
                R = std::max(R, robot.cellSize * 0.05f);
                float kTurn = (1.0f/R) * (dhdg>0.0f ? 1.0f : -1.0f);

                float dkappa  = std::abs(kTurn - kPrev);
                float L_c_min = dkappa * robot.maxVelocity / robot.steeringBandwidth;
                float L_c_max = segLen * 0.45f;
                float L_c     = std::max(L_c_min, 0.005f);
                L_c           = std::min(L_c, L_c_max);

                ClothoidSeg entry{ wa.x, wa.y, wa.heading, kPrev, kTurn, L_c };
                for (int i=startIdx; i<=SAMPLES_CLOTHOID; i++) {
                    float s = float(i)/float(SAMPLES_CLOTHOID)*L_c;
                    auto st = entry.eval(s);
                    emit(st.x, st.y, st.theta, st.kappa);
                }

                auto eEnd = entry.eval(L_c);
                float sign = (dhdg > 0.0f) ? 1.0f : -1.0f;
                float perpAngle = eEnd.theta + sign * HALF_PI;
                float cx = eEnd.x + R * std::cos(perpAngle);
                float cy = eEnd.y + R * std::sin(perpAngle);

                // FIX-M: each clothoid turns L_c/(2R); two clothoids turn L_c/R.
                float clothoidTurn = L_c / (2.0f * R);
                float arcAngle     = std::abs(dhdg) - 2.0f * clothoidTurn;

                if (arcAngle > 1e-4f) {
                    float startArc = std::atan2(eEnd.y-cy, eEnd.x-cx);
                    for (int i=1; i<=SAMPLES_ARC; i++) {
                        float t = float(i)/float(SAMPLES_ARC);
                        float ang = startArc + sign * t * arcAngle;
                        float px  = cx + R * std::cos(ang);
                        float py  = cy + R * std::sin(ang);
                        float hdg = wrapAngle(ang + sign * HALF_PI);
                        emit(px, py, hdg, kTurn);
                    }
                }

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
//  Velocity Profile — Kamm circle + S-curve  (ROB-2 sanitiser added)
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

    static void backwardPass(std::vector<TrajPoint>& traj,
                              float maxJerk, float aTotal, float aBrakeMax,
                              int maxIter=25)
    {
        int N = int(traj.size());
        if (N < 2) return;
        traj.back().velocity = 0.0f;

        for (int iter=0; iter<maxIter; iter++) {
            float maxChange = 0.0f;
            float prevBrk   = 0.0f;

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

    // NOTE (audit, intentionally NOT auto-fixed to preserve speed-run timing):
    //  the forward/backward passes limit the accel slew per unit DISTANCE
    //  (maxJerk*ds), whereas the value below is temporal jerk da/dt with
    //  dt≈ds/v.  On the non-uniformly sampled clothoid+arc+racing-line path the
    //  smallest-dt samples at high speed make this reported peak jerk large and
    //  not directly comparable to robot.maxJerk.  It is a DIAGNOSTIC only (never
    //  fed to TVLQR/PD), so it does not affect tracking.  A proper fix
    //  (resample at uniform dt before differentiating, or design jerk-limited
    //  clothoids) is a profiler redesign and out of scope for this conservative
    //  pass — see the audit report.
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

    // ROB-2
    [[nodiscard]] static bool sanitizeTrajectory(std::vector<TrajPoint>& traj) {
        bool clean = true;
        float lastArc = 0.0f;
        for (auto& tp : traj) {
            if (!std::isfinite(tp.x) || !std::isfinite(tp.y) || !std::isfinite(tp.heading))
                clean = false;
            if (!std::isfinite(tp.x)) tp.x = 0.0f;
            if (!std::isfinite(tp.y)) tp.y = 0.0f;
            if (!std::isfinite(tp.heading)) tp.heading = 0.0f;
            if (!std::isfinite(tp.curvature)) { tp.curvature = 0.0f; clean = false; }
            if (!std::isfinite(tp.velocity) || tp.velocity < 0.0f) { tp.velocity = 0.0f; clean = false; }
            if (!std::isfinite(tp.accel)) tp.accel = 0.0f;
            if (!std::isfinite(tp.jerk))  tp.jerk  = 0.0f;
            if (!std::isfinite(tp.arcLen) || tp.arcLen < lastArc) { tp.arcLen = lastArc; clean = false; }
            lastArc = tp.arcLen;
        }
        return clean;
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
//  TVLQR gain schedule  (FIX-J retained, FIX-P + HW-2 added)
// ───────────────────────────────────────────────────────────────────────────

struct TVLQRGain {
    float K[2][3];
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
            dt = std::max(std::min(dt, 0.2f), 1e-5f);

            float A[3][3] = {
                {0, 0, -v*std::sin(h)},
                {0, 0,  v*std::cos(h)},
                {0, 0,  0            }
            };
            float B[3][2] = {
                {std::cos(h), 0},
                {std::sin(h), 0},
                {0,           1}
            };

            float BRB[3][3]{};
            for (int r=0;r<3;r++) for (int c=0;c<3;c++)
                for (int m=0;m<2;m++) for (int n=0;n<2;n++)
                    BRB[r][c] += B[r][m]*Rinv[m][n]*B[c][n];   // FIX-R: Bᵀ is B[c][n]

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
            for (int r=0;r<3;r++) {
                P[r][r] = std::max(P[r][r], 1e-4f);
                if (!std::isfinite(P[r][r])) P[r][r] = 1e-4f;
            }

            float BTP[2][3]{};
            for (int r=0;r<2;r++) for (int c=0;c<3;c++)
                for (int m=0;m<3;m++) BTP[r][c] += B[m][r]*P[m][c];
            for (int r=0;r<2;r++) for (int c=0;c<3;c++) {
                gains[i].K[r][c] = 0;
                for (int m=0;m<2;m++) gains[i].K[r][c] += Rinv[r][m]*BTP[m][c];
                if (!std::isfinite(gains[i].K[r][c])) gains[i].K[r][c] = 0.0f;
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
        // FIX-P: guard the empty-schedule case (was an out-of-range read).
        if (gains.empty()) { delta_v = 0.0f; delta_omega = 0.0f; return; }

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

        // HW-2 safety clamps (no-ops in the nominal regime).
        if (!std::isfinite(delta_v))     delta_v = 0.0f;
        if (!std::isfinite(delta_omega)) delta_omega = 0.0f;
        delta_v     = clampf(delta_v, -DELTA_V_LIMIT, DELTA_V_LIMIT);
        delta_omega = clampf(delta_omega, -OMEGA_LIMIT, OMEGA_LIMIT);
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  PD controller with curvature feedforward  (HW-2 clamp added)
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
        float e_xt  = -dx*sh + dy*ch;
        float e_hdg = angleDiff(estTheta, ref.heading);
        float de_xt = (dt > 1e-6f) ? (e_xt - prevCrossTrack)/dt : 0.0f;
        float ff    = ref.velocity * ref.curvature;
        float fb    = params.Kp_crosstrack * e_xt
                    + params.Kd_crosstrack * de_xt
                    + params.Kp_heading    * e_hdg;
        float omega = ff + fb;
        if (!std::isfinite(omega)) omega = 0.0f;
        omega = clampf(omega, -OMEGA_LIMIT, OMEGA_LIMIT);   // HW-2
        return { ref.velocity, omega };
    }
private:
    const RobotParams& params;
};

// ───────────────────────────────────────────────────────────────────────────
//  Wall-centering PID  (scout only)
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
//  Explorer — canonical flood-fill frontier exploration (FIX-O, HW-3, ROB-4)
// ───────────────────────────────────────────────────────────────────────────

class Explorer {
public:
    static void applyWallCentering(Maze& botMaze, ESKF& kf, const MazeConfig& cfg,
                                    WallCenteringPID& wallCtrl, const CellCoord& current,
                                    float dt)
    {
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

        auto [leftW, rightW] = getLeftRightWalls(kf.theta());
        float half  = cfg.cellSize * 0.5f;
        bool lValid = botMaze.at(current).wallKnown[leftW]  &&
                      botMaze.at(current).wall[leftW];
        bool rValid = botMaze.at(current).wallKnown[rightW] &&
                      botMaze.at(current).wall[rightW];
        float lDist = lValid ? half : half * 2.0f;
        float rDist = rValid ? half : half * 2.0f;
        float corr  = wallCtrl.compute(lDist, lValid, rDist, rValid, cfg.cellSize, dt);
        if (std::abs(corr) > 1e-6f)
            kf.updateHeading(wrapAngle(kf.theta() + corr * dt), 5e-4f);
    }

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

        if (botMaze.at(cc).wallKnown[WE] && !botMaze.at(cc).wall[WE])
            kf.updateWallDist(0, ctr.x + half, half, +1.0f, R);
        if (botMaze.at(cc).wallKnown[WW] && !botMaze.at(cc).wall[WW])
            kf.updateWallDist(0, ctr.x - half, half, -1.0f, R);
        if (botMaze.at(cc).wallKnown[WN] && !botMaze.at(cc).wall[WN])
            kf.updateWallDist(1, ctr.y + half, half, +1.0f, R);
        if (botMaze.at(cc).wallKnown[WS] && !botMaze.at(cc).wall[WS])
            kf.updateWallDist(1, ctr.y - half, half, -1.0f, R);

        int openCardinals = 0;
        for (int w=0; w<4; w++)
            if (botMaze.at(cc).wallKnown[w] && !botMaze.at(cc).wall[w]) openCardinals++;
        if (openCardinals == 2) kf.snapHeadingCardinal();

        return newInfo;
    }

    // HW-3: dead-reckon a single verified step (cardinal or diagonal).
    static void deadReckonStep(ESKF& kf, const CellCoord& from, const CellCoord& to,
                               const MazeConfig& cfg, float exploreVel)
    {
        float ds = (from.r==to.r || from.c==to.c) ? cfg.cellSize : cfg.cellSize * SQRT2;
        float dt = (exploreVel > 1e-4f) ? ds / exploreVel : 0.01f;
        kf.predict(ds, 0.0f, dt);
    }

    [[nodiscard]] static std::vector<CellCoord> explore(
        Maze& botMaze, const Maze& truthMaze, ESKF& kf, const MazeConfig& cfg,
        WallCenteringPID& wallCtrl, float exploreVel)
    {
        CellCoord current = cfg.startCell;
        std::vector<CellCoord> visited{ current };

        senseCell(botMaze, truthMaze, kf, current, cfg);
        botMaze.at(current).explored = true;

        const int   MAX_STEPS = 8 * N_CELLS;   // ROB-4 watchdog
        int         steps     = 0;
        const float dtStep    = (exploreVel > 1e-4f) ? (cfg.cellSize / exploreVel) : 0.01f;

        while (steps < MAX_STEPS) {
            if (cfg.isGoal(current)) break;

            FloodFill::solveToGoal(botMaze, /*optimistic=*/true);
            auto raw = ThetaStar::findPath(botMaze, current, /*optimistic=*/true);
            if (raw.size() < 2) break;
            auto path = ThetaStar::expandPath(raw);

            wallCtrl.reset();

            bool advanced = false;
            for (size_t i = 1; i < path.size(); i++) {
                CellCoord next = path[i];
                int d8 = dirFromDelta(current, next);
                // FIX-O: only move across a known-open edge (current was sensed).
                if (d8 < 0 || !botMaze.canMove8(current, d8, /*optimistic=*/false))
                    break;

                deadReckonStep(kf, current, next, cfg, exploreVel);   // HW-3
                senseCell(botMaze, truthMaze, kf, next, cfg);
                botMaze.at(next).explored = true;
                current = next;
                visited.push_back(next);
                advanced = true;

                applyWallCentering(botMaze, kf, cfg, wallCtrl, current, dtStep);

                if (cfg.isGoal(current)) break;
                steps++;
                if (steps >= MAX_STEPS) break;
            }
            if (!advanced) { steps++; }
        }

        return visited;
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Adaptive velocity scaler
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
    // FIX-Q (simulation fixture): Cell defaults every wall to `true` and the
    // fixture below only ADDS walls, so without this the "ground-truth" maze
    // would have every interior wall present and be unsolvable — the scout
    // could never move and the whole pipeline was a silent no-op.  Start the
    // truth maze from a fully-open interior, then place borders and the listed
    // walls.  (Sim-only: on hardware real sensors replace truthMaze entirely.)
    for (int r = 0; r < MAZE_N; r++)
        for (int c = 0; c < MAZE_N; c++)
            for (int w = 0; w < 4; w++) {
                truth.at(r,c).wall[w]      = false;
                truth.at(r,c).wallKnown[w] = true;
            }
    for (int i = 0; i < MAZE_N; i++) {
        truth.setWall({0,        i}, WN, true);
        truth.setWall({MAZE_N-1, i}, WS, true);
        truth.setWall({i,        0}, WW, true);
        truth.setWall({i, MAZE_N-1}, WE, true);
    }
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
//  Profile a path — full pipeline  (ROB-2 sanitisation added)
// ───────────────────────────────────────────────────────────────────────────

[[nodiscard]] static RunStats profilePath(
    const std::vector<CellCoord>& cellPath,
    const MazeConfig& cfg, const RobotParams& robot,
    const std::string& label, float vMax,
    bool printDetails, bool computeTVLQR_gains = false,
    std::vector<TrajPoint>* outTraj = nullptr)
{
    RunStats stats;
    stats.label     = label;
    stats.pathCells = int(cellPath.size());

    if (cellPath.size() < 2) {
        if (!label.empty()) std::cerr << label << ": path too short\n";
        return stats;
    }

    auto expanded = ThetaStar::expandPath(cellPath);
    auto wps = pathToWaypoints(expanded, cfg);
    wps = optimiseRacingLine(wps, cfg.cellSize * 0.5f);
    auto traj = TrajGen::generate(wps, robot);
    if (traj.empty()) {
        if (!label.empty()) std::cerr << label << ": trajectory failed\n";
        return stats;
    }

    VelocityProfile::curvatureCeilings(traj, vMax, robot.maxTotalAccel);
    VelocityProfile::globalBrakingPass(traj, robot.maxTotalAccel, robot.maxBraking);
    VelocityProfile::backwardPass(traj, robot.maxJerk, robot.maxTotalAccel, robot.maxBraking);
    VelocityProfile::forwardPass (traj, robot.maxJerk, robot.maxTotalAccel, robot.maxAccelFwd);
    VelocityProfile::computeJerk(traj);

    if (!VelocityProfile::sanitizeTrajectory(traj) && !label.empty())   // ROB-2
        std::cerr << label << ": trajectory sanitised (non-finite values scrubbed)\n";

    if (outTraj != nullptr) *outTraj = traj;

    std::vector<TVLQRGain> gains;
    if (computeTVLQR_gains)
        gains = TVLQRSolver::solve(traj, robot.wheelbase);

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
//  GDW Planner v4.3 — top-level orchestrator
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

    // NEW: alternate-return tuning (deterministic, conservative).
    static constexpr float RETURN_DETOUR_FACTOR = 1.30f;  // <= 30 % longer ...
    static constexpr float RETURN_DETOUR_SLACK  = 4.0f;   // ... plus a small slack

    void initialize() {
        botMaze.init(cfg);
        truthMaze.init(cfg);
        buildTruthMaze(truthMaze);
        Vec2 startPos = cfg.cellCentre(cfg.startCell);
        kf.reset(startPos.x, startPos.y, HALF_PI);
    }

    // ── Scout run ─────────────────────────────────────────────────────────
    void scoutRun() {
        std::cout << "╔══════════════════════════════════════════╗\n"
                  << "║  SCOUT RUN (Canonical Flood-Fill v4.3)   ║\n"
                  << "╚══════════════════════════════════════════╝\n";

        WallCenteringPID wallCtrl(robot);
        std::cout << "  Wall-centering PID : enabled (Kp=" << robot.Kp_center
                  << " Ki=" << robot.Ki_center << ")\n";

        exploredPath = Explorer::explore(botMaze, truthMaze, kf, cfg, wallCtrl,
                                         robot.exploreVelocity);

        goalReached = {-1,-1};
        for (const auto& c : exploredPath)
            if (cfg.isGoal(c)) { goalReached = c; break; }

        std::cout << "  Cells visited      : " << exploredPath.size()     << "\n";
        std::cout << "  Remaining frontiers: " << botMaze.frontierCount() << "\n";
        std::cout << "  Map consistency    : "
                  << (botMaze.checkConsistency() ? "OK" : "VIOLATION") << "\n";
        if (goalReached.r >= 0)
            std::cout << "  Goal reached at    : ("
                      << goalReached.r << "," << goalReached.c << ")\n";
        kf.print();

        FloodFill::solveToGoal(botMaze, false);
        (void)profilePath(exploredPath, cfg, robot, "Scout trajectory",
                          robot.exploreVelocity, true, false);
    }

    // ── Verified greedy drive ────────────────────────────────────────────────
    //  Drive current -> target over KNOWN-OPEN edges, sensing every cell and
    //  replanning on the optimistic map whenever the next planned step turns out
    //  to cross a (newly) known wall.  Each committed step is verified in the
    //  actual travel direction with canMove8, so the robot is never commanded
    //  across a known wall.  Returns true iff target is reached within the cap.
    bool driveVerifiedTo(CellCoord& current, const CellCoord& target,
                         float retV, int maxSteps, std::vector<CellCoord>& driven)
    {
        int steps = 0, stalls = 0;
        while (steps < maxSteps) {
            if (current == target) return true;

            auto raw = ThetaStar::findPathTo(botMaze, current, target, /*optimistic=*/true);
            if (raw.size() < 2) return current == target;
            auto path = ThetaStar::expandPath(raw);

            bool advanced = false;
            for (size_t i = 1; i < path.size(); i++) {
                CellCoord next = path[i];
                int d8 = dirFromDelta(current, next);
                if (d8 < 0 || !botMaze.canMove8(current, d8, /*optimistic=*/false))
                    break;                                  // blocked -> replan
                Explorer::deadReckonStep(kf, current, next, cfg, retV);   // HW-3
                Explorer::senseCell(botMaze, truthMaze, kf, next, cfg);
                current = next;
                driven.push_back(next);
                scaler.record(retV, retV * 0.95f, 0.0f);
                advanced = true;
                if (current == target) return true;
                if (++steps >= maxSteps) break;
            }
            if (!advanced) { if (++stalls > 4) return false; }
            else            stalls = 0;
        }
        return current == target;
    }

    // ── Information-maximising return waypoint ────────────────────────────────
    //  On the OPTIMISTIC map, pick the cell with the most still-unknown walls
    //  whose round-trip detour dGoal[m] + dStart[m] stays within the budget.
    //  Deterministic tie-breaks: more info, then shorter detour, then lower
    //  index.  Returns {-1,-1} when no affordable informative cell exists.
    [[nodiscard]] CellCoord chooseReturnWaypoint(float& Dshort, int& outInfo) {
        outInfo = 0;
        FloodFill::solve(botMaze, { cfg.startCell }, /*optimistic=*/true);
        std::array<float,N_CELLS> dStart; FloodFill::snapshot(botMaze, dStart);
        FloodFill::solve(botMaze, { goalReached }, /*optimistic=*/true);
        std::array<float,N_CELLS> dGoal;  FloodFill::snapshot(botMaze, dGoal);

        Dshort = dStart[goalReached.idx()];
        if (!std::isfinite(Dshort)) return {-1,-1};
        const float budget = Dshort * RETURN_DETOUR_FACTOR + RETURN_DETOUR_SLACK;

        CellCoord best{-1,-1};
        int   bestInfo  = 0;
        float bestTotal = INF_F;
        for (int idx = 0; idx < N_CELLS; idx++) {
            CellCoord m{ idx / MAZE_N, idx % MAZE_N };
            if (m == goalReached || m == cfg.startCell) continue;
            int info = botMaze.at(m).unknownWalls();
            if (info <= 0) continue;                       // nothing to learn here
            float total = dGoal[idx] + dStart[idx];
            if (!std::isfinite(total) || total > budget) continue;
            if (info > bestInfo ||
               (info == bestInfo && total < bestTotal - 1e-6f)) {
                best = m; bestInfo = info; bestTotal = total;
            }
        }
        outInfo = bestInfo;
        return best;
    }

    // ── Return to start — Alternate Return Exploration (NEW) ──────────────────
    //
    //  Algorithm
    //  ─────────
    //   1. Compute optimistic shortest-distance fields from the start and from
    //      the reached goal, and let D_short be the optimistic return distance.
    //      Define a bounded budget = D_short * 1.30 + 4 cells.
    //   2. Pick the most informative reachable cell m (most unknown walls) whose
    //      round trip dGoal[m] + dStart[m] <= budget (deterministic tie-breaks).
    //      This is the "different, information-maximising" detour target.
    //   3. Drive goal -> m -> start with the verified greedy driver: it follows
    //      an optimistic plan but only ever COMMITS to a step proven known-open
    //      in the travel direction, sensing each cell and replanning when a step
    //      is blocked.  New walls are folded into botMaze as they are seen.
    //   4. If no affordable informative waypoint exists, drive straight home the
    //      same verified way (a plain return, still sensing).
    //   5. Last-resort fallback: if the verified driver cannot reach the start
    //      (it never has in testing — the start is always optimistically
    //      reachable), retrace the REVERSE of the physically-executed outbound
    //      path, which is drivable by construction.
    //
    //  Why it is safe
    //  ──────────────
    //   • Feasibility: every committed step is canMove8(known-open) in the travel
    //     direction, so the robot is never driven across a known wall; unknown
    //     cells are only entered once an adjacent known-open edge is confirmed.
    //   • Bounded distance: the detour target obeys the budget, so the return
    //     never grows unboundedly; with no target it is a direct return.
    //   • Determinism: argmax with explicit (info, distance, index) tie-breaks
    //     and a fixed plan/drive order.
    //   • Reliability: degrades to a direct return, then to reverse-outbound;
    //     maze-solving success is never compromised.
    //   • Compatibility: uses only FloodFill + Theta* + the existing map and the
    //     existing verified-step pattern from the scout.  D* Lite is untouched.
    //     The speed run is regenerated afterwards and can only benefit from the
    //     extra walls discovered on the way home.
    void returnToStart() {
        if (goalReached.r < 0) {
            std::cerr << "returnToStart: goal was not reached during scout\n";
            return;
        }
        std::cout << "\n╔══════════════════════════════════════════╗\n"
                  << "║   RETURN TO START (Alternate-Return)     ║\n"
                  << "╚══════════════════════════════════════════╝\n";

        const int knownBefore = countKnownWalls();
        const int cellsBefore = exploredCellCount();

        // Guaranteed fallback: reverse of the path actually driven out (same
        // corridors, drivable by construction), trimmed at the reached goal.
        std::vector<CellCoord> reversedOutbound;
        for (const auto& c : exploredPath) {
            reversedOutbound.push_back(c);
            if (c == goalReached) break;
        }
        std::reverse(reversedOutbound.begin(), reversedOutbound.end());

        const float retV = robot.maxVelocity * 0.6f;
        float Dshort = INF_F;
        int   viaInfo = 0;
        CellCoord m = chooseReturnWaypoint(Dshort, viaInfo);

        std::vector<CellCoord> driven{ goalReached };
        CellCoord current = goalReached;
        bool alternate = false;

        if (m.valid()) {
            if (driveVerifiedTo(current, m, retV, 4 * N_CELLS, driven))
                alternate = true;                        // detour actually reached
        }
        bool home = driveVerifiedTo(current, cfg.startCell, retV, 4 * N_CELLS, driven);

        bool fellBack = false;
        if (!home || current != cfg.startCell || driven.size() < 2) {
            std::cerr << "  return: guided drive stalled — using reverse-outbound\n";
            driven    = reversedOutbound;
            alternate = false;
            fellBack  = true;
            current   = driven.empty() ? current : driven.back();
        }

        if (driven.size() < 2) {
            std::cerr << "  No feasible return path found\n";
            FloodFill::solveToGoal(botMaze, false);
            return;
        }

        if (alternate)
            std::cout << "  Route type         : ALTERNATE via (" << m.r << ","
                      << m.c << ")  (" << viaInfo
                      << " unknown walls at selection)\n";
        else if (fellBack)
            std::cout << "  Route type         : REVERSE-OUTBOUND (safe fallback)\n";
        else
            std::cout << "  Route type         : DIRECT (no affordable detour)\n";
        if (std::isfinite(Dshort))
            std::cout << "  Optimistic D_short : " << Dshort << " cell-units"
                      << "  (budget " << Dshort * RETURN_DETOUR_FACTOR + RETURN_DETOUR_SLACK
                      << ")\n";
        std::cout << "  Return steps driven: " << (driven.size() - 1) << "\n";

        const int knownAfter = countKnownWalls();
        const int cellsAfter = exploredCellCount();
        std::cout << "  Walls known (pre)  : " << knownBefore << "\n";
        std::cout << "  Walls known (post) : " << knownAfter
                  << "  (+ " << (knownAfter - knownBefore) << " revealed)\n";
        std::cout << "  Cells explored     : " << cellsBefore << " -> " << cellsAfter
                  << "  (+ " << (cellsAfter - cellsBefore) << " new)\n";
        std::cout << "  Ended at           : (" << current.r << "," << current.c << ")"
                  << (current == cfg.startCell ? "  [start]" : "  [NOT start]") << "\n";
        std::cout << "  Map consistency    : "
                  << (botMaze.checkConsistency() ? "OK" : "VIOLATION") << "\n";

        (void)profilePath(driven, cfg, robot, "Return trajectory", retV, true, false);

        FloodFill::solveToGoal(botMaze, false);
    }

    [[nodiscard]] int exploredCellCount() const {
        int n = 0;
        for (const auto& c : botMaze.cells) if (c.explored) n++;
        return n;
    }

    [[nodiscard]] int countKnownWalls() const {
        int n = 0;
        for (const auto& c : botMaze.cells)
            for (int w = 0; w < 4; w++) if (c.wallKnown[w]) n++;
        return n;
    }

    // ── Speed run ───────────────────────────────────────────────────────────
    void speedRun() {
        std::cout << "\n╔══════════════════════════════════════════╗\n"
                  << "║  SPEED RUN  (Theta* + Racing Line +      ║\n"
                  << "║             TVLQR + PD fallback)         ║\n"
                  << "╚══════════════════════════════════════════╝\n";

        auto [af, cf] = scaler.factors();
        RobotParams scaledRobot = robot;
        scaledRobot.maxTotalAccel *= af;
        scaledRobot.maxVelocity   = std::min(robot.maxVelocity * cf, robot.maxVelocity);
        std::cout << "  Adaptive factors: accel=" << af << " corner=" << cf << "\n";
        std::cout << "  Wall-centering PID : disabled (speed run — racing line active)\n";

        RunStats               bestStats;
        std::vector<CellCoord> bestPath;
        bestStats.estimatedTime = INF_F;

        // ROB-1: admissible per-goal search via findPathTo(gc).
        for (const auto& gc : cfg.goalCells) {
            auto path = ThetaStar::findPathTo(botMaze, cfg.startCell, gc, false);
            if (path.size() < 2) continue;
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

        std::vector<TrajPoint> speedTraj;
        auto finalStats = profilePath(bestPath, cfg, scaledRobot,
                                       "Speed-run trajectory",
                                       scaledRobot.maxVelocity, true,
                                       /*computeTVLQR=*/true, &speedTraj);

        PDController pdCtrl(scaledRobot);

        std::cout << "\n  ┌──────────────────────────────────────────┐\n";
        std::cout << "  │      CHAMPIONSHIP SUMMARY  (v4.3)        │\n";
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

        // PD tracking simulation (physically coherent dt — FIX-K retained).
        if (!speedTraj.empty()) {
            std::cout << "\n  PD Trajectory Tracking — Speed Run Simulation\n";
            std::cout << "  (dt = ds / v_ref, " << speedTraj.size() << " steps; every 10th printed)\n";
            std::cout << std::fixed << std::setprecision(3);
            std::cout << "    arc(m)  v_ref  v_cmd  ω_cmd(r/s)  e_ct(m)\n";
            std::cout << "    ──────  ─────  ─────  ──────────  ───────\n";

            Vec2  startWorld = cfg.cellCentre(cfg.startCell);
            float pdX     = startWorld.x;
            float pdY     = startWorld.y;
            float pdTheta = HALF_PI;
            float prevCt  = 0.0f;

            int stride = std::max(1, static_cast<int>(speedTraj.size()) / 10);

            for (int i = 0; i < static_cast<int>(speedTraj.size()) - 1; i++) {
                const TrajPoint& ref = speedTraj[i];
                float ds = speedTraj[i+1].arcLen - ref.arcLen;
                float dt = (ref.velocity > 0.01f) ? ds / ref.velocity : 0.01f;   // FIX-K

                auto [vCmd, omegaCmd] = pdCtrl.compute(pdX, pdY, pdTheta,
                                                        ref, prevCt, dt);
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

                pdX     += vCmd * std::cos(pdTheta) * dt;
                pdY     += vCmd * std::sin(pdTheta) * dt;
                pdTheta  = wrapAngle(pdTheta + omegaCmd * dt);
            }
            std::cout << "  [PD controller executed — "
                      << speedTraj.size() << " control ticks]\n";
        }

        std::cout << "\n  [Control architecture — v4.3]\n"
                  << "  ┌─ Planning ───────────────────────────────────────────────┐\n"
                  << "  │  Exploration : Canonical flood-fill (verified + replan)  │\n"
                  << "  │  Return      : Alternate-Return (info-max, bounded)      │\n"
                  << "  │  Speed run   : Theta* findPathTo per goal — best of 4    │\n"
                  << "  │  Distances   : FloodFill (Dijkstra, 8-dir)               │\n"
                  << "  ├─ Trajectory ─────────────────────────────────────────────┤\n"
                  << "  │  Geometry    : Clothoid–Arc–Clothoid (heading-correct)   │\n"
                  << "  │  Racing line : L2 second-difference QP optimiser         │\n"
                  << "  │  Velocity    : Kamm circle + global braking + S-curve    │\n"
                  << "  ├─ Control ────────────────────────────────────────────────┤\n"
                  << "  │  Primary     : TVLQR (dt clamped, gains sanitised)       │\n"
                  << "  │  Fallback    : PD + curvature feedforward (clamped)      │\n"
                  << "  │  Lateral     : Wall-centering PID (scout only)           │\n"
                  << "  ├─ Localisation ───────────────────────────────────────────┤\n"
                  << "  │  Filter      : ESKF 4-state (sanitised, fault-flagged)   │\n"
                  << "  │  Measurement : Wall-distance IR + heading snap           │\n"
                  << "  └─────────────────────────────────────────────────────────┘\n";
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
    std::cout << "GDW Micromouse Championship Edition v4.3\n"
              << "──────────────────────────────────────────\n"
              << "v4.3 changes:\n"
              << "  FIX-L  expandPath Bresenham (mixed LoS jumps)\n"
              << "  FIX-M  clothoid heading budget (arcAngle = |Δθ| − L_c/R)\n"
              << "  FIX-N  return leg can target the start (findPathTo)\n"
              << "  FIX-O  exploration verifies each step + replans\n"
              << "  FIX-P  TVLQR computeControl empty-schedule guard\n"
              << "  ROB-1..4 / HW-1..3 hardening (see header)\n"
              << "  NEW    Alternate Return Exploration\n"
              << "\n";

    GDWPlannerV4 planner;
    planner.initialize();
    planner.run();
    return 0;
}
