// ═══════════════════════════════════════════════════════════════════════════
//  LNR Laberinto Edition v1.0
//  Adapted from GDW Micromouse Championship Edition v4.3
//  C++17 · Single translation unit
//
//  Rules reference: Reglamento Laberinto — Liga Nacional de Robótica (LNR)
//
//  Adaptation inventory (LAB-*) — all v4.3 bug-fixes (FIX-*/ROB-*/HW-*)
//  are fully preserved unless noted:
//
//  LAB-1  Variable maze dimensions: runtime rows×cols (min 4×5), 25 cm/cell.
//          Global LNRGrid namespace replaces compile-time MAZE_N.
//          MAX_CELLS = MAX_MAZE_N² used for static array sizing.
//
//  LAB-2  Single goal cell (piso blanco, §4), unknown until the robot
//          physically enters it and reads the floor sensor.
//          MazeConfig::goalCells[] replaced by CellCoord goalCell.
//          Explorer sets botMaze.cfg->goalCell on first entry.
//
//  LAB-3  Start cell (§4): a cell with exactly 3 walls, placed by the jury.
//          validateStartCell() checks wall count; initial heading derived
//          from the one open passage.
//
//  LAB-4  Penalty system (§7): robot in same cell ≥10 s → penalty + restart;
//          representative requests restart → penalty.
//          After each penalty, 1-min window to restart from startCell.
//          Simulated: if planner stalls (no path), 10 s are charged.
//
//  LAB-5  Round-loss conditions (§8): ≥3 penalties, piece detachment,
//          unauthorized entry.  Also (§6): 15 s immobility or wall-push
//          stops the trial entirely.  Simulated via stall-count tracking.
//
//  LAB-6  Official timer (§5.3.9): starts on jury go-signal; stops when robot
//          is completely inside the goal cell (white floor detected).
//          Time accumulated from velocity-profile estimates.
//
//  LAB-7  Two configurations per event; two rounds per configuration (§5.1).
//          Round 1 = exploration (map unknown).
//          Round 2 = speed run reusing map from Round 1 (robot not modified
//          between rounds — §5.3.3 — so internal map is retained).
//          Best-round selection per §5.2.1; final ranking per §5.2.2.
//
//  LAB-8  30-minute adjustment window between configurations (§5.3.6).
//          Simulation: robot params may be re-tuned; map is wiped for new config.
//
//  LAB-9  Robot submitted to jury after free-practice period (§5.3.3).
//          No modification until the 30-min inter-configuration break.
//
//  LAB-10 Cell size 18 cm → 25 cm; exploreVelocity and maxStep rescaled.
//
// ═══════════════════════════════════════════════════════════════════════════

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <queue>
#include <random>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

// ───────────────────────────────────────────────────────────────────────────
//  Constants
// ───────────────────────────────────────────────────────────────────────────

inline constexpr int   MAX_MAZE_N  = 32;
inline constexpr int   MAX_CELLS   = MAX_MAZE_N * MAX_MAZE_N;   // 1024
inline constexpr float INF_F       = std::numeric_limits<float>::infinity();
inline constexpr float PI          = 3.14159265358979f;
inline constexpr float TWO_PI      = 2.0f * PI;
inline constexpr float HALF_PI     = PI * 0.5f;
inline constexpr float SQRT2       = 1.41421356237f;

inline constexpr float GYRO_MAX_RATE  = 34.9f;    // BMI088 ±2000 dps (HW-1)
inline constexpr float MAX_STEP_DS    = 2.0f;     // LAB-10: generous for 25 cm cell
inline constexpr float OMEGA_LIMIT    = 25.0f;    // rad/s (HW-2)
inline constexpr float DELTA_V_LIMIT  = 6.0f;     // m/s  (HW-2)

// LNR timing rules (§6, §7, §8)
inline constexpr float LNR_STALL_PENALTY_S   = 10.0f;  // §7: stall → penalty
inline constexpr float LNR_IMMOBILITY_STOP_S = 15.0f;  // §6: immobility → stop trial
inline constexpr float LNR_MAX_PENALTIES      = 3;      // §8: ≥3 → round lost
inline constexpr float LNR_RESTART_WINDOW_S   = 60.0f;  // §7: 1 min to restart
inline constexpr float LNR_EXTRA_TIME_S       = 300.0f; // §6: 5-min extra time (once)

// ───────────────────────────────────────────────────────────────────────────
//  LAB-1: Runtime maze geometry (set via LNRGrid::configure before each config)
// ───────────────────────────────────────────────────────────────────────────
namespace LNRGrid {
    inline int rows  = 8;
    inline int cols  = 10;
    inline int total = 80;
    inline void configure(int r, int c) {
        assert(r >= 4 && c >= 5 && r <= MAX_MAZE_N && c <= MAX_MAZE_N);
        rows = r; cols = c; total = r * c;
    }
}

// ───────────────────────────────────────────────────────────────────────────
//  Angle utilities (unchanged)
// ───────────────────────────────────────────────────────────────────────────
[[nodiscard]] inline float wrapAngle(float a) noexcept {
    a = std::fmod(a + PI, TWO_PI);
    if (a < 0.0f) a += TWO_PI;
    return a - PI;
}
[[nodiscard]] inline float angleDiff(float a, float b) noexcept { return wrapAngle(a - b); }
[[nodiscard]] inline float clampf(float v, float lo, float hi) noexcept {
    return std::max(lo, std::min(hi, v));
}

// ───────────────────────────────────────────────────────────────────────────
//  Wall / direction encoding (unchanged)
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
    { WN,-1 },{ WN,WE },{ WE,-1 },{ WE,WS },
    { WS,-1 },{ WS,WW },{ WW,-1 },{ WN,WW }
};
inline constexpr float D8HEADING[8] = {
    HALF_PI, PI*0.25f, 0.0f, -PI*0.25f,
    -HALF_PI, -(PI*0.75f), PI, PI*0.75f
};

// ───────────────────────────────────────────────────────────────────────────
//  CellCoord  (LAB-1: idx() and valid() use LNRGrid)
// ───────────────────────────────────────────────────────────────────────────
struct CellCoord {
    int r = 0, c = 0;
    [[nodiscard]] bool operator==(const CellCoord& o) const noexcept { return r==o.r && c==o.c; }
    [[nodiscard]] bool operator!=(const CellCoord& o) const noexcept { return !(*this==o); }
    [[nodiscard]] bool operator< (const CellCoord& o) const noexcept {
        return (r!=o.r) ? r<o.r : c<o.c;
    }
    // LAB-1: runtime cols from LNRGrid
    [[nodiscard]] int  idx()   const noexcept { return r * LNRGrid::cols + c; }
    [[nodiscard]] bool valid() const noexcept {
        return r>=0 && r<LNRGrid::rows && c>=0 && c<LNRGrid::cols;
    }
    [[nodiscard]] CellCoord neighbour(int w) const noexcept {
        return { r+WALL_DR[w], c+WALL_DC[w] };
    }
    [[nodiscard]] CellCoord step8(int d8) const noexcept {
        return { r+D8R[d8], c+D8C[d8] };
    }
};
[[nodiscard]] inline int dirFromDelta(const CellCoord& a, const CellCoord& b) noexcept {
    int dr=b.r-a.r, dc=b.c-a.c;
    for (int d8=0; d8<8; d8++)
        if (D8R[d8]==dr && D8C[d8]==dc) return d8;
    return -1;
}

// ───────────────────────────────────────────────────────────────────────────
//  Vec2 (unchanged)
// ───────────────────────────────────────────────────────────────────────────
struct Vec2 {
    float x=0, y=0;
    [[nodiscard]] Vec2  operator+(const Vec2& o) const noexcept { return {x+o.x,y+o.y}; }
    [[nodiscard]] Vec2  operator-(const Vec2& o) const noexcept { return {x-o.x,y-o.y}; }
    [[nodiscard]] Vec2  operator*(float s)       const noexcept { return {x*s,y*s}; }
    [[nodiscard]] float dot  (const Vec2& o)     const noexcept { return x*o.x+y*o.y; }
    [[nodiscard]] float cross(const Vec2& o)     const noexcept { return x*o.y-y*o.x; }
    [[nodiscard]] float norm()                   const noexcept { return std::sqrt(x*x+y*y); }
    [[nodiscard]] Vec2  normalised()             const noexcept {
        float n=norm(); return n>1e-9f ? Vec2{x/n,y/n} : Vec2{};
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  MazeConfig  (LAB-1: rows/cols; LAB-2: single goalCell; LAB-3: 3-wall start)
// ───────────────────────────────────────────────────────────────────────────
struct MazeConfig {
    int   rows     = 8;
    int   cols     = 10;
    float cellSize = 0.25f;      // LAB-10: 25 cm per §4

    CellCoord startCell = { 7, 0 };
    CellCoord goalCell  = {-1,-1}; // LAB-2: unknown at start; set when white floor found

    // Maximum time for this configuration, set by jury (§5.1.1)
    float maxTimeSec = 300.0f;

    // Sync LNRGrid globals (call before any maze operations for this config)
    void sync() const { LNRGrid::configure(rows, cols); }

    [[nodiscard]] bool isGoalKnown() const noexcept { return goalCell.r >= 0; }
    [[nodiscard]] bool isGoal(const CellCoord& cc) const noexcept {
        return isGoalKnown() && cc == goalCell;
    }
    [[nodiscard]] bool valid(int r, int c)          const noexcept {
        return r>=0 && r<rows && c>=0 && c<cols;
    }
    [[nodiscard]] bool valid(const CellCoord& cc)   const noexcept {
        return valid(cc.r, cc.c);
    }
    [[nodiscard]] Vec2 cellCentre(const CellCoord& cc) const noexcept {
        return { (cc.c+0.5f)*cellSize, -(cc.r+0.5f)*cellSize };
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Cell (unchanged from v4.3)
// ───────────────────────────────────────────────────────────────────────────
struct Cell {
    std::array<bool,4> wallKnown = {false,false,false,false};
    std::array<bool,4> wall      = {true, true, true, true };
    bool  isGoalCell  = false;   // LAB-2: white floor flag (truth only)
    bool  explored    = false;
    int   visitCount  = 0;
    float floodDist   = INF_F;
    float dstar_g     = INF_F;
    float dstar_rhs   = INF_F;

    [[nodiscard]] bool passableOpt (int w) const noexcept { return !(wallKnown[w]&&wall[w]); }
    [[nodiscard]] bool passableCons(int w) const noexcept { return wallKnown[w]&&!wall[w]; }
    [[nodiscard]] bool hasFrontier()       const noexcept {
        for (int w=0;w<4;w++) if (!wallKnown[w]) return true; return false;
    }
    [[nodiscard]] int unknownWalls()       const noexcept {
        int n=0; for (int w=0;w<4;w++) if (!wallKnown[w]) n++; return n;
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Maze  (LAB-1: MAX_CELLS array; loops use LNRGrid)
// ───────────────────────────────────────────────────────────────────────────
class Maze {
public:
    std::array<Cell,MAX_CELLS> cells{};
    const MazeConfig* cfg = nullptr;

    void init(const MazeConfig& c) {
        cfg = &c;
        c.sync();
        cells.fill(Cell{});
        placeBorderWalls();
    }

    [[nodiscard]] Cell& at(const CellCoord& cc) noexcept       { return cells[cc.idx()]; }
    [[nodiscard]] const Cell& at(const CellCoord& cc) const noexcept { return cells[cc.idx()]; }
    [[nodiscard]] Cell& at(int r, int c) noexcept              { return cells[r*LNRGrid::cols+c]; }
    [[nodiscard]] const Cell& at(int r, int c) const noexcept  { return cells[r*LNRGrid::cols+c]; }

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
        for (int k=0;k<2;k++) {
            int w = D8WALLS[d8][k];
            if (w<0) continue;
            if (optimistic) { if (cell.wallKnown[w]&&cell.wall[w]) return false; }
            else            { if (!cell.wallKnown[w]||cell.wall[w]) return false; }
        }
        return true;
    }
    [[nodiscard]] bool canMoveCardinal(const CellCoord& cc, int w, bool optimistic) const noexcept {
        CellCoord nb = cc.neighbour(w);
        if (!cfg->valid(nb)) return false;
        const Cell& cell = at(cc);
        if (optimistic) return !(cell.wallKnown[w]&&cell.wall[w]);
        return cell.wallKnown[w]&&!cell.wall[w];
    }

    [[nodiscard]] int frontierCount() const noexcept {
        int n=0;
        for (int i=0;i<LNRGrid::total;i++) if (cells[i].explored&&cells[i].hasFrontier()) n++;
        return n;
    }
    [[nodiscard]] bool checkConsistency() const noexcept {
        for (int r=0;r<LNRGrid::rows;r++)
            for (int c=0;c<LNRGrid::cols;c++) {
                CellCoord cc{r,c};
                for (int w=0;w<4;w++) {
                    CellCoord nb = cc.neighbour(w);
                    if (!cfg->valid(nb)) continue;
                    const Cell& a=at(cc), &b=at(nb);
                    if (a.wallKnown[w]&&b.wallKnown[WALL_OPP[w]]&&a.wall[w]!=b.wall[WALL_OPP[w]])
                        return false;
                }
            }
        return true;
    }

private:
    void placeBorderWalls() {
        for (int c=0;c<LNRGrid::cols;c++) {
            setWall({0, c},              WN, true);
            setWall({LNRGrid::rows-1,c}, WS, true);
        }
        for (int r=0;r<LNRGrid::rows;r++) {
            setWall({r, 0},              WW, true);
            setWall({r, LNRGrid::cols-1},WE, true);
        }
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  FloodFill — goal-seeded Dijkstra (LAB-1: loops use LNRGrid; LAB-2: seeds
//  come from the single known goalCell instead of a fixed array)
// ───────────────────────────────────────────────────────────────────────────
class FloodFill {
public:
    static void solve(Maze& maze,
                      const std::vector<CellCoord>& seeds,
                      bool optimistic)
    {
        for (int i=0;i<LNRGrid::total;i++) maze.cells[i].floodDist = INF_F;
        using Entry = std::pair<float,CellCoord>;
        std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> pq;
        for (const auto& s : seeds) {
            if (!maze.cfg->valid(s)) continue;
            maze.at(s).floodDist = 0.0f;
            pq.push({0.0f,s});
        }
        while (!pq.empty()) {
            auto [d,cc] = pq.top(); pq.pop();
            if (d > maze.at(cc).floodDist+1e-6f) continue;
            for (int d8=0;d8<8;d8++) {
                if (!maze.canMove8(cc,d8,optimistic)) continue;
                CellCoord nb = cc.step8(d8);
                float nd = d+D8COST[d8];
                if (nd < maze.at(nb).floodDist-1e-6f) {
                    maze.at(nb).floodDist = nd;
                    pq.push({nd,nb});
                }
            }
        }
    }

    // LAB-2: seed from single goalCell if known, else no-op
    static void solveToGoal(Maze& maze, bool optimistic) {
        if (!maze.cfg->isGoalKnown()) {
            for (int i=0;i<LNRGrid::total;i++) maze.cells[i].floodDist = INF_F;
            return;
        }
        solve(maze, { maze.cfg->goalCell }, optimistic);
    }
    static void solveToStart(Maze& maze, bool optimistic) {
        solve(maze, { maze.cfg->startCell }, optimistic);
    }
    static void snapshot(const Maze& maze, std::array<float,MAX_CELLS>& out) {
        for (int i=0;i<LNRGrid::total;i++) out[i] = maze.cells[i].floodDist;
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Theta* — any-angle A*
//  (LAB-1: arrays use MAX_CELLS; LAB-2: isGoal uses single goalCell;
//   FIX-L/FIX-N from v4.3 retained)
// ───────────────────────────────────────────────────────────────────────────
class ThetaStar {
public:
    [[nodiscard]] static bool lineOfSight(const Maze& maze,
                                           const CellCoord& a,
                                           const CellCoord& b,
                                           bool optimistic) noexcept
    {
        int r0=a.r,c0=a.c,r1=b.r,c1=b.c;
        int dr=std::abs(r1-r0),dc=std::abs(c1-c0);
        int sr=(r1>r0)?1:-1, sc=(c1>c0)?1:-1;
        int r=r0,c=c0,err=dc-dr;
        for (int step=0;step<=dr+dc;step++) {
            if (r==r1&&c==c1) return true;
            if (!maze.cfg->valid(r,c)) return false;
            int e2=2*err;
            bool mC=(e2>-dr),mR=(e2<dc);
            CellCoord cc{r,c};
            if (mC&&mR) {
                int wC=(sc>0)?WE:WW, wR=(sr>0)?WS:WN;
                if (!checkWall(maze,cc,wC,optimistic)) return false;
                if (!checkWall(maze,cc,wR,optimistic)) return false;
                c+=sc; r+=sr; err+=dr-dc;
            } else if (mC) { if (!checkWall(maze,cc,(sc>0)?WE:WW,optimistic)) return false; c+=sc; err+=dr; }
            else            { if (!checkWall(maze,cc,(sr>0)?WS:WN,optimistic)) return false; r+=sr; err-=dc; }
        }
        return true;
    }
    [[nodiscard]] static float dist(const CellCoord& a, const CellCoord& b) noexcept {
        float dr=float(b.r-a.r),dc=float(b.c-a.c);
        return std::sqrt(dr*dr+dc*dc);
    }
    [[nodiscard]] static float octile(const CellCoord& a, const CellCoord& b) noexcept {
        float dr=float(std::abs(a.r-b.r)),dc=float(std::abs(a.c-b.c));
        return std::max(dr,dc)+(SQRT2-1.0f)*std::min(dr,dc);
    }

    // LAB-2: plan toward the single known goal cell via flood-fill heuristic
    [[nodiscard]] static std::vector<CellCoord> findPath(
        const Maze& maze, const CellCoord& start, bool optimistic)
    {
        if (!maze.cfg->isGoalKnown()) return {};
        return search(maze, start, optimistic,
                      [&](const CellCoord& cc){ return maze.cfg->isGoal(cc); },
                      [&](const CellCoord& cc){ return maze.at(cc).floodDist; });
    }

    // FIX-N: plan toward an arbitrary target with octile heuristic (return leg, etc.)
    [[nodiscard]] static std::vector<CellCoord> findPathTo(
        const Maze& maze, const CellCoord& start,
        const CellCoord& target, bool optimistic)
    {
        return search(maze, start, optimistic,
                      [&](const CellCoord& cc){ return cc==target; },
                      [&](const CellCoord& cc){ return octile(cc,target); });
    }

    // FIX-L: Bresenham cell walk (excl. a, incl. b)
    static void bresenhamCells(const CellCoord& a, const CellCoord& b,
                                std::vector<CellCoord>& out)
    {
        int r0=a.r,c0=a.c,r1=b.r,c1=b.c;
        int dr=std::abs(r1-r0),dc=std::abs(c1-c0);
        int sr=(r1>r0)?1:-1, sc=(c1>c0)?1:-1;
        int r=r0,c=c0,err=dc-dr;
        for (int step=0;step<=dr+dc;step++) {
            if (r==r1&&c==c1) { out.push_back({r,c}); break; }
            int e2=2*err;
            bool mC=(e2>-dr),mR=(e2<dc);
            if (mC&&mR)   { c+=sc; r+=sr; err+=dr-dc; }
            else if (mC)  { c+=sc; err+=dr; }
            else          { r+=sr; err-=dc; }
            out.push_back({r,c});
        }
    }
    [[nodiscard]] static std::vector<CellCoord> expandPath(const std::vector<CellCoord>& path) {
        std::vector<CellCoord> expanded;
        if (path.empty()) return expanded;
        expanded.push_back(path[0]);
        for (size_t i=1;i<path.size();i++) bresenhamCells(path[i-1],path[i],expanded);
        return expanded;
    }

private:
    template<class GoalTest, class Heur>
    [[nodiscard]] static std::vector<CellCoord> search(
        const Maze& maze, const CellCoord& start, bool optimistic,
        GoalTest isGoal, Heur h)
    {
        std::array<float,     MAX_CELLS> gCost;  gCost.fill(INF_F);
        std::array<CellCoord, MAX_CELLS> parent; parent.fill({-1,-1});
        std::array<bool,      MAX_CELLS> closed; closed.fill(false);

        if (!maze.cfg->valid(start)) return {};
        if (isGoal(start)) return {start};

        gCost[start.idx()]  = 0.0f;
        parent[start.idx()] = {-2,-2};

        struct Node { float f; CellCoord cc;
            bool operator>(const Node& o) const noexcept { return f>o.f; } };
        std::priority_queue<Node,std::vector<Node>,std::greater<Node>> open;
        open.push({h(start),start});
        CellCoord reached{-1,-1};

        while (!open.empty()) {
            auto [f,cc] = open.top(); open.pop();
            if (closed[cc.idx()]) continue;
            closed[cc.idx()] = true;
            if (isGoal(cc)) { reached=cc; break; }

            for (int d8=0;d8<8;d8++) {
                if (!maze.canMove8(cc,d8,optimistic)) continue;
                CellCoord nb = cc.step8(d8);
                if (closed[nb.idx()]) continue;
                const CellCoord& par = parent[cc.idx()];
                bool hasPar = (par.r>=0);
                float ng; CellCoord via;
                if (hasPar && lineOfSight(maze,par,nb,optimistic)) {
                    ng = gCost[par.idx()]+dist(par,nb); via=par;
                } else { ng = gCost[cc.idx()]+D8COST[d8]; via=cc; }
                if (ng < gCost[nb.idx()]-1e-6f) {
                    gCost[nb.idx()]  = ng;
                    parent[nb.idx()] = via;
                    float hv = h(nb);
                    open.push({ng+(std::isfinite(hv)?hv:0.0f),nb});
                }
            }
        }
        if (reached.r<0) return {};
        std::vector<CellCoord> path;
        CellCoord c=reached;
        while (c.r>=0) { path.push_back(c); const CellCoord& p=parent[c.idx()]; if(p.r==-2) break; c=p; }
        std::reverse(path.begin(),path.end());
        return path;
    }
    [[nodiscard]] static bool checkWall(const Maze& m, const CellCoord& cc,
                                         int w, bool optimistic) noexcept {
        if (!m.cfg->valid(cc)) return false;
        const Cell& cell = m.at(cc);
        if (optimistic) return !(cell.wallKnown[w]&&cell.wall[w]);
        return cell.wallKnown[w]&&!cell.wall[w];
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  SensorModel (unchanged)
// ───────────────────────────────────────────────────────────────────────────
struct SensorReading { float distance=0; bool valid=false; };
class SensorModel {
public:
    static constexpr float RANGE_MIN = 0.04f;
    static constexpr float RANGE_MAX = 0.40f;    // LAB-10: wider range for 25 cm cell
    static constexpr float NOISE_STD = 0.003f;
    static constexpr float NOISE_VAR = NOISE_STD*NOISE_STD;
    [[nodiscard]] SensorReading sample(float trueDist, std::mt19937& rng) const {
        if (trueDist<RANGE_MIN||trueDist>RANGE_MAX) return {trueDist,false};
        std::normal_distribution<float> noise{0.0f,NOISE_STD};
        float m = trueDist+noise(rng);
        return (m<RANGE_MIN||m>RANGE_MAX) ? SensorReading{m,false} : SensorReading{m,true};
    }
    [[nodiscard]] static float measurementVariance() noexcept { return NOISE_VAR; }
};

// ───────────────────────────────────────────────────────────────────────────
//  ESKF — Error-State Kalman Filter (unchanged from v4.3)
// ───────────────────────────────────────────────────────────────────────────
class ESKF {
public:
    float nom_x=0,nom_y=0,nom_theta=0;
    std::array<float,4>  err{0,0,0,0};
    std::array<float,16> P{};
    float Q_xy=1e-5f, Q_theta=2e-4f, Q_bias=1e-6f;
    float R_wall=SensorModel::NOISE_VAR, R_hdg=1e-4f;
    bool faultEncoder=false, faultGyro=false;

    ESKF() { P.fill(0); P[0]=0.01f; P[5]=0.01f; P[10]=0.001f; P[15]=1e-4f; }
    void reset(float x0,float y0,float h0) noexcept {
        nom_x=x0; nom_y=y0; nom_theta=h0; err.fill(0);
        P.fill(0); P[0]=1e-4f; P[5]=1e-4f; P[10]=1e-4f; P[15]=1e-6f;
        faultEncoder=faultGyro=false;
    }
    [[nodiscard]] float x()     const noexcept { return nom_x+err[0]; }
    [[nodiscard]] float y()     const noexcept { return nom_y+err[1]; }
    [[nodiscard]] float theta() const noexcept { return nom_theta+err[2]; }
    [[nodiscard]] float bias()  const noexcept { return err[3]; }
    float& pij(int i,int j) noexcept       { return P[i*4+j]; }
    float  pij(int i,int j) const noexcept { return P[i*4+j]; }

    void sanitize() noexcept {  // ROB-3
        if (!std::isfinite(nom_x))     nom_x=0;
        if (!std::isfinite(nom_y))     nom_y=0;
        if (!std::isfinite(nom_theta)) nom_theta=0;
        for (int i=0;i<4;i++)  if (!std::isfinite(err[i])) err[i]=0;
        for (int i=0;i<16;i++) if (!std::isfinite(P[i]))   P[i]=0;
        for (int d=0;d<4;d++)  { int k=d*4+d; if (P[k]<1e-9f) P[k]=1e-9f; }
    }
    void predict(float ds, float dtheta_meas, float dt) noexcept {
        if (!std::isfinite(ds)||std::abs(ds)>MAX_STEP_DS) {
            faultEncoder=true; ds=clampf(std::isfinite(ds)?ds:0,-MAX_STEP_DS,MAX_STEP_DS);
        }
        if (dt>1e-6f) {
            float maxDt=GYRO_MAX_RATE*dt;
            if (!std::isfinite(dtheta_meas)||std::abs(dtheta_meas)>maxDt) {
                faultGyro=true; dtheta_meas=clampf(std::isfinite(dtheta_meas)?dtheta_meas:0,-maxDt,maxDt);
            }
        }
        float dtheta=dtheta_meas-err[3]*dt;
        float midTheta=nom_theta+0.5f*dtheta;
        float c=std::cos(midTheta),s_t=std::sin(midTheta);
        nom_x+=ds*c; nom_y+=ds*s_t; nom_theta=wrapAngle(nom_theta+dtheta);
        std::array<float,16> Pn=P;
        for (int j=0;j<4;j++) Pn[0*4+j]+=(-ds*s_t)*P[2*4+j];
        for (int j=0;j<4;j++) Pn[1*4+j]+=(ds*c)*P[2*4+j];
        for (int j=0;j<4;j++) Pn[2*4+j]+=(-dt)*P[3*4+j];
        std::array<float,16> P2=Pn;
        for (int i=0;i<4;i++) P2[i*4+0]+=Pn[i*4+2]*(-ds*s_t);
        for (int i=0;i<4;i++) P2[i*4+1]+=Pn[i*4+2]*(ds*c);
        for (int i=0;i<4;i++) P2[i*4+2]+=Pn[i*4+3]*(-dt);
        P2[0]+=Q_xy*std::abs(ds); P2[5]+=Q_xy*std::abs(ds);
        P2[10]+=Q_theta*std::abs(dtheta); P2[15]+=Q_bias;
        P=P2; sanitize();
    }
    void updateWallDist(int axis,float wallCoord,float measuredDist,float sign,float R) noexcept {
        float robotPos=(axis==0)?x():y();
        float expected=sign*(wallCoord-robotPos);
        float innov=measuredDist-expected;
        std::array<float,4> H{}; H[axis]=sign;
        float S=R;
        for (int i=0;i<4;i++) for (int j=0;j<4;j++) S+=H[i]*pij(i,j)*H[j];
        if (S<1e-12f) return;
        if (innov*innov>9.0f*S) return;
        std::array<float,4> K{};
        for (int i=0;i<4;i++) { float PH=0; for(int k=0;k<4;k++) PH+=pij(i,k)*H[k]; K[i]=PH/S; }
        for (int i=0;i<4;i++) err[i]+=K[i]*innov;
        std::array<float,16> A{};
        for (int i=0;i<4;i++) for (int j=0;j<4;j++) A[i*4+j]=(i==j?1.f:0.f)-K[i]*H[j];
        std::array<float,16> AP{};
        for (int i=0;i<4;i++) for (int j=0;j<4;j++) for(int k=0;k<4;k++) AP[i*4+j]+=A[i*4+k]*pij(k,j);
        std::array<float,16> Pn{};
        for (int i=0;i<4;i++) for (int j=0;j<4;j++) {
            float v=0; for(int k=0;k<4;k++) v+=AP[i*4+k]*A[j*4+k]; Pn[i*4+j]=v+K[i]*R*K[j];
        }
        P=Pn; nom_x+=err[0]; nom_y+=err[1]; nom_theta=wrapAngle(nom_theta+err[2]);
        err[0]=err[1]=err[2]=0; sanitize();
    }
    void updateHeading(float meas,float R) noexcept {
        float innov=angleDiff(meas,theta());
        std::array<float,4> H{}; H[2]=1.0f;
        float S=pij(2,2)+R; if(S<1e-12f) return;
        std::array<float,4> K{}; for(int i=0;i<4;i++) K[i]=pij(i,2)/S;
        for(int i=0;i<4;i++) err[i]+=K[i]*innov;
        std::array<float,16> A{};
        for(int i=0;i<4;i++) for(int j=0;j<4;j++) A[i*4+j]=(i==j?1.f:0.f)-K[i]*H[j];
        std::array<float,16> AP{};
        for(int i=0;i<4;i++) for(int j=0;j<4;j++) for(int k=0;k<4;k++) AP[i*4+j]+=A[i*4+k]*pij(k,j);
        std::array<float,16> Pn{};
        for(int i=0;i<4;i++) for(int j=0;j<4;j++) { float v=0; for(int k=0;k<4;k++) v+=AP[i*4+k]*A[j*4+k]; Pn[i*4+j]=v+K[i]*R*K[j]; }
        P=Pn; nom_theta=wrapAngle(nom_theta+err[2]); err[2]=0; sanitize();
    }
    void snapHeadingCardinal() noexcept {
        float snapped=std::round(theta()/HALF_PI)*HALF_PI;
        if (std::abs(angleDiff(theta(),snapped))<0.12f) updateHeading(snapped,1e-3f);
    }
    void print() const {
        std::cout<<std::fixed<<std::setprecision(5)
                 <<"  ESKF: x="<<x()<<" y="<<y()<<" θ="<<theta()
                 <<" bias="<<bias()<<"\n"
                 <<"  Cov diag: ["<<pij(0,0)<<", "<<pij(1,1)<<", "<<pij(2,2)<<", "<<pij(3,3)<<"]\n";
        if (faultEncoder||faultGyro)
            std::cout<<"  WARN sensor faults: encoder="<<faultEncoder<<" gyro="<<faultGyro<<"\n";
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  RobotParams  (LAB-10: cellSize and explore velocity rescaled for 25 cm)
// ───────────────────────────────────────────────────────────────────────────
struct RobotParams {
    float maxTotalAccel   = 12.0f;
    float maxBraking      = 10.0f;
    float maxAccelFwd     =  9.0f;
    float maxJerk         = 60.0f;
    float maxVelocity     =  4.0f;
    float exploreVelocity =  0.5f;  // LAB-10: ~2 cells/s at 25 cm

    float wheelbase         = 0.09f;  // LAB-10: slightly wider chassis for 25 cm cell
    float trackWidth        = 0.07f;
    float cellSize          = 0.25f;  // LAB-10
    float steeringBandwidth = 18.0f;

    float Kp_crosstrack = 4.0f;
    float Kd_crosstrack = 0.3f;
    float Kp_heading    = 2.0f;
    float Kd_heading    = 0.1f;
    float Kp_center = 3.0f;
    float Ki_center = 0.1f;
};

// ───────────────────────────────────────────────────────────────────────────
//  TrajPoint + ClothoidSeg (unchanged)
// ───────────────────────────────────────────────────────────────────────────
struct TrajPoint {
    float x=0,y=0,heading=0,curvature=0,velocity=0,arcLen=0,accel=0,jerk=0,ff_steer_rate=0;
};
struct ClothoidSeg {
    float x0,y0,theta0,kappa0,kappaEnd,length;
    static constexpr float GL_XI[8]={0.0950125098f,0.2816035508f,0.4580167777f,0.6178762444f,0.7554044084f,0.8656312024f,0.9445750231f,0.9894009350f};
    static constexpr float GL_W[8] ={0.1894506105f,0.1826034150f,0.1691565194f,0.1495959889f,0.1246289463f,0.0951585117f,0.0622535239f,0.0271524594f};
    struct State{float x,y,theta,kappa;};
    [[nodiscard]] State eval(float s) const noexcept {
        if(s<=0) return {x0,y0,theta0,kappa0};
        float dkds=(length>1e-9f)?(kappaEnd-kappa0)/length:0;
        auto tAt=[&](float t){return theta0+kappa0*t+0.5f*dkds*t*t;};
        float mid=s*0.5f,hr=s*0.5f,px=x0,py=y0;
        for(int i=0;i<8;i++){float t1=mid+hr*GL_XI[i],t2=mid-hr*GL_XI[i];px+=GL_W[i]*hr*(std::cos(tAt(t1))+std::cos(tAt(t2)));py+=GL_W[i]*hr*(std::sin(tAt(t1))+std::sin(tAt(t2)));}
        return {px,py,tAt(s),kappa0+dkds*s};
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Path → waypoints + Racing-line optimiser (unchanged)
// ───────────────────────────────────────────────────────────────────────────
struct Waypoint{float x,y,heading;};
[[nodiscard]] static std::vector<Waypoint> pathToWaypoints(
    const std::vector<CellCoord>& path, const MazeConfig& cfg)
{
    std::vector<Waypoint> wps; int N=int(path.size());
    for (int i=0;i<N;i++) {
        Vec2 pos=cfg.cellCentre(path[i]);
        float hdg=(wps.empty())?0:wps.back().heading;
        if (i+1<N){Vec2 nxt=cfg.cellCentre(path[i+1]);hdg=std::atan2(nxt.y-pos.y,nxt.x-pos.x);}
        wps.push_back({pos.x,pos.y,hdg});
    }
    if (wps.size()>=2) wps.back().heading=wps[wps.size()-2].heading;
    return wps;
}
[[nodiscard]] static std::vector<Waypoint> optimiseRacingLine(
    std::vector<Waypoint> wps, float halfWidth, float margin=0.03f)
{
    int N=int(wps.size()); if(N<3) return wps;
    float hw=halfWidth-margin; if(hw<=0) return wps;
    std::vector<Waypoint> centres=wps; float step=hw*0.20f;
    for (int iter=0;iter<50;iter++) {
        float totalChange=0;
        for (int i=1;i<N-1;i++) {
            float dix=wps[i+1].x-2*wps[i].x+wps[i-1].x, diy=wps[i+1].y-2*wps[i].y+wps[i-1].y;
            float dpx=0,dpy=0,dnx=0,dny=0;
            if(i>1){dpx=wps[i].x-2*wps[i-1].x+wps[i-2].x;dpy=wps[i].y-2*wps[i-1].y+wps[i-2].y;}
            if(i<N-2){dnx=wps[i+2].x-2*wps[i+1].x+wps[i].x;dny=wps[i+2].y-2*wps[i+1].y+wps[i].y;}
            float gx=2*(dpx+dnx-2*dix), gy=2*(dpy+dny-2*diy);
            float nx=std::max(centres[i].x-hw,std::min(centres[i].x+hw,wps[i].x-step*gx));
            float ny=std::max(centres[i].y-hw,std::min(centres[i].y+hw,wps[i].y-step*gy));
            totalChange+=std::abs(nx-wps[i].x)+std::abs(ny-wps[i].y);
            wps[i].x=nx; wps[i].y=ny;
        }
        for(int i=0;i<N-1;i++){float dx=wps[i+1].x-wps[i].x,dy=wps[i+1].y-wps[i].y;wps[i].heading=std::atan2(dy,dx);}
        if(N>=2) wps[N-1].heading=wps[N-2].heading;
        if(totalChange<1e-7f) break;
    }
    return wps;
}

// ───────────────────────────────────────────────────────────────────────────
//  TrajGen — clothoid-arc-clothoid  (FIX-M retained; cellSize from robot params)
// ───────────────────────────────────────────────────────────────────────────
class TrajGen {
public:
    static constexpr int SC=10, CC=24, AC=20;
    [[nodiscard]] static std::vector<TrajPoint> generate(
        const std::vector<Waypoint>& wps, const RobotParams& robot)
    {
        std::vector<TrajPoint> traj; int N=int(wps.size());
        if(N<2) return traj;
        float cumArc=0,kPrev=0;
        auto emit=[&](float x,float y,float hdg,float k){
            float arc=0;
            if(!traj.empty()){float dx=x-traj.back().x,dy=y-traj.back().y;arc=std::sqrt(dx*dx+dy*dy);}
            cumArc+=arc;
            traj.push_back({x,y,hdg,k,robot.maxVelocity,cumArc,0,0,0});
        };
        for (int wi=0;wi+1<N;wi++) {
            const Waypoint& wa=wps[wi],&wb=wps[wi+1];
            float dhdg=angleDiff(wb.heading,wa.heading);
            float segLen=std::hypot(wb.x-wa.x,wb.y-wa.y);
            if(segLen<1e-7f) continue;
            int si=(wi==0)?0:1;
            if(std::abs(dhdg)<5e-3f) {
                for(int i=si;i<=SC;i++){float t=float(i)/SC;emit(wa.x+t*(wb.x-wa.x),wa.y+t*(wb.y-wa.y),wa.heading,0);}
                kPrev=0;
            } else {
                float R=segLen/(2*std::sin(std::abs(dhdg)*0.5f));
                R=std::max(R,robot.cellSize*0.05f);
                float kTurn=(1/R)*(dhdg>0?1:-1);
                float dkappa=std::abs(kTurn-kPrev);
                float L_c=std::max(dkappa*robot.maxVelocity/robot.steeringBandwidth,0.005f);
                L_c=std::min(L_c,segLen*0.45f);
                ClothoidSeg entry{wa.x,wa.y,wa.heading,kPrev,kTurn,L_c};
                for(int i=si;i<=CC;i++){float s=float(i)/CC*L_c;auto st=entry.eval(s);emit(st.x,st.y,st.theta,st.kappa);}
                auto eEnd=entry.eval(L_c);
                float sign=(dhdg>0)?1:-1;
                float perpAngle=eEnd.theta+sign*HALF_PI;
                float cx=eEnd.x+R*std::cos(perpAngle),cy=eEnd.y+R*std::sin(perpAngle);
                float clothoidTurn=L_c/(2*R);
                float arcAngle=std::abs(dhdg)-2*clothoidTurn;  // FIX-M
                if(arcAngle>1e-4f){
                    float startArc=std::atan2(eEnd.y-cy,eEnd.x-cx);
                    for(int i=1;i<=AC;i++){float t=float(i)/AC,ang=startArc+sign*t*arcAngle;emit(cx+R*std::cos(ang),cy+R*std::sin(ang),wrapAngle(ang+sign*HALF_PI),kTurn);}
                }
                const TrajPoint& ae=traj.back();
                ClothoidSeg exitSeg{ae.x,ae.y,ae.heading,kTurn,0,L_c};
                for(int i=1;i<=CC;i++){float s=float(i)/CC*L_c;auto st=exitSeg.eval(s);emit(st.x,st.y,st.theta,st.kappa);}
                kPrev=0;
            }
        }
        return traj;
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  VelocityProfile (unchanged from v4.3, ROB-2 sanitizer retained)
// ───────────────────────────────────────────────────────────────────────────
class VelocityProfile {
public:
    [[nodiscard]] static float kammLong(float k,float v,float aT) noexcept {
        float aL2=(k*v*v)*(k*v*v),aT2=aT*aT; return (aL2>=aT2)?0:std::sqrt(aT2-aL2);
    }
    [[nodiscard]] static float vMaxCurv(float k,float aT) noexcept {
        return (std::abs(k)<1e-7f)?INF_F:std::sqrt(aT/std::abs(k));
    }
    static void curvatureCeilings(std::vector<TrajPoint>& t,float vMax,float aT) {
        for(auto& tp:t) tp.velocity=std::min(vMax,vMaxCurv(tp.curvature,aT));
    }
    static void globalBrakingPass(std::vector<TrajPoint>& t,float aT,float aB) {
        int N=int(t.size()); if(N<2) return;
        for(int p=0;p<3;p++) for(int i=N-2;i>=0;i--) {
            float ds=t[i+1].arcLen-t[i].arcLen; if(ds<1e-9f) continue;
            float vM=std::sqrt(t[i+1].velocity*t[i+1].velocity+2*std::min(kammLong(t[i].curvature,t[i].velocity,aT),aB)*ds);
            if(vM<t[i].velocity) t[i].velocity=vM;
        }
    }
    static void backwardPass(std::vector<TrajPoint>& t,float mJ,float aT,float aB,int maxI=25) {
        int N=int(t.size()); if(N<2) return;
        t.back().velocity=0;
        for(int iter=0;iter<maxI;iter++){
            float maxC=0,prevB=0;
            for(int i=N-2;i>=0;i--){
                float ds=t[i+1].arcLen-t[i].arcLen; if(ds<1e-9f) continue;
                float v1=t[i+1].velocity;
                float aB2=std::min({kammLong(t[i+1].curvature,v1,aT),aB,prevB+mJ*ds});
                prevB=aB2;
                float vM=std::sqrt(v1*v1+2*aB2*ds),vN=std::min(t[i].velocity,vM);
                maxC=std::max(maxC,std::abs(vN-t[i].velocity)); t[i].velocity=vN;
            }
            if(maxC<1e-5f) break;
        }
    }
    static void forwardPass(std::vector<TrajPoint>& t,float mJ,float aT,float aA) {
        if(t.empty()) return;
        t.front().velocity=0; float prevA=0;
        for(int i=1;i<int(t.size());i++){
            float ds=t[i].arcLen-t[i-1].arcLen; if(ds<1e-9f){t[i].velocity=std::min(t[i].velocity,t[i-1].velocity);continue;}
            float v0=t[i-1].velocity;
            float aA2=std::min({kammLong(t[i-1].curvature,v0,aT),aA,prevA+mJ*ds});
            float vN=std::min(t[i].velocity,std::sqrt(v0*v0+2*aA2*ds));
            t[i].velocity=vN;
            prevA=(v0+vN>1e-6f)?(vN*vN-v0*v0)/(2*ds):0;
            prevA=std::max(-aT,std::min(prevA,aT));
            t[i].accel=prevA;
        }
    }
    static void computeJerk(std::vector<TrajPoint>& t) {
        for(int i=1;i<int(t.size());i++){
            float ds=t[i].arcLen-t[i-1].arcLen;
            float vA=0.5f*(t[i].velocity+t[i-1].velocity);
            float dt=(vA>1e-4f)?ds/vA:1e-3f;
            float dkds=(ds>1e-9f)?(t[i].curvature-t[i-1].curvature)/ds:0;
            t[i].ff_steer_rate=dkds*vA;
            float a_now=(t[i].velocity-t[i-1].velocity)/std::max(dt,1e-6f);
            float a_prv=(i>1)?t[i-1].accel:0;
            t[i].jerk=(a_now-a_prv)/std::max(dt,1e-6f);
        }
    }
    [[nodiscard]] static bool sanitizeTrajectory(std::vector<TrajPoint>& t) {  // ROB-2
        bool clean=true; float lastArc=0;
        for(auto& tp:t){
            if(!std::isfinite(tp.x)||!std::isfinite(tp.y)||!std::isfinite(tp.heading)) clean=false;
            if(!std::isfinite(tp.x)) tp.x=0; if(!std::isfinite(tp.y)) tp.y=0; if(!std::isfinite(tp.heading)) tp.heading=0;
            if(!std::isfinite(tp.curvature)){tp.curvature=0;clean=false;}
            if(!std::isfinite(tp.velocity)||tp.velocity<0){tp.velocity=0;clean=false;}
            if(!std::isfinite(tp.accel)) tp.accel=0; if(!std::isfinite(tp.jerk)) tp.jerk=0;
            if(!std::isfinite(tp.arcLen)||tp.arcLen<lastArc){tp.arcLen=lastArc;clean=false;}
            lastArc=tp.arcLen;
        }
        return clean;
    }
    [[nodiscard]] static float estimateTime(const std::vector<TrajPoint>& t) {
        float tm=0;
        for(int i=1;i<int(t.size());i++){
            float ds=t[i].arcLen-t[i-1].arcLen,vA=0.5f*(t[i].velocity+t[i-1].velocity);
            if(vA>1e-6f) tm+=ds/vA;
        }
        return tm;
    }
    [[nodiscard]] static float peakLatAccel(const std::vector<TrajPoint>& t) {
        float pk=0; for(auto& tp:t) pk=std::max(pk,std::abs(tp.curvature)*tp.velocity*tp.velocity); return pk;
    }
    [[nodiscard]] static float peakVelocity(const std::vector<TrajPoint>& t) {
        float pk=0; for(auto& tp:t) pk=std::max(pk,tp.velocity); return pk;
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  TVLQR (unchanged from v4.3, FIX-J/P/R retained)
// ───────────────────────────────────────────────────────────────────────────
struct TVLQRGain { float K[2][3]; float arcLen; };
class TVLQRSolver {
public:
    static constexpr float Qx=200,Qy=200,Qt=50,Rv=1,Rw=0.5f;
    [[nodiscard]] static std::vector<TVLQRGain> solve(
        const std::vector<TrajPoint>& traj, float)
    {
        int N=int(traj.size()); std::vector<TVLQRGain> gains(N);
        float P[3][3]={{Qx,0,0},{0,Qy,0},{0,0,Qt}};
        float Q[3][3]={{Qx,0,0},{0,Qy,0},{0,0,Qt}};
        float Rinv[2][2]={{1/Rv,0},{0,1/Rw}};
        for(int i=N-1;i>=0;i--){
            const auto& tp=traj[i];
            float v=std::max(tp.velocity,0.01f),h=tp.heading;
            float dt=(i>0)?(traj[i].arcLen-traj[i-1].arcLen)/v:0.01f;
            dt=std::max(std::min(dt,0.2f),1e-5f);
            float A[3][3]={{0,0,-v*std::sin(h)},{0,0,v*std::cos(h)},{0,0,0}};
            float B[3][2]={{std::cos(h),0},{std::sin(h),0},{0,1}};
            float BRB[3][3]{};
            for(int r=0;r<3;r++) for(int c=0;c<3;c++) for(int m=0;m<2;m++) for(int n=0;n<2;n++) BRB[r][c]+=B[r][m]*Rinv[m][n]*B[c][n];
            float ATP[3][3]{},PA[3][3]{},PB[3][3]{},PBRBP[3][3]{};
            for(int r=0;r<3;r++) for(int c=0;c<3;c++) for(int m=0;m<3;m++){ATP[r][c]+=A[m][r]*P[m][c];PA[r][c]+=P[r][m]*A[m][c];PB[r][c]+=P[r][m]*BRB[m][c];}
            for(int r=0;r<3;r++) for(int c=0;c<3;c++) for(int m=0;m<3;m++) PBRBP[r][c]+=PB[r][m]*P[m][c];
            for(int r=0;r<3;r++) for(int c=0;c<3;c++) P[r][c]-=dt*(-Q[r][c]-ATP[r][c]-PA[r][c]+PBRBP[r][c]);
            for(int r=0;r<3;r++) for(int c=r+1;c<3;c++) P[r][c]=P[c][r]=0.5f*(P[r][c]+P[c][r]);
            for(int r=0;r<3;r++){P[r][r]=std::max(P[r][r],1e-4f);if(!std::isfinite(P[r][r]))P[r][r]=1e-4f;}
            float BTP[2][3]{};
            for(int r=0;r<2;r++) for(int c=0;c<3;c++) for(int m=0;m<3;m++) BTP[r][c]+=B[m][r]*P[m][c];
            for(int r=0;r<2;r++) for(int c=0;c<3;c++){gains[i].K[r][c]=0;for(int m=0;m<2;m++) gains[i].K[r][c]+=Rinv[r][m]*BTP[m][c];if(!std::isfinite(gains[i].K[r][c]))gains[i].K[r][c]=0;}
            gains[i].arcLen=tp.arcLen;
        }
        return gains;
    }
    static void computeControl(const std::vector<TVLQRGain>& gains,const TrajPoint& ref,
                                float ex,float ey,float et,float& dv,float& dw)
    {
        if(gains.empty()){dv=0;dw=0;return;}  // FIX-P
        int lo=0,hi=int(gains.size())-1,idx=0;
        while(lo<=hi){int mid=(lo+hi)/2;if(gains[mid].arcLen<ref.arcLen)lo=mid+1;else{idx=mid;hi=mid-1;}}
        const auto& K=gains[idx].K;
        float dx=ex-ref.x,dy=ey-ref.y,dt=angleDiff(et,ref.heading);
        dv=-(K[0][0]*dx+K[0][1]*dy+K[0][2]*dt);
        dw=-(K[1][0]*dx+K[1][1]*dy+K[1][2]*dt);
        if(!std::isfinite(dv)) dv=0; if(!std::isfinite(dw)) dw=0;
        dv=clampf(dv,-DELTA_V_LIMIT,DELTA_V_LIMIT);
        dw=clampf(dw,-OMEGA_LIMIT,OMEGA_LIMIT);
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  PD controller + WallCenteringPID (unchanged)
// ───────────────────────────────────────────────────────────────────────────
class PDController {
public:
    explicit PDController(const RobotParams& p):params(p){}
    [[nodiscard]] std::pair<float,float> compute(
        float ex,float ey,float et,const TrajPoint& ref,float prevCt,float dt) const noexcept
    {
        float ch=std::cos(ref.heading),sh=std::sin(ref.heading);
        float dx=ex-ref.x,dy=ey-ref.y;
        float e_xt=-dx*sh+dy*ch;
        float e_hdg=angleDiff(et,ref.heading);
        float de_xt=(dt>1e-6f)?(e_xt-prevCt)/dt:0;
        float ff=ref.velocity*ref.curvature;
        float omega=ff+params.Kp_crosstrack*e_xt+params.Kd_crosstrack*de_xt+params.Kp_heading*e_hdg;
        if(!std::isfinite(omega)) omega=0;
        omega=clampf(omega,-OMEGA_LIMIT,OMEGA_LIMIT);
        return {ref.velocity,omega};
    }
private: const RobotParams& params;
};
class WallCenteringPID {
public:
    explicit WallCenteringPID(const RobotParams& p):params(p){}
    void reset() noexcept {integral=0;}
    [[nodiscard]] float compute(float lD,bool lV,float rD,bool rV,float cs,float dt) noexcept {
        if(!lV&&!rV) return 0;
        float e=0;
        if(lV&&rV) e=(lD-rD)*0.5f;
        else if(lV) e=lD-cs*0.5f;
        else e=cs*0.5f-rD;
        integral=std::max(-0.05f,std::min(0.05f,integral+e*dt));
        return params.Kp_center*e+params.Ki_center*integral;
    }
private: const RobotParams& params; float integral=0;
};

// ───────────────────────────────────────────────────────────────────────────
//  AdaptiveScaler (unchanged)
// ───────────────────────────────────────────────────────────────────────────
class AdaptiveScaler {
public:
    struct Sample{float planned,achieved,curvature;};
    std::vector<Sample> samples;
    void record(float planned,float achieved,float curvature){samples.push_back({planned,achieved,curvature});}
    [[nodiscard]] std::pair<float,float> factors() const {
        if(samples.empty()) return {1,1};
        float sumSt=0;int nSt=0;float sumCn=0;int nCn=0;
        for(const auto& s:samples){
            if(s.planned<1e-3f) continue;
            float r=std::max(0.5f,std::min(1.2f,s.achieved/s.planned));
            if(std::abs(s.curvature)<2.0f){sumSt+=r;nSt++;}else{sumCn+=r;nCn++;}
        }
        auto scale=[](float sum,int n){if(n==0)return 1.f;float r=sum/n;return std::min(1.08f,1+0.8f*(r-1));};
        return {scale(sumSt,nSt),scale(sumCn,nCn)};
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  Explorer  (LAB-2: goal discovery via white-floor sensor; LAB-3: initial
//  heading from open wall; FIX-O, HW-3, ROB-4 from v4.3 retained)
// ───────────────────────────────────────────────────────────────────────────
class Explorer {
public:
    // LAB-3: detect initial heading from the 3-wall start cell's open passage
    [[nodiscard]] static float startHeading(const Maze& botMaze,
                                             const CellCoord& startCell) noexcept
    {
        for (int w=0;w<4;w++)
            if (botMaze.at(startCell).wallKnown[w] && !botMaze.at(startCell).wall[w])
                return WALL_HEADING[w];
        return 0.0f; // fallback
    }

    static void deadReckonStep(ESKF& kf,const CellCoord& from,const CellCoord& to,
                               const MazeConfig& cfg,float vel)
    {
        float ds=(from.r==to.r||from.c==to.c)?cfg.cellSize:cfg.cellSize*SQRT2;
        float dt=(vel>1e-4f)?ds/vel:0.01f;
        kf.predict(ds,0,dt);
    }

    // LAB-2: sense walls AND check white floor (goal cell detection).
    // Returns true if new map info was acquired.
    static bool senseCell(Maze& botMaze, const Maze& truthMaze,
                           MazeConfig& botCfg,
                           ESKF& kf, const CellCoord& cc, const MazeConfig& cfg)
    {
        bool newInfo=false;
        for (int w=0;w<4;w++) {
            if (!botMaze.at(cc).wallKnown[w]) {
                botMaze.setWall(cc,w,truthMaze.at(cc).wall[w]); newInfo=true;
            }
        }
        botMaze.at(cc).explored=true;
        botMaze.at(cc).visitCount++;

        // LAB-2: white floor detection — set goalCell on first entry
        if (truthMaze.at(cc).isGoalCell && !botCfg.isGoalKnown()) {
            botCfg.goalCell = cc;
            std::cout << "  [GOAL CELL FOUND at (" << cc.r << "," << cc.c
                      << ") — white floor detected]\n";
        }

        Vec2 ctr=cfg.cellCentre(cc);
        float half=cfg.cellSize*0.5f,R=SensorModel::measurementVariance();
        if (botMaze.at(cc).wallKnown[WE]&&!botMaze.at(cc).wall[WE]) kf.updateWallDist(0,ctr.x+half,half,+1,R);
        if (botMaze.at(cc).wallKnown[WW]&&!botMaze.at(cc).wall[WW]) kf.updateWallDist(0,ctr.x-half,half,-1,R);
        if (botMaze.at(cc).wallKnown[WN]&&!botMaze.at(cc).wall[WN]) kf.updateWallDist(1,ctr.y+half,half,+1,R);
        if (botMaze.at(cc).wallKnown[WS]&&!botMaze.at(cc).wall[WS]) kf.updateWallDist(1,ctr.y-half,half,-1,R);
        int open=0; for(int w=0;w<4;w++) if(botMaze.at(cc).wallKnown[w]&&!botMaze.at(cc).wall[w]) open++;
        if(open==2) kf.snapHeadingCardinal();
        return newInfo;
    }

    static void applyWallCentering(Maze& botMaze,ESKF& kf,const MazeConfig& cfg,
                                    WallCenteringPID& wc,const CellCoord& cur,float dt)
    {
        auto getWalls=[](float theta)->std::pair<int,int>{
            float la=wrapAngle(theta+HALF_PI); int lW=0; float mn=INF_F;
            for(int w=0;w<4;w++){float d=std::abs(angleDiff(la,WALL_HEADING[w]));if(d<mn){mn=d;lW=w;}}
            return {lW,WALL_OPP[lW]};
        };
        auto [lW,rW]=getWalls(kf.theta());
        float half=cfg.cellSize*0.5f;
        bool lV=botMaze.at(cur).wallKnown[lW]&&botMaze.at(cur).wall[lW];
        bool rV=botMaze.at(cur).wallKnown[rW]&&botMaze.at(cur).wall[rW];
        float lD=lV?half:half*2, rD=rV?half:half*2;
        float corr=wc.compute(lD,lV,rD,rV,cfg.cellSize,dt);
        if(std::abs(corr)>1e-6f) kf.updateHeading(wrapAngle(kf.theta()+corr*dt),5e-4f);
    }

    // LAB-2/LAB-4: explore until goal found or frontier exhausted.
    // Returns visited cells. stalls out-param counts stall events (→ penalties).
    [[nodiscard]] static std::vector<CellCoord> explore(
        Maze& botMaze, const Maze& truthMaze,
        MazeConfig& botCfg,
        ESKF& kf, const MazeConfig& cfg,
        WallCenteringPID& wallCtrl, float exploreVel,
        int& stallCount)
    {
        stallCount=0;
        CellCoord current=cfg.startCell;
        std::vector<CellCoord> visited{current};
        senseCell(botMaze,truthMaze,botCfg,kf,current,cfg);
        botMaze.at(current).explored=true;

        const int MAX_STEPS=8*LNRGrid::total;  // ROB-4
        int steps=0;
        const float dtStep=(exploreVel>1e-4f)?(cfg.cellSize/exploreVel):0.01f;

        while (steps<MAX_STEPS) {
            // LAB-2: stop when we find the goal
            if (botCfg.isGoalKnown() && botCfg.isGoal(current)) break;

            FloodFill::solveToGoal(botMaze,true);

            std::vector<CellCoord> raw;
            if (botCfg.isGoalKnown()) {
                raw = ThetaStar::findPath(botMaze,current,true);
            } else {
                // Goal unknown: navigate toward nearest unexplored frontier
                // by targeting the cell with the lowest visit count
                CellCoord frontier{-1,-1};
                int bestVisits=INT_MAX;
                for(int i=0;i<LNRGrid::total;i++){
                    if(!botMaze.cells[i].explored){
                        CellCoord fc{i/LNRGrid::cols,i%LNRGrid::cols};
                        if(!botMaze.cfg->valid(fc)) continue;
                        float fd=ThetaStar::octile(current,fc);
                        int v=botMaze.cells[i].visitCount;
                        // prefer unvisited cells closer to current
                        int score=v*100+int(fd*10);
                        if(score<bestVisits){bestVisits=score;frontier=fc;}
                    }
                }
                if (frontier.r>=0)
                    raw=ThetaStar::findPathTo(botMaze,current,frontier,true);
            }

            if (raw.size()<2) {
                // LAB-4: no path → stall event → penalty will be counted by caller
                stallCount++;
                break;
            }
            auto path=ThetaStar::expandPath(raw);
            wallCtrl.reset();
            bool advanced=false;
            for(size_t i=1;i<path.size();i++){
                CellCoord next=path[i];
                int d8=dirFromDelta(current,next);
                if(d8<0||!botMaze.canMove8(current,d8,false)) break;  // FIX-O
                deadReckonStep(kf,current,next,cfg,exploreVel);  // HW-3
                senseCell(botMaze,truthMaze,botCfg,kf,next,cfg);
                botMaze.at(next).explored=true;
                current=next; visited.push_back(next); advanced=true;
                applyWallCentering(botMaze,kf,cfg,wallCtrl,current,dtStep);
                if(botCfg.isGoalKnown()&&botCfg.isGoal(current)) break;
                steps++;
                if(steps>=MAX_STEPS) break;
            }
            if(!advanced){ stallCount++; steps++; }
        }
        return visited;
    }

    // Verified greedy drive (from returnToStart / speed approach, FIX-O)
    static bool driveVerifiedTo(Maze& botMaze,const Maze& truthMaze,
                                 MazeConfig& botCfg,
                                 ESKF& kf,const MazeConfig& cfg,
                                 CellCoord& current,const CellCoord& target,
                                 float vel,int maxSteps,std::vector<CellCoord>& driven)
    {
        int steps=0,stalls=0;
        while(steps<maxSteps){
            if(current==target) return true;
            auto raw=ThetaStar::findPathTo(botMaze,current,target,true);
            if(raw.size()<2) return current==target;
            auto path=ThetaStar::expandPath(raw);
            bool advanced=false;
            for(size_t i=1;i<path.size();i++){
                CellCoord next=path[i];
                int d8=dirFromDelta(current,next);
                if(d8<0||!botMaze.canMove8(current,d8,false)) break;
                deadReckonStep(kf,current,next,cfg,vel);
                senseCell(botMaze,truthMaze,botCfg,kf,next,cfg);
                current=next; driven.push_back(next); advanced=true;
                if(current==target) return true;
                if(++steps>=maxSteps) break;
            }
            if(!advanced){ if(++stalls>4) return false; }
            else stalls=0;
        }
        return current==target;
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  ══════════════════  LNR COMPETITION MANAGEMENT  ══════════════════════════
//
//  These classes implement the competition rules from §5.1–§8 of the LNR
//  Laberinto rulebook.
// ───────────────────────────────────────────────────────────────────────────

// ── LAB-4/5: Penalty and trial-stop tracker ───────────────────────────────
class LNRPenaltyTracker {
public:
    int   penalties    = 0;
    bool  roundLost    = false;    // §8: ≥3 penalties, piece off, etc.
    bool  trialStopped = false;    // §6: 15 s immobility / wall-push
    float stallTimeAcc = 0.0f;    // accumulated immobility time
    bool  extraTimeUsed= false;   // §6: 5-min extra time used once

    void reset() {
        penalties=0; roundLost=false; trialStopped=false;
        stallTimeAcc=0; extraTimeUsed=false;
    }

    // Call when the robot is stuck for 'stallSecs' simulated seconds.
    // Returns true if a penalty was issued.
    bool notifyStall(float stallSecs) {
        stallTimeAcc += stallSecs;
        if (stallTimeAcc >= LNR_IMMOBILITY_STOP_S) {
            trialStopped = true;  // §6
            std::cout << "  [TRIAL STOPPED — " << LNR_IMMOBILITY_STOP_S << " s immobility]\n";
            return false;
        }
        if (stallSecs >= LNR_STALL_PENALTY_S) {
            applyPenalty("10 s stall in cell");
            stallTimeAcc = 0;
            return true;
        }
        return false;
    }

    void applyPenalty(const std::string& reason) {
        penalties++;
        std::cout << "  [PENALTY #" << penalties << " — " << reason << "]\n";
        if (penalties >= LNR_MAX_PENALTIES) {
            roundLost = true;
            std::cout << "  [ROUND LOST — " << LNR_MAX_PENALTIES << " penalties]\n";
        }
    }

    void notifyPieceDetachment() {
        roundLost = true;  // §8
        std::cout << "  [ROUND LOST — piece detachment]\n";
    }
};

// ── LAB-6: Official timer ─────────────────────────────────────────────────
class LNROfficialTimer {
public:
    float elapsed    = 0.0f;  // total elapsed time in seconds
    float officialTime= INF_F; // time recorded when robot entered goal (§5.3.9)
    bool  running    = false;

    void start()   { running=true; elapsed=0; officialTime=INF_F; }
    void stop()    { running=false; }
    void advance(float dt) { if(running) elapsed+=dt; }

    // §5.3.9: time stops when robot completely inside goal cell
    void recordGoal() {
        if(std::isinf(officialTime)) {
            officialTime=elapsed;
            std::cout<<"  [GOAL REACHED — official time: "
                     <<std::fixed<<std::setprecision(3)<<officialTime<<" s]\n";
        }
    }
    [[nodiscard]] bool goalRecorded() const { return std::isfinite(officialTime); }
};

// ── §5.2.1: Per-round result ──────────────────────────────────────────────
struct RoundResult {
    bool  solved     = false;
    bool  lost       = false;    // round forfeited (§8)
    float time       = INF_F;   // official time if solved
    int   penalties  = 0;
    float remainDist = INF_F;   // cells remaining if not solved
    float elapsed    = 0.0f;    // total elapsed time (regardless of solved)

    void print(const std::string& prefix="") const {
        std::cout<<prefix;
        if (lost) { std::cout<<"LOST (round forfeited)\n"; return; }
        if (solved) std::cout<<"SOLVED in "<<std::fixed<<std::setprecision(3)<<time<<" s";
        else        std::cout<<"NOT SOLVED  remain="<<std::fixed<<std::setprecision(1)<<remainDist<<" cells";
        std::cout<<"  penalties="<<penalties<<"  elapsed="<<elapsed<<" s\n";
    }
};

// §5.2.1 personal classification: a is better than b?
[[nodiscard]] static bool isBetterRound(const RoundResult& a, const RoundResult& b) {
    // Lost round is always worst
    if (a.lost != b.lost) return !a.lost;
    if (a.lost) return false;
    // Solved always beats not-solved
    if (a.solved != b.solved) return a.solved;
    if (a.solved) {
        // Both solved: prefer 0-penalty, then lower time, then fewer penalties
        if ((a.penalties==0) != (b.penalties==0)) return a.penalties==0;
        if (std::abs(a.time-b.time)>1e-3f) return a.time < b.time;
        return a.penalties < b.penalties;
    } else {
        // Both not solved: prefer 0-penalty, then lower remaining dist, then fewer penalties
        if ((a.penalties==0) != (b.penalties==0)) return a.penalties==0;
        if (std::abs(a.remainDist-b.remainDist)>1e-3f) return a.remainDist < b.remainDist;
        return a.penalties < b.penalties;
    }
}
[[nodiscard]] static RoundResult bestRound(const RoundResult& r0, const RoundResult& r1) {
    return isBetterRound(r0,r1) ? r0 : r1;
}

// ── §5.1.5 / §5.2.2: Per-configuration and overall result ────────────────
struct ConfigResult {
    RoundResult round[2];
    RoundResult best;           // best of round[0], round[1]
    int configIdx = 0;

    void print() const {
        std::cout<<"\n  Config "<<(configIdx+1)<<":\n";
        round[0].print("    Round 1: ");
        round[1].print("    Round 2: ");
        best.print("    Best:    ");
    }
};

struct OverallResult {
    ConfigResult cfg[2];
    int  solvedConfigs = 0;
    float avgTime      = INF_F;
    float avgRemain    = INF_F;
    int   totalPenalties = 0;
    int   avgPenalties = 0;

    void compute() {
        solvedConfigs=0; totalPenalties=0;
        float sumTime=0,sumRemain=0; int nTime=0,nRemain=0;
        for(int i=0;i<2;i++){
            const auto& b=cfg[i].best;
            if(!b.lost){
                if(b.solved){solvedConfigs++;sumTime+=b.time;nTime++;}
                else{sumRemain+=b.remainDist;nRemain++;}
                totalPenalties+=b.penalties;
            }
        }
        avgTime   =(nTime>0)   ? sumTime/nTime   : INF_F;
        avgRemain =(nRemain>0) ? sumRemain/nRemain: INF_F;
        avgPenalties=totalPenalties/2;
    }

    // §5.2.2: higher is better, returns true if this is better than other
    [[nodiscard]] bool isBetter(const OverallResult& o) const {
        // More solved configs wins
        if (solvedConfigs != o.solvedConfigs) return solvedConfigs > o.solvedConfigs;
        if (solvedConfigs > 0) {
            // Lower avg time; prefer zero-penalty
            bool zeroA=(totalPenalties==0), zeroB=(o.totalPenalties==0);
            if (zeroA!=zeroB) return zeroA;
            if (std::abs(avgTime-o.avgTime)>1e-3f) return avgTime<o.avgTime;
            return avgPenalties < o.avgPenalties;
        } else {
            bool zeroA=(totalPenalties==0),zeroB=(o.totalPenalties==0);
            if(zeroA!=zeroB) return zeroA;
            if(std::abs(avgRemain-o.avgRemain)>1e-3f) return avgRemain<o.avgRemain;
            return avgPenalties < o.avgPenalties;
        }
    }

    void print() const {
        std::cout<<"\n╔══════════════════════════════════════════╗\n"
                 <<"║         OVERALL RESULT (§5.2.2)          ║\n"
                 <<"╚══════════════════════════════════════════╝\n";
        cfg[0].print(); cfg[1].print();
        std::cout<<"\n  Summary:\n"
                 <<"  Solved configurations : "<<solvedConfigs<<"/2\n"
                 <<"  Total penalties       : "<<totalPenalties<<"\n";
        if(std::isfinite(avgTime))
            std::cout<<"  Avg solving time      : "<<std::fixed<<std::setprecision(3)<<avgTime<<" s\n";
        if(std::isfinite(avgRemain))
            std::cout<<"  Avg remaining dist    : "<<std::fixed<<std::setprecision(1)<<avgRemain<<" cells\n";
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  RunStats + profilePath (adapted: cellSize from cfg, LAB-2 single goal)
// ───────────────────────────────────────────────────────────────────────────
struct RunStats {
    std::string label;
    int   pathCells=0,trajPoints=0;
    float pathLength=0,estimatedTime=0,peakLatAccel=0,peakVelocity=0;
};

[[nodiscard]] static RunStats profilePath(
    const std::vector<CellCoord>& cellPath,
    const MazeConfig& cfg, const RobotParams& robot,
    const std::string& label, float vMax,
    bool printDetails, bool computeTVLQR=false,
    std::vector<TrajPoint>* outTraj=nullptr)
{
    RunStats stats; stats.label=label; stats.pathCells=int(cellPath.size());
    if (cellPath.size()<2) {
        if(!label.empty()) std::cerr<<label<<": path too short\n";
        return stats;
    }
    auto expanded=ThetaStar::expandPath(cellPath);
    auto wps=pathToWaypoints(expanded,cfg);
    wps=optimiseRacingLine(wps,cfg.cellSize*0.5f);
    auto traj=TrajGen::generate(wps,robot);
    if(traj.empty()){if(!label.empty()) std::cerr<<label<<": trajectory failed\n";return stats;}

    VelocityProfile::curvatureCeilings(traj,vMax,robot.maxTotalAccel);
    VelocityProfile::globalBrakingPass(traj,robot.maxTotalAccel,robot.maxBraking);
    VelocityProfile::backwardPass(traj,robot.maxJerk,robot.maxTotalAccel,robot.maxBraking);
    VelocityProfile::forwardPass(traj,robot.maxJerk,robot.maxTotalAccel,robot.maxAccelFwd);
    VelocityProfile::computeJerk(traj);
    if(!VelocityProfile::sanitizeTrajectory(traj)&&!label.empty())
        std::cerr<<label<<": trajectory sanitised\n";
    if(outTraj) *outTraj=traj;

    std::vector<TVLQRGain> gains;
    if(computeTVLQR) gains=TVLQRSolver::solve(traj,robot.wheelbase);

    stats.trajPoints   =int(traj.size());
    stats.pathLength   =traj.back().arcLen;
    stats.estimatedTime=VelocityProfile::estimateTime(traj);
    stats.peakLatAccel =VelocityProfile::peakLatAccel(traj);
    stats.peakVelocity =VelocityProfile::peakVelocity(traj);

    if(printDetails){
        std::cout<<std::fixed<<std::setprecision(4);
        std::cout<<"\n── "<<label<<" ──\n"
                 <<"  Cells="<<stats.pathCells<<"  TrajPts="<<stats.trajPoints
                 <<"  Length="<<stats.pathLength<<" m"
                 <<"  Time≈"<<stats.estimatedTime<<" s"
                 <<"  PeakV="<<stats.peakVelocity<<" m/s"
                 <<"  PeakLatG="<<stats.peakLatAccel/9.81f<<" g\n";
        if(computeTVLQR&&!gains.empty())
            std::cout<<"  TVLQR gains at arc=0: v=["<<gains[0].K[0][0]<<","<<gains[0].K[0][1]<<","<<gains[0].K[0][2]<<"]\n";
    }
    return stats;
}

// ───────────────────────────────────────────────────────────────────────────
//  Truth maze builders  (LAB-1/2/3: variable size, single goal, 3-wall start)
//
//  Two representative LNR configurations (8×10 cells, 25 cm each).
//  Config 1: start=(7,0) open East, goal=(2,8)
//  Config 2: start=(0,9) open South, goal=(5,1)
// ───────────────────────────────────────────────────────────────────────────
static MazeConfig buildConfig1() {
    MazeConfig cfg;
    cfg.rows=8; cfg.cols=10; cfg.cellSize=0.25f;
    cfg.startCell={7,0}; cfg.goalCell={-1,-1};  // goal discovered during run
    cfg.maxTimeSec=180.0f;
    return cfg;
}
static MazeConfig buildConfig2() {
    MazeConfig cfg;
    cfg.rows=8; cfg.cols=10; cfg.cellSize=0.25f;
    cfg.startCell={0,9}; cfg.goalCell={-1,-1};
    cfg.maxTimeSec=180.0f;
    return cfg;
}

// LAB-3: validate that start cell has exactly 3 walls (1 open passage)
[[nodiscard]] static bool validateStartCell(const Maze& maze, const CellCoord& sc) {
    int walls=0;
    for(int w=0;w<4;w++) if(maze.at(sc).wallKnown[w]&&maze.at(sc).wall[w]) walls++;
    if(walls!=3){
        std::cerr<<"WARNING: start cell ("<<sc.r<<","<<sc.c<<") has "<<walls
                 <<" walls, expected 3 (§4)\n";
        return false;
    }
    return true;
}

static void placeTruthWalls(Maze& truth, const MazeConfig& cfg, int configIdx) {
    // Start with fully-open interior (borders already set by Maze::init)
    for(int r=0;r<LNRGrid::rows;r++)
        for(int c=0;c<LNRGrid::cols;c++)
            for(int w=0;w<4;w++){
                truth.at(r,c).wall[w]=false;
                truth.at(r,c).wallKnown[w]=true;
            }
    // Re-apply borders
    for(int c=0;c<LNRGrid::cols;c++){
        truth.setWall({0,c},WN,true); truth.setWall({LNRGrid::rows-1,c},WS,true);
    }
    for(int r=0;r<LNRGrid::rows;r++){
        truth.setWall({r,0},WW,true); truth.setWall({r,LNRGrid::cols-1},WE,true);
    }

    if (configIdx==0) {
        // Config 1: start (7,0) 3-wall: N,S,W closed, E open
        truth.setWall({7,0},WN,true); truth.setWall({7,0},WS,true); // WW already border
        // Interior walls creating a maze with solution to (2,8)
        for(auto& [r,c,w]:std::vector<std::tuple<int,int,int>>{
            {6,0,WN},{5,1,WW},{5,1,WN},{4,2,WS},{4,2,WE},
            {3,3,WN},{3,3,WE},{2,4,WS},{1,5,WE},{1,6,WN},
            {0,6,WE},{1,7,WS},{2,7,WE},{3,6,WN},{3,5,WE},
            {4,4,WN},{5,3,WE},{6,4,WS},{6,5,WN},{5,6,WW},
            {4,7,WS},{3,8,WW},{2,9,WW},{1,8,WN},{0,7,WE}
        }) truth.setWall({r,c},w,true);
        // Mark goal cell (white floor)
        truth.at(2,8).isGoalCell=true;
    } else {
        // Config 2: start (0,9) 3-wall: N,S border + WE border; open S
        truth.setWall({0,9},WW,true); // WN and WE already borders; open WS
        // Interior walls for config 2
        for(auto& [r,c,w]:std::vector<std::tuple<int,int,int>>{
            {1,8,WE},{2,7,WN},{2,7,WE},{3,6,WS},{4,5,WN},
            {4,5,WW},{5,4,WS},{6,3,WE},{6,2,WN},{5,1,WE},
            {4,0,WN},{3,1,WS},{2,2,WE},{1,3,WN},{0,4,WE},
            {1,4,WS},{2,5,WW},{3,4,WN},{4,3,WE},{5,2,WS},
            {6,1,WN},{7,0,WE},{7,2,WS},{6,3,WN},{5,4,WE}
        }) truth.setWall({r,c},w,true);
        truth.at(5,1).isGoalCell=true;
    }
}

// ───────────────────────────────────────────────────────────────────────────
//  LNRLaberintoPlanner — orchestrates one competition configuration
//  Implements the LAB-7 round/configuration simulation.
// ───────────────────────────────────────────────────────────────────────────
class LNRLaberintoPlanner {
public:
    MazeConfig    botCfg;    // robot's knowledge (goalCell unknown at round start)
    MazeConfig    truthCfg;  // ground truth for simulation
    Maze          botMaze;
    Maze          truthMaze;
    RobotParams   robot;
    ESKF          kf;
    AdaptiveScaler scaler;
    std::mt19937  rng{42};

    static constexpr float RETURN_DETOUR_FACTOR = 1.30f;
    static constexpr float RETURN_DETOUR_SLACK  = 4.0f;

    // LAB-8: reset robot map (new configuration — walls unknown again)
    void initConfiguration(const MazeConfig& truth, int configIdx) {
        truthCfg = truth;
        truthCfg.sync();

        // Bot starts with no goal knowledge and no internal wall knowledge
        botCfg         = truth;
        botCfg.goalCell= {-1,-1};

        botMaze.init(botCfg);
        truthMaze.init(truthCfg);
        placeTruthWalls(truthMaze, truthCfg, configIdx);

        // LAB-3: derive start heading from the single open wall
        Vec2 startPos = truthCfg.cellCentre(truthCfg.startCell);
        // Sense the start cell walls first so we know the heading
        for(int w=0;w<4;w++) botMaze.setWall(truthCfg.startCell,w,truthMaze.at(truthCfg.startCell).wall[w]);
        float heading0 = Explorer::startHeading(botMaze, truthCfg.startCell);
        kf.reset(startPos.x, startPos.y, heading0);

        if (!validateStartCell(truthMaze, truthCfg.startCell))
            std::cerr << "  [Config " << configIdx+1 << ": start cell §4 violation — proceeding]\n";

        scaler = AdaptiveScaler{};
    }

    // LAB-7 Round 1: exploration — map is empty except borders + start walls.
    // Returns the round result. Exploration time counts as official time.
    RoundResult runRound1(int configIdx) {
        std::cout<<"\n  ── Round 1 (Exploration) ──\n";

        // Reset bot map keeping only start-cell walls (LAB-9: no modification)
        botMaze.init(botCfg);
        for(int w=0;w<4;w++) botMaze.setWall(truthCfg.startCell,w,truthMaze.at(truthCfg.startCell).wall[w]);
        botCfg.goalCell={-1,-1};

        Vec2 sp=truthCfg.cellCentre(truthCfg.startCell);
        float h0=Explorer::startHeading(botMaze,truthCfg.startCell);
        kf.reset(sp.x,sp.y,h0);

        LNRPenaltyTracker pt;
        LNROfficialTimer  timer;
        timer.start();

        WallCenteringPID wallCtrl(robot);
        int stallCount=0;
        auto visited = Explorer::explore(botMaze,truthMaze,botCfg,kf,truthCfg,
                                         wallCtrl,robot.exploreVelocity,stallCount);

        // Simulate time from exploration trajectory
        auto exploredStats = profilePath(visited,truthCfg,robot,"R1 Scout",robot.exploreVelocity,true);
        timer.advance(exploredStats.estimatedTime);

        // Account for stall events → penalties (LAB-4)
        for(int s=0;s<stallCount;s++){
            pt.notifyStall(LNR_STALL_PENALTY_S);
            if(pt.roundLost||pt.trialStopped) break;
            timer.advance(LNR_RESTART_WINDOW_S);  // 1-min restart window
        }

        RoundResult result;
        result.penalties = pt.penalties;
        result.elapsed   = timer.elapsed;
        result.lost      = pt.roundLost || pt.trialStopped;

        if (!result.lost) {
            bool goalReached = botCfg.isGoalKnown() &&
                               std::any_of(visited.begin(),visited.end(),
                                   [&](const CellCoord& c){ return botCfg.isGoal(c); });
            if (goalReached) {
                timer.recordGoal();
                result.solved = true;
                result.time   = timer.officialTime;
            } else {
                // Calculate remaining distance to goal (if goal is known)
                result.solved = false;
                if (botCfg.isGoalKnown()) {
                    FloodFill::solveToGoal(botMaze,false);
                    float dist=botMaze.at(visited.back()).floodDist;
                    result.remainDist=std::isfinite(dist)?dist:INF_F;
                } else {
                    result.remainDist=INF_F;  // goal never found
                }
            }
        }

        result.print("  Round 1 result: ");

        // Prepare shared state for Round 2 (map + goal retained)
        std::cout<<"  Map consistency: "
                 <<(botMaze.checkConsistency()?"OK":"VIOLATION")<<"\n";
        std::cout<<"  Cells explored: "<<visited.size()
                 <<"  Frontiers remaining: "<<botMaze.frontierCount()<<"\n";
        return result;
    }

    // LAB-7 Round 2: speed run using map from Round 1.
    // Robot is NOT re-initialized (§5.3.3 — map retained between rounds of same config).
    RoundResult runRound2(int /*configIdx*/) {
        std::cout<<"\n  ── Round 2 (Speed Run — map retained from Round 1) ──\n";

        if (!botCfg.isGoalKnown()) {
            std::cout<<"  Goal cell unknown — repeating exploration\n";
            // Fallback: re-run exploration to find goal
            int sc2=0;
            WallCenteringPID wc2(robot);
            auto vis2=Explorer::explore(botMaze,truthMaze,botCfg,kf,truthCfg,
                                         wc2,robot.exploreVelocity,sc2);
            if (!botCfg.isGoalKnown()) {
                RoundResult r; r.lost=false; r.solved=false; r.remainDist=INF_F;
                r.print("  Round 2 result: ");
                return r;
            }
        }

        // Reset position to start; keep map (LAB-7)
        Vec2 sp=truthCfg.cellCentre(truthCfg.startCell);
        float h0=Explorer::startHeading(botMaze,truthCfg.startCell);
        kf.reset(sp.x,sp.y,h0);

        LNRPenaltyTracker pt;
        LNROfficialTimer  timer;
        timer.start();

        // ROB-1 style: find best path to the single known goal cell
        FloodFill::solveToGoal(botMaze,false);
        auto path = ThetaStar::findPath(botMaze,truthCfg.startCell,false);

        RoundResult result;
        if (path.size()<2) {
            std::cerr<<"  Round 2: no path to goal on conservative map\n";
            result.solved=false;
            result.remainDist=botMaze.at(truthCfg.startCell).floodDist;
            result.penalties=0; result.elapsed=0;
            result.print("  Round 2 result: ");
            return result;
        }

        auto [af,cf]=scaler.factors();
        RobotParams scaledRobot=robot;
        scaledRobot.maxTotalAccel*=af;
        scaledRobot.maxVelocity=std::min(robot.maxVelocity*cf,robot.maxVelocity);
        std::cout<<"  Adaptive factors: accel="<<af<<" corner="<<cf<<"\n";

        std::vector<TrajPoint> speedTraj;
        auto stats=profilePath(path,truthCfg,scaledRobot,"R2 Speed Run",
                               scaledRobot.maxVelocity,true,true,&speedTraj);

        timer.advance(stats.estimatedTime);
        timer.recordGoal();

        // Simulate verification drive along path (FIX-O pattern)
        std::vector<CellCoord> driven{truthCfg.startCell};
        CellCoord current=truthCfg.startCell;
        Explorer::driveVerifiedTo(botMaze,truthMaze,botCfg,kf,truthCfg,
                                   current,botCfg.goalCell,
                                   scaledRobot.maxVelocity,4*LNRGrid::total,driven);

        bool reached=current==botCfg.goalCell;
        result.penalties=pt.penalties;
        result.elapsed=timer.elapsed;
        result.lost=pt.roundLost;
        if(!result.lost && reached){
            result.solved=true;
            result.time=timer.officialTime;
        } else {
            result.solved=false;
            FloodFill::solveToGoal(botMaze,false);
            float d=botMaze.at(current).floodDist;
            result.remainDist=std::isfinite(d)?d:INF_F;
        }
        result.print("  Round 2 result: ");

        // TVLQR + PD tracking report
        if(!speedTraj.empty()){
            auto gains=TVLQRSolver::solve(speedTraj,robot.wheelbase);
            PDController pdCtrl(scaledRobot);
            Vec2 sw=truthCfg.cellCentre(truthCfg.startCell);
            float pdX=sw.x,pdY=sw.y,pdT=h0,prevCt=0;
            std::cout<<"  Tracking snippet (every 8th step):\n";
            std::cout<<"    arc(m)   v    ω(r/s)  e_ct(m)\n";
            int stride=std::max(1,int(speedTraj.size())/8);
            for(int i=0;i+1<int(speedTraj.size());i++){
                const TrajPoint& ref=speedTraj[i];
                float ds=speedTraj[i+1].arcLen-ref.arcLen;
                float dt=(ref.velocity>0.01f)?ds/ref.velocity:0.01f;
                auto [vC,wC]=pdCtrl.compute(pdX,pdY,pdT,ref,prevCt,dt);
                float sh=std::sin(ref.heading),ch=std::cos(ref.heading);
                float eCt=-(pdX-ref.x)*sh+(pdY-ref.y)*ch; prevCt=eCt;
                if(i%stride==0)
                    std::cout<<std::fixed<<std::setprecision(3)
                             <<"    "<<std::setw(6)<<ref.arcLen
                             <<"  "<<std::setw(5)<<ref.velocity
                             <<"  "<<std::setw(7)<<wC
                             <<"  "<<std::setw(7)<<eCt<<"\n";
                pdX+=vC*std::cos(pdT)*dt; pdY+=vC*std::sin(pdT)*dt;
                pdT=wrapAngle(pdT+wC*dt);
            }
        }
        return result;
    }

    // LAB-7: run both rounds for one configuration; return ConfigResult
    ConfigResult runConfiguration(int configIdx) {
        ConfigResult cr; cr.configIdx=configIdx;
        std::cout<<"\n╔══════════════════════════════════════════╗\n"
                 <<"║  Configuration "<<(configIdx+1)<<"                         ║\n"
                 <<"║  Maze: "<<LNRGrid::rows<<"×"<<LNRGrid::cols
                 <<"  cellSize="<<truthCfg.cellSize*100<<" cm"<<"  ║\n"
                 <<"╚══════════════════════════════════════════╝\n";
        if(botCfg.isGoalKnown())
            std::cout<<"  Start: ("<<truthCfg.startCell.r<<","<<truthCfg.startCell.c<<")"
                     <<"  Goal: ("<<botCfg.goalCell.r<<","<<botCfg.goalCell.c<<")\n";
        else
            std::cout<<"  Start: ("<<truthCfg.startCell.r<<","<<truthCfg.startCell.c<<")"
                     <<"  Goal: [unknown — white floor]\n";

        cr.round[0]=runRound1(configIdx);
        cr.round[1]=runRound2(configIdx);
        cr.best=bestRound(cr.round[0],cr.round[1]);
        std::cout<<"\n  Best round for config "<<(configIdx+1)<<": ";
        cr.best.print();
        return cr;
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  LNRCompetitionManager — top-level orchestrator (§5.1–§5.3, LAB-7/8/9)
// ───────────────────────────────────────────────────────────────────────────
class LNRCompetitionManager {
public:
    LNRLaberintoPlanner planner;
    OverallResult       result;

    void run() {
        std::cout<<"╔══════════════════════════════════════════════════╗\n"
                 <<"║   LNR Laberinto Edition v1.0                     ║\n"
                 <<"║   Rules: Reglamento Laberinto — LNR              ║\n"
                 <<"║   Adapted from GDW Micromouse Championship v4.3  ║\n"
                 <<"╚══════════════════════════════════════════════════╝\n\n";

        // ── Free practice (§5.3.2): different maze, not shown here ──────────
        std::cout<<"[§5.3.2] Free practice maze available (simulation omitted)\n";
        std::cout<<"[§5.3.3] Robot submitted to jury\n";
        std::cout<<"[§5.3.3] No modifications allowed until inter-config break\n\n";

        // ── Configuration 1 (§5.1.1) ─────────────────────────────────────
        MazeConfig cfg1=buildConfig1();
        cfg1.sync();
        planner.initConfiguration(cfg1, 0);
        result.cfg[0]=planner.runConfiguration(0);

        // ── LAB-8: 30-minute inter-configuration break (§5.3.6) ──────────
        std::cout<<"\n[§5.3.6] 30-minute adjustment break.\n"
                 <<"         Teams may modify robots and charge batteries.\n"
                 <<"         Internal map wiped for new configuration.\n";

        // ── Configuration 2 (§5.1.3) ─────────────────────────────────────
        MazeConfig cfg2=buildConfig2();
        cfg2.sync();
        planner.initConfiguration(cfg2, 1);  // LAB-8: full reset
        result.cfg[1]=planner.runConfiguration(1);

        // ── Final result (§5.1.5) ─────────────────────────────────────────
        result.compute();
        result.print();

        // ── Classification summary ────────────────────────────────────────
        std::cout<<"\n╔══════════════════════════════════════════════════╗\n"
                 <<"║          CLASSIFICATION  (§5.2.2)                ║\n"
                 <<"╚══════════════════════════════════════════════════╝\n";
        std::cout<<"  Ranking criteria (in priority order):\n"
                 <<"  1. Solved both configurations: "<<(result.solvedConfigs==2?"YES":"NO")<<"\n"
                 <<"  2. Avg solving time (no penalties): ";
        if(result.totalPenalties==0 && std::isfinite(result.avgTime))
            std::cout<<std::fixed<<std::setprecision(3)<<result.avgTime<<" s\n";
        else std::cout<<"N/A\n";
        std::cout<<"  3. Avg solving time + avg penalties: ";
        if(std::isfinite(result.avgTime))
            std::cout<<std::fixed<<std::setprecision(3)<<result.avgTime<<" s, "<<result.avgPenalties<<"\n";
        else std::cout<<"N/A\n";
        std::cout<<"  4. Avg remaining distance (no penalties): ";
        if(result.totalPenalties==0 && std::isfinite(result.avgRemain))
            std::cout<<std::fixed<<std::setprecision(1)<<result.avgRemain<<" cells\n";
        else std::cout<<"N/A\n";
        std::cout<<"  5. Avg remaining distance + avg penalties: ";
        if(std::isfinite(result.avgRemain))
            std::cout<<std::fixed<<std::setprecision(1)<<result.avgRemain<<" cells, "<<result.avgPenalties<<"\n";
        else std::cout<<"N/A\n";

        std::cout<<"\n  Architecture summary (LNR Laberinto Edition):\n"
                 <<"  ┌─ Planning ──────────────────────────────────────────────┐\n"
                 <<"  │ Exploration : FloodFill frontier + Theta* (verified)    │\n"
                 <<"  │ Goal detect : White floor sensor on first cell entry     │\n"
                 <<"  │ Speed run   : Theta* findPath (conservative map)        │\n"
                 <<"  │ Cell size   : 25 cm (LAB-10); maze min 4×5 (LAB-1)     │\n"
                 <<"  ├─ Competition rules ─────────────────────────────────────┤\n"
                 <<"  │ Configs     : 2, each 2 rounds; best round per §5.2.1  │\n"
                 <<"  │ Penalty     : 10 s stall → restart; ≥3 → round lost    │\n"
                 <<"  │ Timer       : stops when robot fully in goal (§5.3.9)  │\n"
                 <<"  │ Break       : 30 min between configs (§5.3.6)          │\n"
                 <<"  │ Ranking     : §5.2.2 group criteria                    │\n"
                 <<"  ├─ Localisation ──────────────────────────────────────────┤\n"
                 <<"  │ Filter      : ESKF 4-state (v4.3 ROB-3/HW-1 retained)  │\n"
                 <<"  ├─ Trajectory ─────────────────────────────────────────── ┤\n"
                 <<"  │ Geometry    : Clothoid–Arc–Clothoid (FIX-M retained)   │\n"
                 <<"  │ Velocity    : Kamm circle + S-curve + sanitizer        │\n"
                 <<"  ├─ Control ───────────────────────────────────────────────┤\n"
                 <<"  │ Primary     : TVLQR (FIX-P/R/J retained)               │\n"
                 <<"  │ Fallback    : PD + curvature feedforward               │\n"
                 <<"  │ Lateral     : Wall-centering PID (scout only)          │\n"
                 <<"  └────────────────────────────────────────────────────────┘\n";
    }
};

// ───────────────────────────────────────────────────────────────────────────
//  main
// ───────────────────────────────────────────────────────────────────────────
int main() {
    LNRCompetitionManager mgr;
    mgr.run();
    return 0;
}
