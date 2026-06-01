#pragma once
// ═══════════════════════════════════════════════════════════════════════════
//  src/core/MazeEngine.h
//  GDW Design Lab — Maze representation, planners, and procedural generator
// ═══════════════════════════════════════════════════════════════════════════
#include "Types.h"
#include <deque>
#include <functional>
#include <map>
#include <optional>
#include <queue>
#include <random>
#include <set>
#include <vector>

// ── Cell ──────────────────────────────────────────────────────────────────
struct Cell {
    std::array<bool,4> wallKnown  = {false,false,false,false};
    std::array<bool,4> wall       = {true,true,true,true};
    bool  explored    = false;
    int   visitCount  = 0;
    float floodDist   = INF_F;
    // D* Lite state
    float dstar_g     = INF_F;
    float dstar_rhs   = INF_F;
    // Rendering hints
    int   colorHeat   = 0;   // 0-255 for heatmap rendering

    [[nodiscard]] bool passableOpt (int w) const noexcept {
        return !(wallKnown[w] && wall[w]);
    }
    [[nodiscard]] bool passableCons(int w) const noexcept {
        return wallKnown[w] && !wall[w];
    }
    [[nodiscard]] bool hasFrontier () const noexcept {
        for (int w=0;w<4;w++) if (!wallKnown[w]) return true;
        return false;
    }
};

// ── Maze ──────────────────────────────────────────────────────────────────
class Maze {
public:
    std::array<Cell, N_CELLS> cells{};
    const MazeConfig* cfg = nullptr;

    void init(const MazeConfig& c);
    void reset();                          // clear known walls, keep truth walls

    [[nodiscard]] Cell&       at(const CellCoord& cc)       noexcept;
    [[nodiscard]] const Cell& at(const CellCoord& cc) const noexcept;
    [[nodiscard]] Cell&       at(int r, int c)              noexcept;
    [[nodiscard]] const Cell& at(int r, int c)        const noexcept;

    void setWall(const CellCoord& cc, int w, bool present);
    void setWallKnown(const CellCoord& cc, int w, bool known);

    [[nodiscard]] bool canMove8     (const CellCoord& cc, int d8, bool optimistic) const noexcept;
    [[nodiscard]] bool canMoveCardinal(const CellCoord& cc, int w, bool optimistic) const noexcept;
    [[nodiscard]] int  frontierCount()  const noexcept;
    [[nodiscard]] int  exploredCount()  const noexcept;
    [[nodiscard]] bool fullyExplored()  const noexcept;

    // Compute actual wall distance in world frame from cell centre
    [[nodiscard]] float trueWallDistance(const CellCoord& cc, int w) const noexcept;

private:
    void placeBorderWalls();
};

// ── FloodFill ─────────────────────────────────────────────────────────────
class FloodFill {
public:
    static void solve   (Maze& maze, const std::vector<CellCoord>& seeds, bool optimistic);
    static void solveToGoal (Maze& maze, bool optimistic);
    static void solveToStart(Maze& maze, bool optimistic);
};

// ── DStarLite ─────────────────────────────────────────────────────────────
class DStarLite {
public:
    struct Key {
        float k1, k2;
        bool operator< (const Key& o) const noexcept {
            return (std::abs(k1-o.k1)>1e-6f) ? k1<o.k1 : k2<o.k2;
        }
        bool operator> (const Key& o) const noexcept { return o<*this; }
        bool operator<=(const Key& o) const noexcept { return !(o<*this); }
    };

    Maze*     maze       = nullptr;
    CellCoord start      = {-1,-1};
    bool      optimistic = true;
    float     km         = 0.f;

    using QEntry = std::pair<Key, CellCoord>;
    struct QCmp { bool operator()(const QEntry& a, const QEntry& b) const noexcept { return a.first>b.first; } };
    std::priority_queue<QEntry, std::vector<QEntry>, QCmp> U;

    void init(Maze& m, CellCoord s, bool opt);
    void computeShortestPath();
    void notifyWallChanged(const CellCoord& cc);

    [[nodiscard]] float heuristic(const CellCoord& cc) const noexcept;
    [[nodiscard]] Key   calcKey  (const CellCoord& cc) const noexcept;
};

// ── ThetaStar ─────────────────────────────────────────────────────────────
class ThetaStar {
public:
    [[nodiscard]] static bool lineOfSight(const Maze& maze,
                                           const CellCoord& a,
                                           const CellCoord& b,
                                           bool optimistic) noexcept;

    [[nodiscard]] static float dist(const CellCoord& a, const CellCoord& b) noexcept;

    [[nodiscard]] static std::vector<CellCoord> findPath(
        const Maze& maze, const CellCoord& start, bool optimistic);

    [[nodiscard]] static std::vector<CellCoord> expandPath(
        const std::vector<CellCoord>& path);

private:
    [[nodiscard]] static bool checkWall(const Maze& m, const CellCoord& cc,
                                         int w, bool optimistic) noexcept;
};

// ── MazeGenerator ─────────────────────────────────────────────────────────
// Generates random valid APEC-standard micromouse mazes (16×16).
class MazeGenerator {
public:
    // Generate a maze using randomized Prim's MST algorithm.
    // All cells reachable from start, at least one path to goal.
    static Maze generate(std::mt19937& rng, const MazeConfig& cfg);

    // Official GDW v4.1 test maze (hardcoded walls)
    static Maze officialGDW(const MazeConfig& cfg);

    // Copy maze structure (walls only, no exploration state)
    static void copyStructure(const Maze& src, Maze& dst);

private:
    static void carve(Maze& maze, const MazeConfig& cfg, std::mt19937& rng);
    static void ensureGoalAccess(Maze& maze, const MazeConfig& cfg, std::mt19937& rng);
};
