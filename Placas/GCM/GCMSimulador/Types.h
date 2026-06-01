#pragma once
// ═══════════════════════════════════════════════════════════════════════════
//  src/core/Types.h
//  GDW Design Lab — Shared primitive types and constants
//  All subsystems include this header.
// ═══════════════════════════════════════════════════════════════════════════
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

// ── Numeric constants ─────────────────────────────────────────────────────
inline constexpr int   MAZE_N  = 16;
inline constexpr int   N_CELLS = MAZE_N * MAZE_N;
inline constexpr float INF_F   = std::numeric_limits<float>::infinity();
inline constexpr float PI      = 3.14159265358979f;
inline constexpr float TWO_PI  = 2.0f * PI;
inline constexpr float HALF_PI = PI * 0.5f;
inline constexpr float SQRT2   = 1.41421356237f;
inline constexpr float G_ACCEL = 9.81f;          // m/s²
inline constexpr float AIR_RHO = 1.225f;         // kg/m³ air density

// ── Angle utilities ───────────────────────────────────────────────────────
[[nodiscard]] inline float wrapAngle(float a) noexcept {
    a = std::fmod(a + PI, TWO_PI);
    if (a < 0.0f) a += TWO_PI;
    return a - PI;
}
[[nodiscard]] inline float angleDiff(float a, float b) noexcept {
    return wrapAngle(a - b);
}
template<typename T>
[[nodiscard]] inline T clamp(T v, T lo, T hi) noexcept {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

// ── Wall / direction encoding ─────────────────────────────────────────────
// World frame: x = East, y = North
// Cell (r,c): row 0 at North wall, col 0 at West wall
// Positive curvature = left (CCW) turn
enum Wall : int { WN = 0, WE = 1, WS = 2, WW = 3 };

inline constexpr int   WALL_OPP[4]     = { WS, WW, WN, WE };
inline constexpr int   WALL_DC[4]      = {  0,  1,  0, -1 }; // column delta
inline constexpr int   WALL_DR[4]      = { -1,  0,  1,  0 }; // row delta
inline constexpr float WALL_HEADING[4] = { HALF_PI, 0.f, -HALF_PI, PI };

// 8-direction movement
inline constexpr int   D8C[8]    = {  0, 1, 1, 1,  0,-1,-1,-1 };
inline constexpr int   D8R[8]    = { -1,-1, 0, 1,  1, 1, 0,-1 };
inline constexpr float D8COST[8] = {  1.f,SQRT2,1.f,SQRT2,1.f,SQRT2,1.f,SQRT2 };
inline constexpr int   D8WALLS[8][2] = {
    {WN,-1},{WN,WE},{WE,-1},{WE,WS},{WS,-1},{WS,WW},{WW,-1},{WN,WW}
};

// ── CellCoord ─────────────────────────────────────────────────────────────
struct CellCoord {
    int r = 0, c = 0;
    [[nodiscard]] bool operator==(const CellCoord& o) const noexcept { return r==o.r&&c==o.c; }
    [[nodiscard]] bool operator!=(const CellCoord& o) const noexcept { return !(*this==o); }
    [[nodiscard]] bool operator< (const CellCoord& o) const noexcept {
        return (r!=o.r) ? r<o.r : c<o.c;
    }
    [[nodiscard]] int  idx()   const noexcept { return r*MAZE_N+c; }
    [[nodiscard]] bool valid() const noexcept { return r>=0&&r<MAZE_N&&c>=0&&c<MAZE_N; }
    [[nodiscard]] CellCoord neighbour(int w) const noexcept { return {r+WALL_DR[w],c+WALL_DC[w]}; }
    [[nodiscard]] CellCoord step8(int d)     const noexcept { return {r+D8R[d],c+D8C[d]}; }
};

// ── Vec2 — world-space 2-D vector (x=East, y=North) ──────────────────────
struct Vec2 {
    float x=0.f, y=0.f;
    [[nodiscard]] Vec2  operator+(const Vec2& o) const noexcept { return {x+o.x,y+o.y}; }
    [[nodiscard]] Vec2  operator-(const Vec2& o) const noexcept { return {x-o.x,y-o.y}; }
    [[nodiscard]] Vec2  operator*(float s)       const noexcept { return {x*s,y*s}; }
    [[nodiscard]] float dot  (const Vec2& o)     const noexcept { return x*o.x+y*o.y; }
    [[nodiscard]] float cross(const Vec2& o)     const noexcept { return x*o.y-y*o.x; }
    [[nodiscard]] float norm()                   const noexcept { return std::sqrt(x*x+y*y); }
    [[nodiscard]] Vec2  normalised() const noexcept {
        float n=norm(); return n>1e-9f ? Vec2{x/n,y/n} : Vec2{};
    }
};

// ── MazeConfig ────────────────────────────────────────────────────────────
struct MazeConfig {
    int   size     = MAZE_N;
    float cellSize = 0.18f;   // metres per cell
    std::array<CellCoord,4> goalCells = {{{7,7},{7,8},{8,7},{8,8}}};
    CellCoord startCell = {15, 0};

    [[nodiscard]] bool isGoal(const CellCoord& cc) const noexcept {
        for (auto& g : goalCells) if (g==cc) return true;
        return false;
    }
    [[nodiscard]] bool valid(int r, int c) const noexcept {
        return r>=0&&r<size&&c>=0&&c<size;
    }
    [[nodiscard]] bool valid(const CellCoord& cc) const noexcept {
        return valid(cc.r,cc.c);
    }
    // Cell centre in world frame
    [[nodiscard]] Vec2 cellCentre(const CellCoord& cc) const noexcept {
        return {(cc.c+0.5f)*cellSize, -(cc.r+0.5f)*cellSize};
    }
};

// ── TrajPoint — one point on a velocity-profiled trajectory ──────────────
struct TrajPoint {
    float x=0.f, y=0.f;
    float heading=0.f;    // rad (0=East, π/2=North, CCW positive)
    float curvature=0.f;  // 1/m, positive = left (CCW)
    float velocity=0.f;   // m/s
    float arcLen=0.f;     // m cumulative
    float accel=0.f;      // m/s² longitudinal
    float jerk=0.f;       // m/s³
    float ff_steer_rate=0.f; // dκ/dt * v (feedforward)
};

// ── RobotParams — algorithm-side performance parameters ───────────────────
// Derived from RobotDesign.computeDerived(); used directly by GDW v4.1 stack.
struct RobotParams {
    float maxTotalAccel    = 12.0f;  // m/s² Kamm-circle radius
    float maxBraking       = 10.0f;  // m/s²
    float maxAccelFwd      =  9.0f;  // m/s²
    float maxJerk          = 60.0f;  // m/s³
    float maxVelocity      =  5.0f;  // m/s
    float exploreVelocity  =  0.6f;  // m/s scout run
    float wheelbase        = 0.07f;  // m
    float trackWidth       = 0.06f;  // m
    float cellSize         = 0.18f;  // m
    float steeringBandwidth= 20.0f;  // rad/s
    // PD controller gains
    float Kp_crosstrack    =  4.0f;
    float Kd_crosstrack    =  0.3f;
    float Kp_heading       =  2.0f;
    float Kd_heading       =  0.1f;
    // Wall-centering PID gains (scout run)
    float Kp_center        =  3.0f;
    float Ki_center        =  0.1f;
};

// ── RunStats — complete statistics from one simulation run ────────────────
struct RunStats {
    std::string label;

    // Path / trajectory
    int   pathCells     = 0;
    int   trajPoints    = 0;
    float pathLength    = 0.f;   // m

    // Timing
    float scoutTime     = 0.f;   // s
    float returnTime    = 0.f;   // s
    float speedRunTime  = 0.f;   // s
    float estimatedTime = 0.f;   // s  (from velocity profile)
    float totalTime     = 0.f;   // s

    // Peak values
    float peakLatAccel  = 0.f;   // m/s²
    float peakLongAccel = 0.f;   // m/s²
    float peakJerk      = 0.f;   // m/s³
    float peakVelocity  = 0.f;   // m/s

    // Physics simulation stats
    float avgWheelSlip     = 0.f;   // dimensionless
    float maxWheelSlip     = 0.f;
    float energyConsumed   = 0.f;   // J
    float localizationRMSE = 0.f;   // m
    float trackingRMSE     = 0.f;   // m
    int   collisionCount   = 0;
    bool  completed        = false;

    // Optimization
    float fitnessScore = -1.f;
};
