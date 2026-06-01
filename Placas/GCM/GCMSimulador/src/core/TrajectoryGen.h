#pragma once
// ═══════════════════════════════════════════════════════════════════════════
//  src/core/TrajectoryGen.h
//  GDW Design Lab — Trajectory generation subsystem
// ═══════════════════════════════════════════════════════════════════════════
#include "Types.h"
#include "MazeEngine.h"
#include <vector>

// ── ClothoidSeg — 8-point Gauss-Legendre clothoid ────────────────────────
struct ClothoidSeg {
    float x0, y0, theta0, kappa0, kappaEnd, length;

    static constexpr float GL_XI[8] = {
        0.0950125098f,0.2816035508f,0.4580167777f,0.6178762444f,
        0.7554044084f,0.8656312024f,0.9445750231f,0.9894009350f
    };
    static constexpr float GL_W[8] = {
        0.1894506105f,0.1826034150f,0.1691565194f,0.1495959889f,
        0.1246289463f,0.0951585117f,0.0622535239f,0.0271524594f
    };

    struct State { float x, y, theta, kappa; };
    [[nodiscard]] State eval(float s) const noexcept;
};

// ── Waypoint ─────────────────────────────────────────────────────────────
struct Waypoint { float x, y, heading; };

// ── PathUtils ────────────────────────────────────────────────────────────
struct PathUtils {
    [[nodiscard]] static std::vector<Waypoint> pathToWaypoints(
        const std::vector<CellCoord>& path, const MazeConfig& cfg);
};

// ── RacingLineOptimiser ───────────────────────────────────────────────────
class RacingLineOptimiser {
public:
    // Minimise second-difference (curvature proxy) via L-BFGS-lite gradient descent.
    // halfWidth: corridor half-width in metres.
    // margin: safety margin inward from wall.
    [[nodiscard]] static std::vector<Waypoint> optimise(
        std::vector<Waypoint> wps,
        float halfWidth,
        float margin = 0.025f,
        int   maxIter = 80);
};

// ── TrajGen — clothoid-arc-clothoid generator ─────────────────────────────
class TrajGen {
public:
    static constexpr int SAMPLES_STRAIGHT = 10;
    static constexpr int SAMPLES_CLOTHOID = 24;
    static constexpr int SAMPLES_ARC      = 20;

    [[nodiscard]] static std::vector<TrajPoint> generate(
        const std::vector<Waypoint>& wps, const RobotParams& robot);
};

// ── VelocityProfile ───────────────────────────────────────────────────────
class VelocityProfile {
public:
    [[nodiscard]] static float kammLong(float kappa, float v, float aTotal) noexcept;
    [[nodiscard]] static float vMaxCurv(float kappa, float aTotal) noexcept;

    static void curvatureCeilings(std::vector<TrajPoint>& traj,
                                   float vMax, float aTotal);
    static void globalBrakingPass(std::vector<TrajPoint>& traj,
                                   float aTotal, float aBrakeMax);
    static void backwardPass     (std::vector<TrajPoint>& traj,
                                   float maxJerk, float aTotal, float aBrakeMax,
                                   int maxIter=25);
    static void forwardPass      (std::vector<TrajPoint>& traj,
                                   float maxJerk, float aTotal, float aAccelMax);
    static void computeJerk      (std::vector<TrajPoint>& traj);

    [[nodiscard]] static float estimateTime    (const std::vector<TrajPoint>& traj);
    [[nodiscard]] static float peakLatAccel    (const std::vector<TrajPoint>& traj);
    [[nodiscard]] static float peakLongAccel   (const std::vector<TrajPoint>& traj);
    [[nodiscard]] static float peakJerk        (const std::vector<TrajPoint>& traj);
};
