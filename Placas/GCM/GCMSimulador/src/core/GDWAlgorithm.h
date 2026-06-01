#pragma once
// ═══════════════════════════════════════════════════════════════════════════
//  src/core/GDWAlgorithm.h
//  GDW Design Lab — The complete GDW v4.1 planning + control stack
//
//  This module implements EXACTLY the GDW v4.1 algorithm stack:
//
//  Scout Run:  Sensors → FloodFill → D*Lite → WallCenteringPID → Explorer
//  Speed Run:  Theta* → RacingLine → Clothoids → VelocityProfile
//              → ESKF → TVLQR → PD fallback
//
//  All bugs from v4.1 provenance are preserved (FIX-A through FIX-G).
// ═══════════════════════════════════════════════════════════════════════════
#include "Types.h"
#include "MazeEngine.h"
#include "RobotDesign.h"
#include <array>
#include <optional>
#include <random>
#include <vector>

// ══════════════════════════════════════════════════════════════════════════
//  ESKF — Error-State Kalman Filter  (FIX-B applied)
//  State: nominal [x, y, θ]  +  error [δx, δy, δθ, δb_g]
// ══════════════════════════════════════════════════════════════════════════
class ESKF {
public:
    float nom_x=0.f, nom_y=0.f, nom_theta=0.f;
    std::array<float,4>  err{0,0,0,0};
    std::array<float,16> P{};

    float Q_xy    = 1e-5f;
    float Q_theta = 2e-4f;
    float Q_bias  = 1e-6f;
    float R_wall  = 9e-6f;   // SensorModel::NOISE_VAR
    float R_hdg   = 1e-4f;

    ESKF();
    void reset(float x0, float y0, float h0) noexcept;

    [[nodiscard]] float x()     const noexcept;
    [[nodiscard]] float y()     const noexcept;
    [[nodiscard]] float theta() const noexcept;
    [[nodiscard]] float bias()  const noexcept;

    // Dead-reckoning predict (ds = arc length, dtheta_meas = gyro integral, dt = seconds)
    void predict(float ds, float dtheta_meas, float dt) noexcept;

    // Wall distance update (axis 0=x, 1=y; sign +1 if wall in + direction)
    void updateWallDist(int axis, float wallCoord, float measuredDist,
                        float sign, float R_meas) noexcept;

    // Heading snap update
    void updateHeading(float measuredHeading, float R_meas) noexcept;
    void snapHeadingCardinal() noexcept;

    [[nodiscard]] float    pij(int i, int j) const noexcept { return P[i*4+j]; }
    float& pij(int i, int j)       noexcept { return P[i*4+j]; }
};

// ══════════════════════════════════════════════════════════════════════════
//  Sensor model
// ══════════════════════════════════════════════════════════════════════════
struct SensorReading { float distance=0.f; bool valid=false; };

class SensorModel {
public:
    static constexpr float RANGE_MIN  = 0.04f;
    static constexpr float RANGE_MAX  = 0.30f;
    static constexpr float NOISE_STD  = 0.003f;
    static constexpr float NOISE_VAR  = NOISE_STD * NOISE_STD;

    [[nodiscard]] SensorReading sample(float trueDistance, std::mt19937& rng) const;
    [[nodiscard]] static float measurementVariance() noexcept { return NOISE_VAR; }
};

// ══════════════════════════════════════════════════════════════════════════
//  Waypoint / trajectory types
// ══════════════════════════════════════════════════════════════════════════
struct Waypoint { float x, y, heading; };

// ══════════════════════════════════════════════════════════════════════════
//  TVLQRGain — one entry in the gain schedule
// ══════════════════════════════════════════════════════════════════════════
struct TVLQRGain {
    float K[2][3];   // 2 controls × 3 states [δx, δy, δθ]
    float arcLen;
};

// ══════════════════════════════════════════════════════════════════════════
//  TVLQRSolver  (FIX-A: B[2][0]=0, not κ)
// ══════════════════════════════════════════════════════════════════════════
class TVLQRSolver {
public:
    static constexpr float Qx  = 200.f;
    static constexpr float Qy  = 200.f;
    static constexpr float Qt  =  50.f;
    static constexpr float Rv  =   1.f;
    static constexpr float Rw  =   0.5f;

    [[nodiscard]] static std::vector<TVLQRGain> solve(
        const std::vector<TrajPoint>& traj, float wheelbase);

    static void computeControl(
        const std::vector<TVLQRGain>& gains,
        const TrajPoint& ref,
        float est_x, float est_y, float est_theta,
        float& delta_v, float& delta_omega);
};

// ══════════════════════════════════════════════════════════════════════════
//  PDController  (active in Speed Run, v4.1)
// ══════════════════════════════════════════════════════════════════════════
class PDController {
public:
    explicit PDController(const RobotParams& p) : params_(p) {}

    [[nodiscard]] std::pair<float,float> compute(
        float estX, float estY, float estTheta,
        const TrajPoint& ref, float prevCrossTrack, float dt) const noexcept;

private:
    const RobotParams& params_;
};

// ══════════════════════════════════════════════════════════════════════════
//  WallCenteringPID  (active ONLY in Scout Run, v4.1)
// ══════════════════════════════════════════════════════════════════════════
class WallCenteringPID {
public:
    explicit WallCenteringPID(const RobotParams& p) : params_(p) {}
    void reset() noexcept { integral_=0.f; }
    [[nodiscard]] float compute(float leftDist, bool leftValid,
                                 float rightDist, bool rightValid,
                                 float cellSize, float dt) noexcept;
private:
    const RobotParams& params_;
    float integral_ = 0.f;
};

// ══════════════════════════════════════════════════════════════════════════
//  Explorer  (info-theoretic + D* Lite, v4.1)
// ══════════════════════════════════════════════════════════════════════════
class Explorer {
public:
    [[nodiscard]] static float infoGain(const Maze& bot, const CellCoord& cc) noexcept;
    [[nodiscard]] static float utility (const Maze& bot, const CellCoord& cc) noexcept;

    static bool senseCell(Maze& botMaze, const Maze& truthMaze,
                           ESKF& kf, const CellCoord& cc,
                           const MazeConfig& cfg);

    [[nodiscard]] static std::vector<CellCoord> explore(
        Maze& botMaze, const Maze& truthMaze,
        ESKF& kf, const MazeConfig& cfg,
        WallCenteringPID& wallCtrl);
};

// ══════════════════════════════════════════════════════════════════════════
//  AdaptiveScaler — learns from scout run to scale speed run params
// ══════════════════════════════════════════════════════════════════════════
class AdaptiveScaler {
public:
    struct Sample { float planned, achieved, curvature; };
    std::vector<Sample> samples;

    void record(float planned, float achieved, float curvature);
    [[nodiscard]] std::pair<float,float> factors() const;
};

// ══════════════════════════════════════════════════════════════════════════
//  GDWPlannerV4 — top-level orchestrator (the v4.1 stack)
// ══════════════════════════════════════════════════════════════════════════
class GDWPlannerV4 {
public:
    MazeConfig       cfg;
    Maze             botMaze;
    Maze             truthMaze;   // must be set before calling run()
    RobotParams      robot;
    ESKF             kf;
    AdaptiveScaler   scaler;
    std::mt19937     rng{42};

    CellCoord              goalReached{-1,-1};
    std::vector<CellCoord> exploredPath;
    std::vector<CellCoord> speedPath;
    std::vector<TrajPoint> speedTraj;

    RunStats scoutStats;
    RunStats speedStats;

    bool verbose = false;  // print detailed logs

    void initialize();
    void initialize(const Maze& truth, const RobotParams& params);

    // Run full cycle: scout → return → speed
    RunStats runFull();

    // Individual phases
    RunStats scoutRun();
    RunStats returnToStart();
    RunStats speedRun();

private:
    RunStats profilePath_(const std::vector<CellCoord>& cellPath,
                          float vMax, const std::string& label,
                          bool computeTVLQR,
                          std::vector<TrajPoint>* outTraj = nullptr);
};
