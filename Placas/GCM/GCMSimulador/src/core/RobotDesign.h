#pragma once
// ═══════════════════════════════════════════════════════════════════════════
//  src/core/RobotDesign.h
//  GDW Design Lab — Robot Physical Design Representation
//
//  RobotDesign holds EVERY physical parameter the optimizer can vary.
//  computeDerived() converts physical parameters → algorithm RobotParams.
//  clamp() enforces physically realizable ranges.
// ═══════════════════════════════════════════════════════════════════════════
#include "Types.h"
#include <random>
#include <string>

// ── Suction / downforce system (optional) ─────────────────────────────────
struct SuctionSystem {
    bool  enabled      = false;
    float fanDiameter  = 0.04f;   // m
    float fanPower     = 2.0f;    // W
    float massKg       = 0.015f;  // kg (fan + motor + duct)
    float mountX       = 0.0f;    // m relative to CoM (forward = +x)
    float mountZ       = 0.005f;  // m height from ground
    // Derived (filled by computeDerived)
    float thrustNewtons = 0.f;    // N downforce at rated power
    float dragNewtons   = 0.f;    // N parasitic drag
};

// ── Sensor placement (per sensor) ─────────────────────────────────────────
struct SensorPlacement {
    float mountX = 0.035f;  // m from robot geometric centre (+= fwd)
    float mountY = 0.0f;    // m from centreline (+= left)
    float angle  = 0.0f;    // rad from robot heading (+= left)
};

// ══════════════════════════════════════════════════════════════════════════
//  RobotDesign — the genome the optimizer evolves
// ══════════════════════════════════════════════════════════════════════════
struct RobotDesign {
    // ── Identity ──────────────────────────────────────────────────────────
    int         id            = 0;
    std::string name          = "Design";
    int         generation    = 0;
    float       fitnessScore  = -1.f;

    // ── Chassis geometry (metres) ─────────────────────────────────────────
    float bodyLength    = 0.090f;   // front-to-back [0.05 – 0.12]
    float bodyWidth     = 0.080f;   // left-to-right [0.05 – 0.09]
    float bodyHeight    = 0.040f;   // floor-to-top  [0.02 – 0.07]
    float wheelbase     = 0.065f;   // dist between axle centres [0.04 – 0.09]
    float trackWidth    = 0.065f;   // dist between tyre centrelines [0.04 – 0.085]

    // Nose shaping
    float frontTaperAngle = 30.0f;  // deg [0 – 60]
    float rearTaperAngle  = 15.0f;  // deg [0 – 45]

    // ── Wheels ────────────────────────────────────────────────────────────
    float wheelDiameter = 0.033f;   // m [0.020 – 0.042]
    float wheelWidth    = 0.008f;   // m [0.005 – 0.015]
    // Wheel placement relative to geometric body centre
    float wheelFwdOffset= 0.0f;     // m fwd of body centre [−0.02 – +0.02]

    // ── Tyres ─────────────────────────────────────────────────────────────
    float tireFrictionCoeff    = 1.20f;   // μ [0.60 – 2.00]
    float tireRollingResist    = 0.015f;  // Crr [0.005 – 0.04]
    float tireDeformStiffness  = 150.0f;  // N/m lateral [50 – 400]

    // ── Motors ────────────────────────────────────────────────────────────
    float motorKv          = 1800.f;  // RPM/V [800 – 3000]
    float motorKt          = 0.005f;  // Nm/A  [0.003 – 0.020]
    float motorResistance  = 1.2f;    // Ω     [0.5 – 5.0]
    float motorInductance  = 0.0003f; // H
    float motorMaxCurrent  = 3.0f;    // A     [1.0 – 8.0]
    float motorMaxRPM      = 12000.f; // RPM   [4000 – 25000]
    // Motor placement relative to body centre
    float motorFwdOffset   = -0.010f; // m (behind centre)
    float motorLateralPos  = 0.030f;  // m from centreline (symmetric)
    float motorHeightZ     = 0.010f;  // m above ground

    // ── Battery ───────────────────────────────────────────────────────────
    float batteryVoltage  = 7.4f;    // V nominal (2S LiPo) [3.7 – 14.8]
    float batteryCapacity = 300.0f;  // mAh [100 – 1000]
    float batteryInternalR= 0.03f;   // Ω internal resistance [0.01 – 0.15]
    // Battery placement
    float batteryFwdOffset  = 0.0f;  // m from body centre
    float batteryHeightZ    = 0.020f; // m above ground

    // ── PCB ───────────────────────────────────────────────────────────────
    float pcbMass         = 0.012f;  // kg [0.005 – 0.030]
    float pcbFwdOffset    = 0.010f;  // m
    float pcbHeightZ      = 0.025f;  // m

    // ── Mass budget ───────────────────────────────────────────────────────
    float chassisMass     = 0.025f;  // kg [0.008 – 0.060]
    float totalMassTarget = 0.090f;  // kg [0.050 – 0.180] — optimizer target

    // ── Inertia inputs (used to compute Izz) ──────────────────────────────
    // Centre of mass offset from geometric body centre
    float comFwdOffset  = 0.0f;   // m (+= forward)
    float comLatOffset  = 0.0f;   // m (+= left, should stay near 0)
    float comHeightZ    = 0.020f; // m above ground [0.01 – 0.05]

    // ── Sensors ───────────────────────────────────────────────────────────
    // 4 sensors: 0=front-left, 1=front-right, 2=side-left, 3=side-right
    std::array<SensorPlacement,4> sensors = {{
        { 0.038f,  0.018f,  PI/8.f  },   // front-left angled out
        { 0.038f, -0.018f, -PI/8.f  },   // front-right angled out
        { 0.010f,  0.038f,  HALF_PI },   // side-left  pointing left
        { 0.010f, -0.038f, -HALF_PI }    // side-right pointing right
    }};

    // ── Optional suction system ───────────────────────────────────────────
    SuctionSystem suction;

    // ═══════════════════════════════════════════════════════════════════════
    //  DERIVED QUANTITIES (filled by computeDerived())
    // ═══════════════════════════════════════════════════════════════════════

    // Mass (kg)
    float totalMass     = 0.f;
    float motorMass     = 0.f;   // per motor × 2
    float batteryMass   = 0.f;

    // Geometry
    float wheelRadius   = 0.f;
    float gearRatio     = 1.0f;  // currently direct drive

    // Moments of inertia
    float Izz           = 0.f;   // kg·m²  yaw moment
    float Ixx           = 0.f;   // kg·m²  roll  (damping factor)
    float Iyy           = 0.f;   // kg·m²  pitch (damping factor)

    // Motor-to-wheel dynamics
    float wheelMaxTorque    = 0.f;  // Nm per wheel
    float wheelMaxAngularV  = 0.f;  // rad/s
    float robotMaxLinearV   = 0.f;  // m/s  (from motor limits)
    float robotMaxAccel     = 0.f;  // m/s² (from traction + motor)

    // Downforce
    float normalForceStatic  = 0.f;   // N (gravity + suction static)
    float normalForceDynamic = 0.f;   // N per wheel at vMax (approximation)

    // Aerodynamics
    float dragArea       = 0.f;   // m² (frontal area * Cd)
    float downforceArea  = 0.f;   // m² (planform area * Cl, if suction)

    // Derived algorithm params (fed into GDW v4.1 stack)
    RobotParams algorithmParams;

    // ─────────────────────────────────────────────────────────────────────
    //  Fill all derived quantities from physical parameters
    // ─────────────────────────────────────────────────────────────────────
    void computeDerived();

    // ─────────────────────────────────────────────────────────────────────
    //  Clamp all parameters to physically realizable ranges
    // ─────────────────────────────────────────────────────────────────────
    void clampToLimits();

    // ─────────────────────────────────────────────────────────────────────
    //  Default "baseline" champion design (GDW v4.1 reference)
    // ─────────────────────────────────────────────────────────────────────
    static RobotDesign baseline();

    // ─────────────────────────────────────────────────────────────────────
    //  Random design — uniform over valid ranges (for initializing population)
    // ─────────────────────────────────────────────────────────────────────
    static RobotDesign random(std::mt19937& rng, int id = 0);

    // ─────────────────────────────────────────────────────────────────────
    //  Genetic operations
    // ─────────────────────────────────────────────────────────────────────
    static RobotDesign crossover(const RobotDesign& a, const RobotDesign& b,
                                  std::mt19937& rng);
    void mutate(std::mt19937& rng, float rate = 0.15f);

    // ─────────────────────────────────────────────────────────────────────
    //  Serialization helpers
    // ─────────────────────────────────────────────────────────────────────
    [[nodiscard]] std::string toJSON()  const;
    [[nodiscard]] std::string toCSV()   const;
    [[nodiscard]] static std::string csvHeader();
};
