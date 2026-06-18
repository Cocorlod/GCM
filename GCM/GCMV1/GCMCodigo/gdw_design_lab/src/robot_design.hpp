// ═══════════════════════════════════════════════════════════════════════════
//  robot_design.hpp — RobotDesign genome + physics derivation
//
//  RobotDesign holds every physical parameter the optimizer may vary. physics::
//  derive() maps a design to (a) the gdw::RobotParams the planner needs and
//  (b) a PhysicsModel of the closed-form limits the executor/fitness use:
//  traction (Kamm radius), motor-torque-limited accel, top speed, downforce,
//  inertia, energy. The mapping is grounded in first-principles vehicle
//  dynamics with documented simplifying assumptions — enough that the GA finds
//  physically meaningful trade-offs (mass vs grip vs downforce vs wheel size),
//  not arbitrary ones.
// ═══════════════════════════════════════════════════════════════════════════
#pragma once
#include "gdw_core.hpp"
#include <array>
#include <cmath>
#include <random>
#include <string>

namespace gdw {

inline constexpr float GRAV = 9.81f;

// ── Design genome ──────────────────────────────────────────────────────────
// Lengths in metres, masses in kg, angles in radians unless noted. Placements
// are body-frame offsets from the geometric centre (x: +forward, y: +left).
struct RobotDesign {
    // Chassis geometry
    float length        = 0.100f;  // m
    float width         = 0.070f;  // m
    float height        = 0.020f;  // m
    float noseTaper     = 0.30f;   // 0..1 fraction of length tapered at front
    float frontTaperAng = 0.35f;   // rad
    float rearTaperAng  = 0.20f;   // rad

    // Drivetrain geometry
    float wheelbase     = 0.070f;  // m (front-rear axle separation; ref/export)
    float trackWidth    = 0.060f;  // m (left-right wheel separation)
    float wheelDiameter = 0.024f;  // m
    float wheelWidth    = 0.008f;  // m

    // Placements (body-frame, m)
    float motorX = 0.0f,  motorY = 0.030f;   // motors near rear axle, ±track/2 in y
    float batteryX = -0.015f, batteryY = 0.0f;
    float pcbX = 0.005f,  pcbY = 0.0f;
    // Two side IR sensors + diagonals + front; positions relative to nose
    std::array<float,5> sensorX = { 0.045f, 0.045f, 0.040f, 0.040f, 0.048f };
    std::array<float,5> sensorY = { 0.030f,-0.030f, 0.020f,-0.020f, 0.000f };
    std::array<float,5> sensorAng = { HALF_PI, -HALF_PI, 0.6f, -0.6f, 0.0f };

    // Mass budget (kg)
    float massChassis = 0.040f;
    float massMotors  = 0.020f;
    float massBattery = 0.018f;
    float massPCB     = 0.012f;
    float massWheels  = 0.010f;
    float comOffsetX  = 0.0f;   // additional COM bias (m), tunable
    float inertiaScale= 1.0f;   // multiplier on slab-estimate yaw inertia

    // Tyres
    float tireFriction = 1.10f; // μ (silicone micromouse tyres reach ~1.0–1.4)

    // Motors / battery
    float motorStallTorque = 0.012f;  // N·m per motor (at wheel after gearing)
    float motorFreeSpeed   = 900.0f;  // rad/s wheel free speed (after gearing)
    int   cellCount        = 2;       // LiPo cells in series
    float battInternalR    = 0.090f;  // ohm pack internal resistance
    float battCapacity_mAh = 200.0f;

    // Optional aero / downforce
    bool  hasFan        = false;
    float fanPower_W    = 0.0f;     // electrical W into impeller (if hasFan)
    float fanMass       = 0.012f;   // added when hasFan
    bool  hasSuction    = false;
    float suctionArea   = 0.0030f;  // m² skirt area
    float suctionPressure = 1500.f; // Pa depression (if hasSuction)
    float suctionMass   = 0.015f;

    [[nodiscard]] float totalMass() const noexcept {
        float m = massChassis + massMotors + massBattery + massPCB + massWheels;
        if (hasFan)     m += fanMass;
        if (hasSuction) m += suctionMass;
        return m;
    }
    [[nodiscard]] float wheelRadius() const noexcept { return 0.5f * wheelDiameter; }
};

// ── Bounds for each tunable gene (used by GA mutation/init/clamp) ───────────
struct DesignBounds {
    float lengthMin=0.070f,  lengthMax=0.130f;
    float widthMin =0.050f,  widthMax =0.090f;
    float wheelDiaMin=0.016f,wheelDiaMax=0.040f;
    float wheelWidMin=0.004f,wheelWidMax=0.014f;
    float trackMin=0.045f,   trackMax=0.080f;
    float wheelbaseMin=0.045f,wheelbaseMax=0.090f;
    float muMin=0.70f,       muMax=1.40f;
    float stallMin=0.006f,   stallMax=0.030f;
    float freeSpdMin=500.f,  freeSpdMax=1400.f;
    float battRmin=0.04f,    battRmax=0.20f;
    float massChMin=0.025f,  massChMax=0.070f;
    float massBtMin=0.010f,  massBtMax=0.035f;
    float fanPwrMin=0.0f,    fanPwrMax=40.0f;
    float suctPaMin=0.0f,    suctPaMax=3500.0f;
    float comMin=-0.020f,    comMax=0.020f;
    float inertiaMin=0.6f,   inertiaMax=1.6f;
};

// ── Derived physics model ───────────────────────────────────────────────────
struct PhysicsModel {
    float mass=0;          // kg
    float yawInertia=0;    // kg·m²
    float mu=0;            // tyre friction
    float normalForce=0;   // N (weight + downforce)
    float downforce=0;     // N
    float aLatMax=0;       // m/s² traction-limited lateral (Kamm radius)
    float aMotorMax=0;     // m/s² motor-torque-limited longitudinal
    float aLongAccel=0;    // m/s² usable forward accel = min(traction, motor)
    float aBrake=0;        // m/s² braking (traction-limited)
    float vMax=0;          // m/s top speed (motor/battery limited)
    float maxJerk=0;       // m/s³
    float battVoltage=0;   // V nominal
    float fanElecPower=0;  // W continuous draw of downforce system
    float comX=0;          // m COM x (body frame, +forward)
    float tractionMargin=0;// ratio aLatMax / (mu*g) — >1 means downforce helps
};

namespace physics {

// Compute yaw moment of inertia as a thin rectangular slab + lumped masses.
[[nodiscard]] inline float yawInertia(const RobotDesign& d) {
    float m = d.totalMass();
    float Islab = (1.0f/12.0f) * m * (d.length*d.length + d.width*d.width);
    return Islab * d.inertiaScale;
}

[[nodiscard]] inline float comX(const RobotDesign& d) {
    float m = d.totalMass(); if (m < 1e-6f) return 0.f;
    float mx = d.massBattery*d.batteryX + d.massPCB*d.pcbX + d.massMotors*d.motorX;
    return mx / m + d.comOffsetX;
}

[[nodiscard]] inline PhysicsModel derive(const RobotDesign& d) {
    PhysicsModel p;
    p.mass = d.totalMass();
    p.yawInertia = yawInertia(d);
    p.mu = d.tireFriction;
    p.comX = comX(d);

    // Battery: nominal voltage, sag handled inside accel via internal R.
    p.battVoltage = 3.7f * float(d.cellCount);

    // Downforce from fan and/or skirt suction.
    float downforce = 0.f;
    float fanElec = 0.f;
    if (d.hasFan) {
        // Impeller: thrust ≈ k * P_elec (efficiency lumped). ~0.05 N per W is a
        // generous-but-not-absurd figure for a small high-rpm micromouse fan.
        const float fanThrustPerWatt = 0.05f;
        downforce += fanThrustPerWatt * d.fanPower_W;
        fanElec   += d.fanPower_W;
    }
    if (d.hasSuction) {
        downforce += d.suctionPressure * d.suctionArea;  // F = ΔP · A
        // A suction skirt needs a fan too; charge ~half the fan model.
        fanElec   += 0.5f * d.suctionPressure * d.suctionArea; // crude W proxy
    }
    p.downforce = downforce;
    p.fanElecPower = fanElec;
    p.normalForce = p.mass * GRAV + downforce;

    // Traction-limited lateral accel (Kamm circle radius):
    //   a_lat_max = μ·N / m = μ·(g + F_down/m)
    p.aLatMax = (p.mass > 1e-6f) ? p.mu * (GRAV + downforce / p.mass) : p.mu * GRAV;
    p.aLatMax = std::min(p.aLatMax, 40.0f);   // physical ceiling (~4 g) — keeps
                                              // trajectory generation well-conditioned
    p.tractionMargin = p.aLatMax / (p.mu * GRAV);

    // Motor-torque-limited accel, with battery sag at peak current.
    //   F_motor = 2 · T / r_wheel  (two driven wheels)
    //   sag derate: under load, usable torque drops ~ V_load/V_nom.
    float r = d.wheelRadius();
    float sag = 1.0f / (1.0f + d.battInternalR * 6.0f);   // crude load derate factor
    float fMotor = 2.0f * d.motorStallTorque * sag / std::max(r, 1e-3f);
    p.aMotorMax = (p.mass > 1e-6f) ? fMotor / p.mass : 0.f;

    // Usable forward accel cannot exceed traction OR motor.
    p.aLongAccel = std::min(p.aLatMax, p.aMotorMax);
    // Braking is traction-limited (assume capable brakes/regen).
    p.aBrake = p.aLatMax;

    // Top speed: wheel free speed × radius, derated by sag; never beyond what
    // traction can sustain on the fastest straight (planner caps corners).
    float vMotor = d.motorFreeSpeed * r * std::sqrt(sag);
    p.vMax = std::min(vMotor, 8.0f);   // 8 m/s hard sanity cap for a 16×16

    // Jerk limit scales with available accel and steering bandwidth proxy.
    p.maxJerk = std::max(20.0f, 6.0f * p.aLatMax);

    return p;
}

// Map design + physics into the planner's RobotParams.
[[nodiscard]] inline RobotParams toRobotParams(const RobotDesign& d, const PhysicsModel& p,
                                               float cellSize) {
    RobotParams rp;
    rp.maxTotalAccel   = std::max(1.0f, p.aLatMax);
    rp.maxBraking      = std::max(1.0f, p.aBrake);
    rp.maxAccelFwd     = std::max(1.0f, p.aLongAccel);
    rp.maxJerk         = p.maxJerk;
    rp.maxVelocity     = std::max(0.5f, p.vMax);
    rp.exploreVelocity = std::min(0.8f, 0.25f * p.vMax + 0.3f);
    rp.wheelbase       = d.wheelbase;
    rp.trackWidth      = d.trackWidth;
    rp.cellSize        = cellSize;
    // Steering bandwidth: smaller, lighter robots steer faster.
    rp.steeringBandwidth = std::max(8.0f, 25.0f * (0.05f / std::max(p.yawInertia*100.f, 0.5f)) + 12.0f);
    return rp;
}

} // namespace physics
} // namespace gdw