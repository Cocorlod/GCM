#pragma once
// ═══════════════════════════════════════════════════════════════════════════
//  src/core/PhysicsEngine.h
//  GDW Design Lab — Physics Simulation
//
//  Simulates a differential-drive micromouse with:
//    - Rigid body dynamics (Izz, CoM, mass distribution)
//    - Wheel slip model (Pacejka-inspired, simplified)
//    - Motor dynamics (back-EMF, current limiting, PWM)
//    - Battery voltage sag
//    - Sensor noise (IR + gyro)
//    - Collision detection against maze walls
//    - Downforce (optional suction)
// ═══════════════════════════════════════════════════════════════════════════
#include "Types.h"
#include "RobotDesign.h"
#include "MazeEngine.h"
#include <random>

// ── RobotState — complete instantaneous state ─────────────────────────────
struct RobotState {
    // Pose (world frame)
    float x     = 0.f;   // m East
    float y     = 0.f;   // m North
    float theta = 0.f;   // rad (0=East, π/2=North, CCW positive)

    // Velocity
    float vx    = 0.f;   // m/s world-frame x
    float vy    = 0.f;   // m/s world-frame y
    float omega = 0.f;   // rad/s yaw rate

    // Derived velocities (body frame)
    float vLong = 0.f;   // m/s longitudinal (forward)
    float vLat  = 0.f;   // m/s lateral (left positive)

    // Accelerations (body frame, m/s²)
    float aLong = 0.f;
    float aLat  = 0.f;

    // Jerk (m/s³)
    float jerkLong = 0.f;

    // Wheel states
    float wheelVelLeft  = 0.f;  // rad/s
    float wheelVelRight = 0.f;  // rad/s
    float slipLeft      = 0.f;  // dimensionless slip ratio
    float slipRight     = 0.f;

    // Motor states
    float currentLeft   = 0.f;  // A
    float currentRight  = 0.f;
    float torqueLeft    = 0.f;  // Nm
    float torqueRight   = 0.f;

    // Battery
    float battVoltage     = 7.4f;  // V (terminal)
    float battCapacityUsed= 0.f;   // mAh

    // Energy
    float energyUsed    = 0.f;   // J total

    // Timing
    float simTime       = 0.f;   // s

    // Collision
    bool  inCollision   = false;
    int   collisionCount= 0;

    // Curvature (geometric)
    float curvature     = 0.f;   // 1/m
};

// ── MotorState — per-motor ─────────────────────────────────────────────────
struct MotorState {
    float current    = 0.f;   // A
    float backEMF    = 0.f;   // V
    float torque     = 0.f;   // Nm
    float angularVel = 0.f;   // rad/s (motor shaft)
    float temperature= 25.f;  // °C (simple thermal model)
};

// ── SensorMeasurements — noisy outputs at each timestep ───────────────────
struct SensorMeasurements {
    // IR sensors (4 sensors, indexed 0-3)
    std::array<float, 4> irDistance = {};  // m (NaN if out of range)
    std::array<bool,  4> irValid    = {};  // true if in range

    // IMU
    float gyroZ      = 0.f;   // rad/s (true + noise + bias)
    float accelX     = 0.f;   // m/s² longitudinal + noise
    float accelY     = 0.f;   // m/s² lateral + noise

    // Encoders (wheel ticks, already converted to m)
    float encoderLeft  = 0.f;  // m arc-length this timestep
    float encoderRight = 0.f;
};

// ── PhysicsConfig ──────────────────────────────────────────────────────────
struct PhysicsConfig {
    float dt       = 0.001f;  // integration timestep (s) — 1 kHz
    float cellSize = 0.18f;
    bool  noiseEnabled    = true;
    bool  collisionEnabled= true;
    bool  motorDynamics   = true;   // if false, treat motors as ideal
    float wallBounceDamp  = 0.3f;   // velocity damping on wall contact
};

// ── Noise parameters ───────────────────────────────────────────────────────
struct NoiseParams {
    float gyroNoise    = 0.002f;  // rad/s RMS
    float gyroBias     = 0.001f;  // rad/s constant drift
    float encoderNoise = 0.00015f; // m RMS per step
    float irNoise      = 0.003f;  // m RMS
    float accelNoise   = 0.05f;   // m/s² RMS
};

// ══════════════════════════════════════════════════════════════════════════
//  PhysicsEngine
// ══════════════════════════════════════════════════════════════════════════
class PhysicsEngine {
public:
    PhysicsConfig config;
    NoiseParams   noise;

    // Persistent noise state
    float gyroBiasState = 0.f;

    // ─────────────────────────────────────────────────────────────────────
    void init(const PhysicsConfig& cfg, const NoiseParams& n, unsigned seed = 42);

    // ─────────────────────────────────────────────────────────────────────
    //  Step the physics simulation by config.dt seconds.
    //
    //  Inputs:
    //    design     — robot physical parameters
    //    state      — current robot state (modified in-place)
    //    truthMaze  — ground-truth maze for wall collision + sensor rays
    //    vCmd       — commanded linear velocity (m/s) from controller
    //    omegaCmd   — commanded yaw rate (rad/s) from controller
    //
    //  Returns: SensorMeasurements (noisy sensor data for ESKF)
    // ─────────────────────────────────────────────────────────────────────
    SensorMeasurements step(const RobotDesign& design,
                             RobotState& state,
                             const Maze& truthMaze,
                             float vCmd,
                             float omegaCmd);

    // ─────────────────────────────────────────────────────────────────────
    //  Cast one IR sensor ray from (px,py) in direction (angle relative to
    //  world heading theta) and return distance to nearest wall (m).
    // ─────────────────────────────────────────────────────────────────────
    float castSensorRay(float px, float py, float rayAngle,
                        const Maze& maze, float maxRange = 0.30f) const noexcept;

    // Reset state for a new run
    static RobotState makeInitialState(const RobotDesign& design,
                                        const MazeConfig& cfg);

private:
    std::mt19937 rng_;

    // Motor integration
    void stepMotors(const RobotDesign& design,
                    RobotState& state,
                    float vCmdLeft, float vCmdRight,
                    MotorState& motorL, MotorState& motorR);

    // Wheel slip (simplified Pacejka magic formula)
    static float slipForce(float slip, float normalForce,
                             float mu, float stiffness) noexcept;

    // Collision detection against maze wall segments
    bool checkWallCollision(const RobotDesign& design,
                             RobotState& state,
                             const Maze& maze);

    // Differential-drive inverse kinematics
    static void diffDriveIK(float v, float omega,
                              float trackWidth,
                              float& vLeft, float& vRight) noexcept;

    MotorState motorL_, motorR_;
    float prevVLong_ = 0.f;
    float prevALong_ = 0.f;
};
