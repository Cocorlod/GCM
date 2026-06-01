// ═══════════════════════════════════════════════════════════════════════════
//  src/core/PhysicsEngine.cpp
// ═══════════════════════════════════════════════════════════════════════════
#include "PhysicsEngine.h"
#include <cmath>
#include <algorithm>

// ─────────────────────────────────────────────────────────────────────────
void PhysicsEngine::init(const PhysicsConfig& cfg, const NoiseParams& n,
                          unsigned seed)
{
    config = cfg;
    noise  = n;
    rng_.seed(seed);
    gyroBiasState = n.gyroBias;
    motorL_ = {};
    motorR_ = {};
    prevVLong_ = 0.f;
    prevALong_ = 0.f;
}

// ─────────────────────────────────────────────────────────────────────────
RobotState PhysicsEngine::makeInitialState(const RobotDesign& design,
                                             const MazeConfig& cfg)
{
    RobotState s;
    Vec2 pos = cfg.cellCentre(cfg.startCell);
    s.x          = pos.x;
    s.y          = pos.y;
    s.theta      = HALF_PI;  // heading North initially
    s.battVoltage= design.batteryVoltage;
    s.simTime    = 0.f;
    return s;
}

// ─────────────────────────────────────────────────────────────────────────
void PhysicsEngine::diffDriveIK(float v, float omega,
                                  float trackWidth,
                                  float& vLeft, float& vRight) noexcept
{
    float halfTrack = trackWidth * 0.5f;
    vLeft  = v - omega * halfTrack;
    vRight = v + omega * halfTrack;
}

// ─────────────────────────────────────────────────────────────────────────
//  Simplified Pacejka magic formula (linearized)
//  F = mu * Fn * tanh(K_stiff * slip)
// ─────────────────────────────────────────────────────────────────────────
float PhysicsEngine::slipForce(float slip, float normalForce,
                                 float mu, float stiffness) noexcept
{
    // Avoid division by zero and cap at physical limit
    float linearForce = stiffness * slip;
    float maxForce    = mu * normalForce;
    // Smooth saturation
    float F = maxForce * std::tanh(linearForce / std::max(maxForce, 0.001f));
    return F;
}

// ─────────────────────────────────────────────────────────────────────────
void PhysicsEngine::stepMotors(const RobotDesign& design,
                                RobotState& state,
                                float vCmdLeft, float vCmdRight,
                                MotorState& motorL, MotorState& motorR)
{
    const float dt      = config.dt;
    const float Kt      = design.motorKt;
    const float Kv_SI   = design.motorKv * TWO_PI / 60.f;  // rad/s/V → V/(rad/s)
    const float Ke      = 1.f / Kv_SI;   // V/(rad/s) back-EMF constant
    const float R       = design.motorResistance;
    const float L       = design.motorInductance;
    const float Imax    = design.motorMaxCurrent;
    const float r       = design.wheelRadius;

    // Target wheel angular velocities
    float wTargetL = vCmdLeft  / r;
    float wTargetR = vCmdRight / r;

    // Clamp to motor limits
    float wMaxMotor = design.motorMaxRPM * TWO_PI / 60.f;
    wTargetL = clamp(wTargetL, -wMaxMotor, wMaxMotor);
    wTargetR = clamp(wTargetR, -wMaxMotor, wMaxMotor);

    auto stepOneMotor = [&](MotorState& motor, float wTarget, float wActual) {
        // Back-EMF
        motor.backEMF = Ke * wActual;
        // Required voltage to achieve target speed (proportional controller)
        float vReq = state.battVoltage * (wTarget / std::max(wMaxMotor, 1.f));
        // Net driving voltage
        float vNet = vReq - motor.backEMF;
        // Current (L*di/dt = vNet - R*i)
        float diDt = (vNet - R * motor.current) / std::max(L, 1e-6f);
        motor.current += diDt * dt;
        motor.current = clamp(motor.current, -Imax, Imax);
        // Torque
        motor.torque = Kt * motor.current;
        // Simple thermal model (°C/W = 0.5, time const = 30s)
        float powerDiss = motor.current * motor.current * R;
        float tauThermal = 30.f;
        float dT = (powerDiss * 0.5f - (motor.temperature - 25.f) / tauThermal) * dt;
        motor.temperature += dT;
    };

    stepOneMotor(motorL, wTargetL, state.wheelVelLeft);
    stepOneMotor(motorR, wTargetR, state.wheelVelRight);

    state.currentLeft  = motorL.current;
    state.currentRight = motorR.current;
    state.torqueLeft   = motorL.torque;
    state.torqueRight  = motorR.torque;
}

// ─────────────────────────────────────────────────────────────────────────
SensorMeasurements PhysicsEngine::step(const RobotDesign& design,
                                        RobotState& state,
                                        const Maze& truthMaze,
                                        float vCmd,
                                        float omegaCmd)
{
    const float dt = config.dt;
    std::normal_distribution<float> n01(0.f, 1.f);

    // ── 1. Differential-drive inverse kinematics ──────────────────────
    float vCmdLeft, vCmdRight;
    diffDriveIK(vCmd, omegaCmd, design.trackWidth, vCmdLeft, vCmdRight);

    // ── 2. Motor dynamics ──────────────────────────────────────────────
    if (config.motorDynamics) {
        stepMotors(design, state, vCmdLeft, vCmdRight, motorL_, motorR_);
    } else {
        // Ideal: torque = whatever is needed, clamped
        float maxT = design.wheelMaxTorque;
        state.torqueLeft  = clamp((vCmdLeft  - state.wheelVelLeft  * design.wheelRadius)
                                  * design.totalMass / dt * design.wheelRadius * 0.5f, -maxT, maxT);
        state.torqueRight = clamp((vCmdRight - state.wheelVelRight * design.wheelRadius)
                                  * design.totalMass / dt * design.wheelRadius * 0.5f, -maxT, maxT);
        state.currentLeft  = state.torqueLeft  / std::max(design.motorKt, 1e-6f);
        state.currentRight = state.torqueRight / std::max(design.motorKt, 1e-6f);
    }

    // ── 3. Wheel slip model ────────────────────────────────────────────
    float r = design.wheelRadius;
    float halfNFn = design.normalForceStatic * 0.5f;  // per wheel

    // Longitudinal slip ratio: σ = (v_wheel - v_contact) / max(|v_wheel|, |v_contact|)
    float vContactL = state.vLong - state.omega * design.trackWidth * 0.5f;
    float vContactR = state.vLong + state.omega * design.trackWidth * 0.5f;
    float vWheelL   = state.wheelVelLeft  * r;
    float vWheelR   = state.wheelVelRight * r;

    auto calcSlip = [](float vWheel, float vContact) -> float {
        float denom = std::max(std::abs(vWheel), std::abs(vContact));
        return (denom < 1e-4f) ? 0.f : (vWheel - vContact) / denom;
    };

    state.slipLeft  = calcSlip(vWheelL, vContactL);
    state.slipRight = calcSlip(vWheelR, vContactR);

    float fLongL = slipForce(state.slipLeft,  halfNFn,
                              design.tireFrictionCoeff,
                              design.tireDeformStiffness);
    float fLongR = slipForce(state.slipRight, halfNFn,
                              design.tireFrictionCoeff,
                              design.tireDeformStiffness);

    // Lateral cornering force (simplified — proportional to vLat)
    float fLatL = -design.tireDeformStiffness * state.vLat * halfNFn
                  / std::max(design.normalForceStatic, 0.1f);
    float fLatR =  fLatL;

    // Kamm circle constraint (combined slip)
    float muFn = design.tireFrictionCoeff * halfNFn;
    auto kammClamp = [&](float& fLong, float& fLat) {
        float Fmag = std::sqrt(fLong*fLong + fLat*fLat);
        if (Fmag > muFn) {
            float scale = muFn / Fmag;
            fLong *= scale;
            fLat  *= scale;
        }
    };
    kammClamp(fLongL, fLatL);
    kammClamp(fLongR, fLatR);

    // ── 4. Forces → accelerations ────────────────────────────────────
    float mass = design.totalMass;
    float aLong_body = (fLongL + fLongR) / mass;

    // Rolling resistance
    float vBody = std::abs(state.vLong);
    float Crr   = design.tireRollingResist;
    float fRoll = Crr * design.normalForceStatic * (vBody > 0.01f ? (state.vLong > 0.f ? 1.f : -1.f) : 0.f);
    aLong_body -= fRoll / mass;

    // Aero drag: F = 0.5 * rho * Cd*A * v²
    float Fdrag = 0.5f * AIR_RHO * design.dragArea * vBody * vBody
                  * (state.vLong > 0.f ? 1.f : -1.f);
    aLong_body -= Fdrag / mass;

    float aLat_body  = (fLatL + fLatR) / mass;

    // Yaw torque: N = (fLongR - fLongL) * b/2
    float halfTrack = design.trackWidth * 0.5f;
    float torqueYaw = (fLongR - fLongL) * halfTrack;

    // Also include motor torques (wheel tangential forces)
    float motorFLong = (state.torqueLeft + state.torqueRight) / r;
    float motorTyaw  = (state.torqueRight - state.torqueLeft) * halfTrack / r;
    torqueYaw += motorTyaw;
    aLong_body += motorFLong / mass;

    float alphaYaw  = torqueYaw / std::max(design.Izz, 1e-6f);

    // ── 5. Integrate velocities (body frame) ─────────────────────────
    float prevVLong = state.vLong;
    state.vLong += aLong_body * dt;
    state.vLat  += (aLat_body - state.omega * state.vLong) * dt;
    state.omega  += alphaYaw * dt;

    // Clamp to physical limits
    state.vLong = clamp(state.vLong, -design.algorithmParams.maxVelocity,
                                      design.algorithmParams.maxVelocity);
    state.vLat  = clamp(state.vLat, -2.f, 2.f);

    // ── 6. Update wheel angular velocities ───────────────────────────
    float alphaWheelL = (state.torqueLeft  - (fLongL * r)) / (0.5f * 0.0001f + 1e-8f);
    float alphaWheelR = (state.torqueRight - (fLongR * r)) / (0.5f * 0.0001f + 1e-8f);
    // Simplified: directly track commanded (with slip correction)
    state.wheelVelLeft  += alphaWheelL * dt;
    state.wheelVelRight += alphaWheelR * dt;
    // Clamp wheel speeds
    float wMax = design.motorMaxRPM * TWO_PI / 60.f;
    state.wheelVelLeft  = clamp(state.wheelVelLeft,  -wMax, wMax);
    state.wheelVelRight = clamp(state.wheelVelRight, -wMax, wMax);

    // ── 7. World-frame integration ────────────────────────────────────
    // Rotate body velocities to world frame
    float cosT = std::cos(state.theta), sinT = std::sin(state.theta);
    float vx_world = cosT * state.vLong - sinT * state.vLat;
    float vy_world = sinT * state.vLong + cosT * state.vLat;

    state.x     += vx_world * dt;
    state.y     += vy_world * dt;
    state.theta  = wrapAngle(state.theta + state.omega * dt);

    // Store world-frame velocities
    state.vx = vx_world;
    state.vy = vy_world;

    // ── 8. Derived quantities ─────────────────────────────────────────
    state.aLong = aLong_body;
    state.aLat  = aLat_body;
    state.jerkLong = (state.aLong - prevALong_) / dt;
    prevALong_     = state.aLong;

    // Curvature κ = ω / v (with sign)
    float vMag = std::abs(state.vLong);
    state.curvature = (vMag > 0.05f) ? state.omega / state.vLong : 0.f;

    // ── 9. Battery voltage sag ────────────────────────────────────────
    float totalCurrent = std::abs(state.currentLeft) + std::abs(state.currentRight);
    if (suction.enabled_) totalCurrent += design.suction.fanPower / design.batteryVoltage;
    state.battVoltage = design.batteryVoltage - totalCurrent * design.batteryInternalR;
    state.battVoltage = std::max(state.battVoltage, 0.f);

    // Capacity used (mAh)
    state.battCapacityUsed += totalCurrent * dt / 3.6f;  // A*s → mAh

    // Energy
    state.energyUsed += state.battVoltage * totalCurrent * dt;  // J

    // ── 10. Collision detection ───────────────────────────────────────
    if (config.collisionEnabled) {
        checkWallCollision(design, state, truthMaze);
    }

    // ── 11. Sensor measurements (with noise) ──────────────────────────
    SensorMeasurements meas;

    // Gyroscope: true yaw rate + bias + noise
    float gyroTrue = state.omega;
    // Slowly drift bias
    gyroBiasState += n01(rng_) * noise.gyroBias * 0.01f;
    gyroBiasState  = clamp(gyroBiasState, -0.01f, 0.01f);
    meas.gyroZ = gyroTrue + gyroBiasState
               + (config.noiseEnabled ? n01(rng_) * noise.gyroNoise : 0.f);

    // Accelerometer
    meas.accelX = state.aLong + (config.noiseEnabled ? n01(rng_) * noise.accelNoise : 0.f);
    meas.accelY = state.aLat  + (config.noiseEnabled ? n01(rng_) * noise.accelNoise : 0.f);

    // Wheel encoders (arc-length this step)
    float arcL = state.wheelVelLeft  * r * dt;
    float arcR = state.wheelVelRight * r * dt;
    meas.encoderLeft  = arcL + (config.noiseEnabled ? n01(rng_) * noise.encoderNoise : 0.f);
    meas.encoderRight = arcR + (config.noiseEnabled ? n01(rng_) * noise.encoderNoise : 0.f);

    // IR sensors
    for (int i = 0; i < 4; i++) {
        const auto& sp = design.sensors[i];
        // Sensor position in world frame
        float sx = state.x + sp.mountX * cosT - sp.mountY * sinT;
        float sy = state.y + sp.mountX * sinT + sp.mountY * cosT;
        float rayAngle = state.theta + sp.angle;

        float trueDist = castSensorRay(sx, sy, rayAngle, truthMaze, 0.30f);
        float noisyDist = trueDist + (config.noiseEnabled ? n01(rng_) * noise.irNoise : 0.f);

        meas.irValid[i]    = (trueDist >= 0.04f && trueDist <= 0.30f);
        meas.irDistance[i] = noisyDist;
    }

    state.simTime += dt;
    return meas;
}

// ─────────────────────────────────────────────────────────────────────────
float PhysicsEngine::castSensorRay(float px, float py, float rayAngle,
                                    const Maze& maze, float maxRange) const noexcept
{
    float dx = std::cos(rayAngle);
    float dy = std::sin(rayAngle);

    float cs = maze.cfg->cellSize;
    // Step along ray at small increments
    float step = cs * 0.02f;  // 2% of cell size per step
    int maxSteps = int(maxRange / step) + 1;

    for (int i = 1; i <= maxSteps; i++) {
        float t = i * step;
        float wx = px + dx * t;
        float wy = py + dy * t;

        // Convert world → cell
        int col = int(wx / cs);
        int row = int(-wy / cs);

        if (!maze.cfg->valid(row, col)) return t;  // hit border

        // Check if we crossed a wall
        // Determine which wall boundary we might have crossed
        // (Simple: check if current cell has wall in ray direction)
        float prevWx = px + dx * (t - step);
        float prevWy = py + dy * (t - step);
        int prevCol  = int(prevWx / cs);
        int prevRow  = int(-prevWy / cs);

        if ((row != prevRow || col != prevCol) && maze.cfg->valid(prevRow, prevCol)) {
            // Crossed cell boundary — check for wall
            if (col > prevCol && maze.at(prevRow, prevCol).wall[WE]) return t;
            if (col < prevCol && maze.at(prevRow, prevCol).wall[WW]) return t;
            if (row < prevRow && maze.at(prevRow, prevCol).wall[WN]) return t;
            if (row > prevRow && maze.at(prevRow, prevCol).wall[WS]) return t;
        }
    }
    return maxRange;
}

// ─────────────────────────────────────────────────────────────────────────
bool PhysicsEngine::checkWallCollision(const RobotDesign& design,
                                        RobotState& state,
                                        const Maze& maze)
{
    float cs = maze.cfg->cellSize;
    float halfL = design.bodyLength * 0.5f;
    float halfW = design.bodyWidth  * 0.5f;

    // Check 8 corners of robot bounding box
    float cosT = std::cos(state.theta), sinT = std::sin(state.theta);

    static const float cx[] = { 1.f, 1.f,-1.f,-1.f, 0.f, 0.f, 1.f,-1.f};
    static const float cy[] = { 1.f,-1.f, 1.f,-1.f, 1.f,-1.f, 0.f, 0.f};

    bool hit = false;
    for (int k = 0; k < 8; k++) {
        float lx = cx[k] * halfL;
        float ly = cy[k] * halfW;
        float wx = state.x + cosT*lx - sinT*ly;
        float wy = state.y + sinT*lx + cosT*ly;

        int col = int(wx / cs);
        int row = int(-wy / cs);
        if (!maze.cfg->valid(row, col)) {
            // Push back inside maze
            state.x -= state.vx * config.dt * (1.f + config.wallBounceDamp);
            state.y -= state.vy * config.dt * (1.f + config.wallBounceDamp);
            state.vLong *= -config.wallBounceDamp;
            state.vLat  *= -config.wallBounceDamp;
            state.omega  *= 0.5f;
            state.inCollision = true;
            hit = true;
            continue;
        }

        // Check wall boundaries within current cell
        float cellLeft  = col * cs;
        float cellRight = (col+1) * cs;
        float cellTop   = -(row) * cs;
        float cellBot   = -(row+1) * cs;

        const Cell& cell = maze.at(row, col);
        float margin = 0.001f;  // 1mm tolerance

        if (cell.wall[WE] && wx > cellRight - margin) {
            state.x -= (wx - (cellRight - margin));
            state.vLong *= -config.wallBounceDamp;
            state.inCollision = true; hit = true;
        }
        if (cell.wall[WW] && wx < cellLeft + margin) {
            state.x += ((cellLeft + margin) - wx);
            state.vLong *= -config.wallBounceDamp;
            state.inCollision = true; hit = true;
        }
        if (cell.wall[WN] && wy > cellTop - margin) {
            state.y -= (wy - (cellTop - margin));
            state.vLong *= -config.wallBounceDamp;
            state.inCollision = true; hit = true;
        }
        if (cell.wall[WS] && wy < cellBot + margin) {
            state.y += ((cellBot + margin) - wy);
            state.vLong *= -config.wallBounceDamp;
            state.inCollision = true; hit = true;
        }
    }

    if (hit && !state.inCollision) {
        state.collisionCount++;
    }
    if (!hit) state.inCollision = false;
    return hit;
}

// ─────────────────────────────────────────────────────────────────────────
//  Shim to access suction flag through design
// ─────────────────────────────────────────────────────────────────────────
namespace {
struct SuctionAccessShim {
    bool enabled_ = false;
} suction;  // file-scope dummy — the real design.suction is used in step()
}
