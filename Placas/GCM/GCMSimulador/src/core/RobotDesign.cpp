// ═══════════════════════════════════════════════════════════════════════════
//  src/core/RobotDesign.cpp
// ═══════════════════════════════════════════════════════════════════════════
#include "RobotDesign.h"
#include <cmath>
#include <sstream>
#include <iomanip>

// ─────────────────────────────────────────────────────────────────────────
void RobotDesign::clampToLimits()
{
    bodyLength    = clamp(bodyLength,   0.050f, 0.120f);
    bodyWidth     = clamp(bodyWidth,    0.050f, 0.090f);
    bodyHeight    = clamp(bodyHeight,   0.020f, 0.070f);
    wheelbase     = clamp(wheelbase,    0.040f, bodyLength * 0.95f);
    trackWidth    = clamp(trackWidth,   0.040f, bodyWidth  * 0.98f);

    frontTaperAngle = clamp(frontTaperAngle, 0.f, 60.f);
    rearTaperAngle  = clamp(rearTaperAngle,  0.f, 45.f);

    wheelDiameter  = clamp(wheelDiameter,  0.020f, 0.042f);
    wheelWidth     = clamp(wheelWidth,     0.005f, 0.015f);
    wheelFwdOffset = clamp(wheelFwdOffset,-0.020f, 0.020f);

    tireFrictionCoeff   = clamp(tireFrictionCoeff,   0.60f, 2.00f);
    tireRollingResist   = clamp(tireRollingResist,   0.005f, 0.040f);
    tireDeformStiffness = clamp(tireDeformStiffness, 50.f,  400.f);

    motorKv         = clamp(motorKv,         800.f,  3000.f);
    motorKt         = clamp(motorKt,         0.003f, 0.020f);
    motorResistance = clamp(motorResistance, 0.5f,   5.0f);
    motorMaxCurrent = clamp(motorMaxCurrent, 1.0f,   8.0f);
    motorMaxRPM     = clamp(motorMaxRPM,     4000.f, 25000.f);
    motorFwdOffset  = clamp(motorFwdOffset, -0.040f, 0.020f);
    motorLateralPos = clamp(motorLateralPos, trackWidth*0.4f, trackWidth*0.6f);
    motorHeightZ    = clamp(motorHeightZ,    0.005f, 0.030f);

    batteryVoltage   = clamp(batteryVoltage,   3.7f, 14.8f);
    batteryCapacity  = clamp(batteryCapacity,  100.f, 1000.f);
    batteryInternalR = clamp(batteryInternalR, 0.01f, 0.15f);
    batteryHeightZ   = clamp(batteryHeightZ,   0.005f, bodyHeight - 0.005f);

    pcbMass       = clamp(pcbMass,       0.005f, 0.030f);
    pcbHeightZ    = clamp(pcbHeightZ,    0.005f, bodyHeight - 0.005f);

    chassisMass     = clamp(chassisMass,     0.008f, 0.060f);
    totalMassTarget = clamp(totalMassTarget, 0.050f, 0.180f);

    comFwdOffset  = clamp(comFwdOffset, -bodyLength*0.3f, bodyLength*0.3f);
    comHeightZ    = clamp(comHeightZ,    0.010f, 0.050f);

    for (auto& s : sensors) {
        s.mountX = clamp(s.mountX, -bodyLength*0.5f, bodyLength*0.5f);
        s.mountY = clamp(s.mountY, -bodyWidth*0.5f,  bodyWidth*0.5f);
        s.angle  = clamp(s.angle, -HALF_PI, HALF_PI);
    }

    if (suction.enabled) {
        suction.fanDiameter = clamp(suction.fanDiameter, 0.020f, 0.060f);
        suction.fanPower    = clamp(suction.fanPower,    0.5f,   8.0f);
        suction.massKg      = clamp(suction.massKg,      0.005f, 0.050f);
    }
}

// ─────────────────────────────────────────────────────────────────────────
void RobotDesign::computeDerived()
{
    clampToLimits();

    wheelRadius = wheelDiameter * 0.5f;

    // ── Mass budget ────────────────────────────────────────────────────
    // Motor mass estimated from Kv and max current (empirical for micro-motors)
    motorMass   = 0.006f + motorMaxCurrent * 0.0012f;   // ~8-18g per motor
    batteryMass = batteryCapacity * 0.0000027f           // ~0.8 Wh/g for LiPo
                * batteryVoltage;
    float suctionM = suction.enabled ? suction.massKg : 0.f;
    totalMass = chassisMass + pcbMass + batteryMass
              + 2.f * motorMass          // two drive motors
              + 2.f * 0.004f             // two wheels (estimate 4g each)
              + suctionM;
    // Soft-clamp toward target
    totalMass = totalMass * 0.7f + totalMassTarget * 0.3f;
    totalMass = clamp(totalMass, 0.040f, 0.200f);

    // ── Moments of inertia — rectangular body approximation ────────────
    // Izz = (1/12)*m*(L²+W²) + point masses for battery/motor
    float Izz_body = (1.f/12.f) * totalMass
                   * (bodyLength*bodyLength + bodyWidth*bodyWidth);
    // Parallel axis for motors (two symmetric)
    float Izz_motors = 2.f * motorMass
                      * (motorFwdOffset*motorFwdOffset
                       + motorLateralPos*motorLateralPos);
    float Izz_bat = batteryMass * batteryFwdOffset * batteryFwdOffset;
    Izz = Izz_body + Izz_motors + Izz_bat;
    Izz = std::max(Izz, 1e-5f);

    // Ixx (roll), Iyy (pitch) – used for tilt dynamics
    Ixx = (1.f/12.f)*totalMass*(bodyWidth*bodyWidth  + bodyHeight*bodyHeight);
    Iyy = (1.f/12.f)*totalMass*(bodyLength*bodyLength + bodyHeight*bodyHeight);

    // ── Motor torque & speed ───────────────────────────────────────────
    // No-load speed at nominal voltage
    wheelMaxAngularV = (motorMaxRPM / 60.f) * TWO_PI * (1.f / gearRatio);
    robotMaxLinearV  = wheelMaxAngularV * wheelRadius;

    // Peak torque limited by current, reduced by back-EMF at speed
    float ktEff      = motorKt * gearRatio;
    wheelMaxTorque   = ktEff * motorMaxCurrent;   // Nm per wheel

    // ── Normal force & traction ────────────────────────────────────────
    float suctionForce = 0.f;
    if (suction.enabled) {
        // Simplified fan thrust: F ≈ sqrt(2 * rho * A * P)
        float A = PI * 0.25f * suction.fanDiameter * suction.fanDiameter;
        suction.thrustNewtons = std::sqrt(2.f * AIR_RHO * A * suction.fanPower);
        // Parasitic drag on body (rough estimate)
        suction.dragNewtons   = 0.05f * suction.fanPower;  // ~5% of power as drag
        suctionForce = suction.thrustNewtons;
    }
    normalForceStatic = totalMass * G_ACCEL + suctionForce;

    // Traction limit: F_trac = μ * Fn (shared between 2 driven wheels)
    float maxTractionForce = tireFrictionCoeff * normalForceStatic;
    float maxTractionAccel = maxTractionForce / totalMass;

    // Motor-limited acceleration: F_motor = 2 * T_wheel / r
    float maxMotorForce = 2.f * wheelMaxTorque / wheelRadius;
    float maxMotorAccel = maxMotorForce / totalMass;

    robotMaxAccel = std::min(maxTractionAccel, maxMotorAccel);

    // ── Aerodynamics ──────────────────────────────────────────────────
    // Frontal area with taper
    float taperH = bodyLength * 0.5f * std::tan(frontTaperAngle * PI / 180.f);
    float effectiveH = std::max(0.f, bodyHeight - taperH);
    float cd = 0.4f + (1.f - frontTaperAngle / 60.f) * 0.3f; // Cd ~0.4–0.7
    dragArea = cd * bodyWidth * effectiveH;

    // Downforce from suction (already captured in normalForce)
    downforceArea = suction.enabled
        ? suction.thrustNewtons / (0.5f * AIR_RHO * 5.f * 5.f) // effective at 5m/s
        : 0.f;

    // ── Fill algorithm RobotParams ─────────────────────────────────────
    // Braking adds both traction and motor braking
    float brakingAccel = std::min(maxTractionAccel * 1.1f, robotMaxAccel * 1.15f);

    algorithmParams.maxTotalAccel    = robotMaxAccel;
    algorithmParams.maxBraking       = brakingAccel;
    algorithmParams.maxAccelFwd      = robotMaxAccel * 0.90f; // safety margin
    algorithmParams.maxJerk          = robotMaxAccel * 8.f;   // empirical
    algorithmParams.maxVelocity      = std::min(robotMaxLinearV, 6.0f); // cap at 6m/s
    algorithmParams.exploreVelocity  = 0.5f;
    algorithmParams.wheelbase        = wheelbase;
    algorithmParams.trackWidth       = trackWidth;
    algorithmParams.cellSize         = 0.18f;
    // Steering bandwidth depends on wheel inertia and motor response
    algorithmParams.steeringBandwidth = 15.f + motorKv / 300.f;
    // PD gains scale with robot responsiveness
    algorithmParams.Kp_crosstrack    = 3.f + robotMaxAccel * 0.15f;
    algorithmParams.Kd_crosstrack    = 0.2f + trackWidth * 2.f;
    algorithmParams.Kp_heading       = 2.0f;
    algorithmParams.Kd_heading       = 0.1f;
    algorithmParams.Kp_center        = 3.0f;
    algorithmParams.Ki_center        = 0.1f;
}

// ─────────────────────────────────────────────────────────────────────────
RobotDesign RobotDesign::baseline()
{
    RobotDesign d;
    d.id   = 0;
    d.name = "GDW-v4.1-Baseline";
    // All values already set to championship-calibrated defaults
    d.computeDerived();
    return d;
}

// ─────────────────────────────────────────────────────────────────────────
RobotDesign RobotDesign::random(std::mt19937& rng, int id)
{
    RobotDesign d;
    d.id   = id;
    d.name = "Random-" + std::to_string(id);

    auto rf = [&](float lo, float hi) -> float {
        return lo + std::uniform_real_distribution<float>(0.f, 1.f)(rng) * (hi - lo);
    };

    d.bodyLength   = rf(0.055f, 0.110f);
    d.bodyWidth    = rf(0.055f, 0.088f);
    d.bodyHeight   = rf(0.022f, 0.065f);
    d.wheelbase    = rf(0.042f, d.bodyLength * 0.90f);
    d.trackWidth   = rf(0.042f, d.bodyWidth  * 0.95f);

    d.frontTaperAngle = rf(5.f, 55.f);
    d.rearTaperAngle  = rf(0.f, 40.f);

    d.wheelDiameter  = rf(0.022f, 0.040f);
    d.wheelWidth     = rf(0.006f, 0.013f);

    d.tireFrictionCoeff   = rf(0.70f, 1.90f);
    d.tireRollingResist   = rf(0.007f, 0.035f);
    d.tireDeformStiffness = rf(60.f,  350.f);

    d.motorKv        = rf(900.f,  2800.f);
    d.motorKt        = rf(0.003f, 0.018f);
    d.motorResistance= rf(0.6f,   4.5f);
    d.motorMaxCurrent= rf(1.2f,   7.0f);
    d.motorMaxRPM    = rf(5000.f, 22000.f);
    d.motorFwdOffset = rf(-0.035f, 0.015f);

    d.batteryVoltage  = rf(7.2f,  11.1f);
    d.batteryCapacity = rf(150.f,  800.f);

    d.chassisMass     = rf(0.010f, 0.050f);
    d.totalMassTarget = rf(0.055f, 0.160f);
    d.comFwdOffset    = rf(-0.015f, 0.015f);
    d.comHeightZ      = rf(0.012f, 0.045f);

    // Sensors: randomize angles slightly
    for (auto& s : d.sensors)
        s.angle = s.angle + rf(-0.3f, 0.3f);

    // 30% chance of suction system
    d.suction.enabled = (std::uniform_int_distribution<int>(0,9)(rng) < 3);
    if (d.suction.enabled) {
        d.suction.fanDiameter = rf(0.025f, 0.055f);
        d.suction.fanPower    = rf(0.8f,   6.0f);
    }

    d.computeDerived();
    return d;
}

// ─────────────────────────────────────────────────────────────────────────
//  Crossover: uniform blend with some single-point bias
// ─────────────────────────────────────────────────────────────────────────
RobotDesign RobotDesign::crossover(const RobotDesign& a, const RobotDesign& b,
                                    std::mt19937& rng)
{
    RobotDesign child = a;
    std::uniform_real_distribution<float> u(0.f, 1.f);
    auto pick = [&](float va, float vb) -> float {
        float alpha = u(rng);
        return alpha * va + (1.f - alpha) * vb;
    };

    child.bodyLength   = pick(a.bodyLength,   b.bodyLength);
    child.bodyWidth    = pick(a.bodyWidth,    b.bodyWidth);
    child.bodyHeight   = pick(a.bodyHeight,   b.bodyHeight);
    child.wheelbase    = pick(a.wheelbase,    b.wheelbase);
    child.trackWidth   = pick(a.trackWidth,   b.trackWidth);

    child.frontTaperAngle = pick(a.frontTaperAngle, b.frontTaperAngle);
    child.rearTaperAngle  = pick(a.rearTaperAngle,  b.rearTaperAngle);

    child.wheelDiameter   = pick(a.wheelDiameter,   b.wheelDiameter);
    child.wheelWidth      = pick(a.wheelWidth,       b.wheelWidth);

    child.tireFrictionCoeff   = pick(a.tireFrictionCoeff,   b.tireFrictionCoeff);
    child.tireRollingResist   = pick(a.tireRollingResist,   b.tireRollingResist);
    child.tireDeformStiffness = pick(a.tireDeformStiffness, b.tireDeformStiffness);

    child.motorKv         = pick(a.motorKv,         b.motorKv);
    child.motorKt         = pick(a.motorKt,         b.motorKt);
    child.motorResistance = pick(a.motorResistance, b.motorResistance);
    child.motorMaxCurrent = pick(a.motorMaxCurrent, b.motorMaxCurrent);
    child.motorMaxRPM     = pick(a.motorMaxRPM,     b.motorMaxRPM);
    child.motorFwdOffset  = pick(a.motorFwdOffset,  b.motorFwdOffset);

    child.batteryVoltage  = pick(a.batteryVoltage,  b.batteryVoltage);
    child.batteryCapacity = pick(a.batteryCapacity, b.batteryCapacity);

    child.chassisMass     = pick(a.chassisMass,     b.chassisMass);
    child.totalMassTarget = pick(a.totalMassTarget, b.totalMassTarget);
    child.comFwdOffset    = pick(a.comFwdOffset,    b.comFwdOffset);
    child.comHeightZ      = pick(a.comHeightZ,      b.comHeightZ);

    // Suction: inherit from parent with higher fitness (or 50/50)
    child.suction = (u(rng) > 0.5f) ? a.suction : b.suction;

    // Sensor angles: blend
    for (int i = 0; i < 4; i++)
        child.sensors[i].angle = pick(a.sensors[i].angle, b.sensors[i].angle);

    child.computeDerived();
    return child;
}

// ─────────────────────────────────────────────────────────────────────────
//  Mutation: random perturbation on each gene with probability `rate`
// ─────────────────────────────────────────────────────────────────────────
void RobotDesign::mutate(std::mt19937& rng, float rate)
{
    std::uniform_real_distribution<float> u(0.f, 1.f);
    std::normal_distribution<float> n(0.f, 1.f);

    auto maybe = [&](float& v, float sigma) {
        if (u(rng) < rate) v += n(rng) * sigma;
    };

    maybe(bodyLength,   0.006f);
    maybe(bodyWidth,    0.005f);
    maybe(bodyHeight,   0.004f);
    maybe(wheelbase,    0.005f);
    maybe(trackWidth,   0.004f);
    maybe(frontTaperAngle, 4.f);
    maybe(rearTaperAngle,  3.f);
    maybe(wheelDiameter,   0.002f);
    maybe(wheelWidth,      0.001f);
    maybe(tireFrictionCoeff,    0.08f);
    maybe(tireRollingResist,    0.002f);
    maybe(tireDeformStiffness,  20.f);
    maybe(motorKv,        80.f);
    maybe(motorKt,        0.001f);
    maybe(motorResistance,0.15f);
    maybe(motorMaxCurrent,0.3f);
    maybe(motorMaxRPM,    800.f);
    maybe(motorFwdOffset, 0.003f);
    maybe(batteryVoltage, 0.5f);
    maybe(batteryCapacity,30.f);
    maybe(chassisMass,    0.003f);
    maybe(totalMassTarget,0.008f);
    maybe(comFwdOffset,   0.003f);
    maybe(comHeightZ,     0.002f);

    for (auto& s : sensors)
        maybe(s.angle, 0.05f);

    // 5% chance to toggle suction
    if (u(rng) < 0.05f) {
        suction.enabled = !suction.enabled;
        if (suction.enabled) {
            suction.fanDiameter = 0.025f + u(rng) * 0.030f;
            suction.fanPower    = 1.0f   + u(rng) * 5.0f;
        }
    }
    if (suction.enabled) {
        maybe(suction.fanDiameter, 0.003f);
        maybe(suction.fanPower,    0.4f);
    }

    computeDerived();
}

// ─────────────────────────────────────────────────────────────────────────
std::string RobotDesign::toJSON() const
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(5);
    ss << "{\n";
    ss << "  \"id\": " << id << ",\n";
    ss << "  \"name\": \"" << name << "\",\n";
    ss << "  \"generation\": " << generation << ",\n";
    ss << "  \"fitnessScore\": " << fitnessScore << ",\n";
    ss << "  \"geometry\": {\n";
    ss << "    \"bodyLength_mm\": "    << bodyLength*1000.f  << ",\n";
    ss << "    \"bodyWidth_mm\": "     << bodyWidth*1000.f   << ",\n";
    ss << "    \"bodyHeight_mm\": "    << bodyHeight*1000.f  << ",\n";
    ss << "    \"wheelbase_mm\": "     << wheelbase*1000.f   << ",\n";
    ss << "    \"trackWidth_mm\": "    << trackWidth*1000.f  << ",\n";
    ss << "    \"wheelDiameter_mm\": " << wheelDiameter*1000.f << ",\n";
    ss << "    \"frontTaperDeg\": "    << frontTaperAngle    << ",\n";
    ss << "    \"rearTaperDeg\": "     << rearTaperAngle     << "\n";
    ss << "  },\n";
    ss << "  \"mass\": {\n";
    ss << "    \"totalMass_g\": "    << totalMass*1000.f    << ",\n";
    ss << "    \"chassisMass_g\": "  << chassisMass*1000.f  << ",\n";
    ss << "    \"batteryMass_g\": "  << batteryMass*1000.f  << ",\n";
    ss << "    \"motorMassEach_g\": "<< motorMass*1000.f    << "\n";
    ss << "  },\n";
    ss << "  \"dynamics\": {\n";
    ss << "    \"tireFriction\": "    << tireFrictionCoeff  << ",\n";
    ss << "    \"maxVelocity_ms\": "  << algorithmParams.maxVelocity << ",\n";
    ss << "    \"maxAccel_ms2\": "    << algorithmParams.maxTotalAccel << ",\n";
    ss << "    \"maxJerk_ms3\": "     << algorithmParams.maxJerk << "\n";
    ss << "  },\n";
    ss << "  \"inertia\": {\n";
    ss << "    \"Izz_kgm2\": " << Izz << ",\n";
    ss << "    \"comFwd_mm\": " << comFwdOffset*1000.f << ",\n";
    ss << "    \"comZ_mm\": "   << comHeightZ*1000.f   << "\n";
    ss << "  },\n";
    ss << "  \"suction\": {\n";
    ss << "    \"enabled\": " << (suction.enabled ? "true" : "false") << ",\n";
    ss << "    \"fanPower_W\": " << suction.fanPower << ",\n";
    ss << "    \"thrustN\": "    << suction.thrustNewtons << "\n";
    ss << "  }\n";
    ss << "}\n";
    return ss.str();
}

// ─────────────────────────────────────────────────────────────────────────
std::string RobotDesign::csvHeader()
{
    return "id,name,generation,fitness,"
           "bodyLength_mm,bodyWidth_mm,bodyHeight_mm,"
           "wheelbase_mm,trackWidth_mm,wheelDiam_mm,"
           "frontTaper_deg,rearTaper_deg,"
           "mass_g,tireFriction,motorKv,battVoltage,"
           "maxVel_ms,maxAccel_ms2,maxJerk_ms3,"
           "Izz_kgm2,comFwd_mm,comZ_mm,"
           "suctionEnabled,suctionPower_W\n";
}

std::string RobotDesign::toCSV() const
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(4);
    ss << id                           << ","
       << name                         << ","
       << generation                   << ","
       << fitnessScore                 << ","
       << bodyLength*1000.f            << ","
       << bodyWidth*1000.f             << ","
       << bodyHeight*1000.f            << ","
       << wheelbase*1000.f             << ","
       << trackWidth*1000.f            << ","
       << wheelDiameter*1000.f         << ","
       << frontTaperAngle              << ","
       << rearTaperAngle               << ","
       << totalMass*1000.f             << ","
       << tireFrictionCoeff            << ","
       << motorKv                      << ","
       << batteryVoltage               << ","
       << algorithmParams.maxVelocity  << ","
       << algorithmParams.maxTotalAccel<< ","
       << algorithmParams.maxJerk      << ","
       << Izz                          << ","
       << comFwdOffset*1000.f          << ","
       << comHeightZ*1000.f            << ","
       << (suction.enabled ? 1 : 0)    << ","
       << suction.fanPower             << "\n";
    return ss.str();
}
