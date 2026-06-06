// ═══════════════════════════════════════════════════════════════════════════
//  export.hpp — output of the discovered optimal design
//
//  writeJSON / writeCSV / writeSpec : the optimal RobotDesign + derived physics
//  + expected performance. CADExporter is the "future-ready" interface the spec
//  asks for: a base class with a concrete stub that emits a neutral parametric
//  description; STEP/Fusion/SolidWorks/FreeCAD back-ends slot in behind it.
// ═══════════════════════════════════════════════════════════════════════════
#pragma once
#include "robot_design.hpp"
#include "genetic.hpp"
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

namespace gdw {

inline std::string mm(float metres) {
    std::ostringstream os; os << std::fixed << std::setprecision(2) << metres*1000.f; return os.str();
}

inline void writeJSON(const std::string& path, const Individual& best) {
    const RobotDesign& d = best.design;
    PhysicsModel p = physics::derive(d);
    std::ofstream f(path);
    f << std::fixed << std::setprecision(4);
    f << "{\n";
    f << "  \"design\": {\n";
    f << "    \"length_mm\": "        << d.length*1000   << ",\n";
    f << "    \"width_mm\": "         << d.width*1000    << ",\n";
    f << "    \"height_mm\": "        << d.height*1000   << ",\n";
    f << "    \"wheelbase_mm\": "     << d.wheelbase*1000<< ",\n";
    f << "    \"track_width_mm\": "   << d.trackWidth*1000<<",\n";
    f << "    \"wheel_diameter_mm\": "<< d.wheelDiameter*1000<<",\n";
    f << "    \"wheel_width_mm\": "   << d.wheelWidth*1000<<",\n";
    f << "    \"tire_friction\": "    << d.tireFriction  << ",\n";
    f << "    \"motor_stall_torque_Nm\": " << d.motorStallTorque << ",\n";
    f << "    \"motor_free_speed_rad_s\": " << d.motorFreeSpeed << ",\n";
    f << "    \"cells\": "            << d.cellCount     << ",\n";
    f << "    \"batt_internal_R_ohm\": " << d.battInternalR << ",\n";
    f << "    \"has_fan\": "          << (d.hasFan?"true":"false") << ",\n";
    f << "    \"fan_power_W\": "      << d.fanPower_W    << ",\n";
    f << "    \"has_suction\": "      << (d.hasSuction?"true":"false") << ",\n";
    f << "    \"suction_pressure_Pa\": " << d.suctionPressure << ",\n";
    f << "    \"mass_total_kg\": "    << d.totalMass()   << "\n";
    f << "  },\n";
    f << "  \"physics\": {\n";
    f << "    \"mass_kg\": "          << p.mass          << ",\n";
    f << "    \"yaw_inertia_kgm2\": " << p.yawInertia    << ",\n";
    f << "    \"com_x_mm\": "         << p.comX*1000     << ",\n";
    f << "    \"downforce_N\": "      << p.downforce     << ",\n";
    f << "    \"a_lat_max_ms2\": "    << p.aLatMax       << ",\n";
    f << "    \"a_long_accel_ms2\": " << p.aLongAccel    << ",\n";
    f << "    \"v_max_ms\": "         << p.vMax          << ",\n";
    f << "    \"traction_margin\": "  << p.tractionMargin<< "\n";
    f << "  },\n";
    f << "  \"performance\": {\n";
    f << "    \"fitness\": "          << best.fitness        << ",\n";
    f << "    \"mean_speed_time_s\": "<< best.meanSpeedTime  << ",\n";
    f << "    \"mean_exec_time_s\": " << best.meanExecTime   << ",\n";
    f << "    \"mean_peak_velocity_ms\": " << best.meanPeakV << ",\n";
    f << "    \"mean_slip_ratio\": "  << best.meanSlip       << ",\n";
    f << "    \"mean_tracking_error_m\": " << best.meanTrack << ",\n";
    f << "    \"mean_localization_error_m\": " << best.meanLoc << ",\n";
    f << "    \"mean_energy_J\": "    << best.meanEnergy     << ",\n";
    f << "    \"mean_collisions\": " << best.meanCollisions << ",\n";
    f << "    \"reliability\": "      << best.reliability    << "\n";
    f << "  }\n";
    f << "}\n";
}

inline void writeCSV(const std::string& path, const std::vector<Individual>& hallOfFame) {
    std::ofstream f(path);
    f << "rank,fitness,length_mm,width_mm,wheel_dia_mm,track_mm,mass_g,mu,"
         "has_fan,fan_W,a_lat_max,v_max,mean_exec_time_s,mean_slip,mean_collisions,reliability\n";
    f << std::fixed << std::setprecision(4);
    int rank=1;
    for (const auto& ind : hallOfFame) {
        const RobotDesign& d = ind.design; PhysicsModel p = physics::derive(d);
        f << rank++ << "," << ind.fitness << ","
          << d.length*1000 << "," << d.width*1000 << "," << d.wheelDiameter*1000 << ","
          << d.trackWidth*1000 << "," << d.totalMass()*1000 << "," << d.tireFriction << ","
          << (d.hasFan?1:0) << "," << d.fanPower_W << ","
          << p.aLatMax << "," << p.vMax << ","
          << ind.meanExecTime << "," << ind.meanSlip << "," << ind.meanCollisions << ","
          << ind.reliability << "\n";
    }
}

inline std::string specReport(const Individual& best) {
    const RobotDesign& d = best.design; PhysicsModel p = physics::derive(d);
    std::ostringstream o; o << std::fixed << std::setprecision(2);
    o << "OPTIMAL MICROMOUSE DESIGN  (GDW v4.1)\n";
    o << "=====================================\n";
    o << "Geometry\n";
    o << "  Length            : " << mm(d.length) << " mm\n";
    o << "  Width             : " << mm(d.width)  << " mm\n";
    o << "  Height            : " << mm(d.height) << " mm\n";
    o << "  Wheelbase         : " << mm(d.wheelbase) << " mm\n";
    o << "  Track width       : " << mm(d.trackWidth) << " mm\n";
    o << "  Wheel diameter    : " << mm(d.wheelDiameter) << " mm\n";
    o << "  Wheel width       : " << mm(d.wheelWidth) << " mm\n";
    o << "Placements (body frame, mm)\n";
    o << "  Motors            : (" << mm(d.motorX) << ", +/-" << mm(d.motorY) << ")\n";
    o << "  Battery           : (" << mm(d.batteryX) << ", " << mm(d.batteryY) << ")\n";
    o << "  PCB               : (" << mm(d.pcbX) << ", " << mm(d.pcbY) << ")\n";
    o << "  Sensor 0 (L side) : (" << mm(d.sensorX[0]) << ", " << mm(d.sensorY[0]) << ")\n";
    o << "  Sensor 4 (front)  : (" << mm(d.sensorX[4]) << ", " << mm(d.sensorY[4]) << ")\n";
    o << "Mass / inertia\n";
    o << "  Total mass        : " << d.totalMass()*1000 << " g\n";
    o << "  COM x             : " << p.comX*1000 << " mm\n";
    o << "  Yaw inertia       : " << p.yawInertia*1e6 << " g.cm^2\n";
    o << "Tyres / drivetrain\n";
    o << "  Tyre friction mu  : " << d.tireFriction << "\n";
    o << "  Motor stall torque: " << d.motorStallTorque*1000 << " mN.m\n";
    o << "  Motor free speed  : " << d.motorFreeSpeed << " rad/s\n";
    o << "Aero\n";
    o << "  Fan               : " << (d.hasFan?"YES":"no") << "  power " << d.fanPower_W << " W\n";
    o << "  Suction           : " << (d.hasSuction?"YES":"no") << "  dP " << d.suctionPressure << " Pa\n";
    o << "  Downforce         : " << p.downforce << " N\n";
    o << "Derived limits\n";
    o << "  a_lat_max (Kamm)  : " << p.aLatMax << " m/s^2  (" << p.aLatMax/GRAV << " g)\n";
    o << "  a_long accel      : " << p.aLongAccel << " m/s^2\n";
    o << "  v_max             : " << p.vMax << " m/s\n";
    o << "  traction margin   : " << p.tractionMargin << "x\n";
    o << "Expected performance (avg over maze set)\n";
    o << "  Speed-run time    : " << best.meanSpeedTime << " s (planned)\n";
    o << "  Executed time     : " << best.meanExecTime  << " s (closed-loop)\n";
    o << "  Peak velocity     : " << best.meanPeakV     << " m/s\n";
    o << "  Mean slip ratio   : " << best.meanSlip      << "\n";
    o << "  Mean track error  : " << best.meanTrack*1000<< " mm\n";
    o << "  Localization err  : " << best.meanLoc*1000  << " mm\n";
    o << "  Mean collisions   : " << best.meanCollisions<< "\n";
    o << "  Energy / run      : " << best.meanEnergy    << " J\n";
    o << "  Reliability       : " << best.reliability*100 << " %\n";
    return o.str();
}

inline void writeSpec(const std::string& path, const Individual& best) {
    std::ofstream f(path); f << specReport(best);
}

// ── Future-ready CAD export interface ──────────────────────────────────────
class CADExporter {
public:
    virtual ~CADExporter() = default;
    virtual std::string format() const = 0;
    virtual bool exportDesign(const RobotDesign& d, const std::string& path) = 0;
};

// Neutral parametric description (works today; STEP/Fusion/SW/FreeCAD plug in
// the same way behind this interface).
class ParametricCADExporter : public CADExporter {
public:
    std::string format() const override { return "parametric-txt"; }
    bool exportDesign(const RobotDesign& d, const std::string& path) override {
        std::ofstream f(path); if (!f) return false;
        f << std::fixed << std::setprecision(4);
        f << "# GDW Design Lab — neutral parametric CAD description\n";
        f << "# units: mm\n";
        f << "PARAM length "  << d.length*1000 << "\n";
        f << "PARAM width "   << d.width*1000  << "\n";
        f << "PARAM height "  << d.height*1000 << "\n";
        f << "PARAM nose_taper_frac " << d.noseTaper << "\n";
        f << "PARAM front_taper_deg " << d.frontTaperAng*180/PI << "\n";
        f << "PARAM rear_taper_deg "  << d.rearTaperAng*180/PI  << "\n";
        f << "PARAM wheel_diameter " << d.wheelDiameter*1000 << "\n";
        f << "PARAM wheel_width "    << d.wheelWidth*1000    << "\n";
        f << "PARAM track_width "    << d.trackWidth*1000    << "\n";
        f << "PARAM wheelbase "      << d.wheelbase*1000     << "\n";
        for (int i=0;i<5;i++)
            f << "SENSOR " << i << " " << d.sensorX[i]*1000 << " "
              << d.sensorY[i]*1000 << " " << d.sensorAng[i]*180/PI << "\n";
        f << "PLACE motor "   << d.motorX*1000   << " " << d.motorY*1000   << "\n";
        f << "PLACE battery " << d.batteryX*1000 << " " << d.batteryY*1000 << "\n";
        f << "PLACE pcb "     << d.pcbX*1000     << " " << d.pcbY*1000     << "\n";
        f << "# Back-ends: emit STEP AP242 / Fusion 360 API / SolidWorks API /\n";
        f << "#            FreeCAD Python from these parameters.\n";
        return true;
    }
};

// Stub back-ends so the interface is concretely "future-ready".
class StepCADExporter : public CADExporter {
public:
    std::string format() const override { return "STEP-AP242 (stub)"; }
    bool exportDesign(const RobotDesign&, const std::string& path) override {
        std::ofstream f(path);
        f << "ISO-10303-21;\n/* GDW Design Lab STEP back-end placeholder. */\nEND-ISO-10303-21;\n";
        return true;
    }
};

inline std::unique_ptr<CADExporter> makeCADExporter(const std::string& fmt) {
    if (fmt == "step") return std::make_unique<StepCADExporter>();
    return std::make_unique<ParametricCADExporter>();
}

} // namespace gdw