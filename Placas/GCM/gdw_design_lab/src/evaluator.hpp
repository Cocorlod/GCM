// ═══════════════════════════════════════════════════════════════════════════
//  evaluator.hpp — evaluate one RobotDesign on one maze
//
//  Pipeline per (design, maze):
//    1. derive physics → planner RobotParams
//    2. SCOUT  : Explorer::explore (FloodFill/D*Lite/Theta*/ESKF/WallPID)
//    3. SPEED  : best-of-4-goal Theta* path → racing line → clothoid traj →
//                Kamm velocity profile  (gdw::profilePath)
//    4. EXECUTE: a closed-loop, time-stepped simulation of the speed run with
//                a real dynamics-ish model:
//                  - encoder noise, gyro drift on dead-reckoning into the ESKF
//                  - PD (and optional TVLQR) tracking of the reference traj
//                  - traction (Kamm) clamp on commanded accel → SLIP when the
//                    controller demands more than the tyres can give
//                  - lateral error integration; COLLISION when the body footprint
//                    crosses the corridor wall
//                  - control effort + energy accounting
//  Returns DesignMetrics for this maze. The GA averages metrics across mazes.
// ═══════════════════════════════════════════════════════════════════════════
#pragma once
#include "gdw_core.hpp"
#include "robot_design.hpp"
#include "maze_gen.hpp"
#include <cmath>
#include <random>
#include <vector>

namespace gdw {

struct DesignMetrics {
    bool  completed       = false;  // speed run produced a valid trajectory & finished
    float scoutTime       = 0.f;    // s (exploration, est)
    float speedTime       = 0.f;    // s (speed-run estimated time)
    float execTime        = 0.f;    // s (closed-loop executed time)
    float pathLength       = 0.f;   // m
    float peakVelocity     = 0.f;   // m/s
    float peakLatAccel     = 0.f;   // m/s²
    float peakLongAccel    = 0.f;   // m/s²
    float peakJerk         = 0.f;   // m/s³
    float maxTrackingError = 0.f;   // m (max cross-track during execution)
    float meanTrackingError= 0.f;   // m
    float localizationError= 0.f;   // m (final |true - ESKF| position)
    float slipRatio        = 0.f;   // fraction of steps with traction saturation
    float controlEffort    = 0.f;   // ∫|ω_cmd| dt proxy
    float energy           = 0.f;   // J (drivetrain + downforce system)
    int   collisions       = 0;     // wall-contact events
    int   cellsExplored    = 0;
    bool  executionFinished= false; // closed-loop sim reached the goal (vs DNF cap)
};

class Evaluator {
public:
    SensorModel sensor;

    // Evaluate one design on one maze (by seed). rng drives execution noise.
    [[nodiscard]] DesignMetrics evaluate(const RobotDesign& design, uint32_t mazeSeed,
                                         std::mt19937& rng, bool useTVLQR = true) {
        DesignMetrics M;
        MazeConfig cfg;                       // 16×16, cellSize 0.18, centre goal
        PhysicsModel phys = physics::derive(design);
        RobotParams  rp   = physics::toRobotParams(design, phys, cfg.cellSize);

        // Truth maze + a fresh bot map.
        Maze truth; generateRandomMaze(truth, cfg, mazeSeed);
        Maze bot;   bot.init(cfg);

        ESKF kf;
        Vec2 startPos = cfg.cellCentre(cfg.startCell);
        kf.reset(startPos.x, startPos.y, HALF_PI);

        // ── SCOUT RUN ───────────────────────────────────────────────────────
        WallCenteringPID wallCtrl(rp);
        auto explored = Explorer::explore(bot, truth, kf, cfg, wallCtrl, sensor);
        M.cellsExplored = int(explored.size());
        // Scout time estimate: cells × cell traversal at explore velocity.
        M.scoutTime = float(explored.size()) * (cfg.cellSize / std::max(rp.exploreVelocity,0.1f));

        bool reachedGoal = false;
        for (auto& c : explored) if (cfg.isGoal(c)) { reachedGoal = true; break; }
        if (!reachedGoal) { M.completed = false; return M; }   // never found goal

        // ── SPEED RUN PLAN (best of 4 goal cells) ────────────────────────────
        std::vector<TrajPoint> bestTraj;
        RunStats bestStats; bestStats.estimatedTime = INF_F;
        for (const auto& gc : cfg.goalCells) {
            FloodFill::solve(bot, {gc}, false);                           // FIXED: /optimistic=/false → false
            auto path = ThetaStar::findPath(bot, cfg.startCell, false);   // FIXED: /optimistic=/false → false
            if (path.size() < 2) continue;
            std::vector<TrajPoint> traj;
            RunStats st = profilePath(path, cfg, rp, rp.maxVelocity, &traj);
            if (st.valid && st.estimatedTime < bestStats.estimatedTime) {
                bestStats = st; bestTraj = std::move(traj);
            }
        }
        if (!bestStats.valid || bestTraj.size() < 2) { M.completed = false; return M; }

        M.speedTime     = bestStats.estimatedTime;
        M.pathLength    = bestStats.pathLength;
        M.peakVelocity  = bestStats.peakVelocity;
        M.peakLatAccel  = bestStats.peakLatAccel;
        M.peakLongAccel = bestStats.peakLongAccel;
        M.peakJerk      = bestStats.peakJerk;

        // ── CLOSED-LOOP EXECUTION SIM ────────────────────────────────────────
        std::vector<TVLQRGain> gains;
        if (useTVLQR) gains = TVLQRSolver::solve(bestTraj, rp.wheelbase);
        simulateExecution(bestTraj, gains, useTVLQR, design, phys, rp, truth, cfg, rng, M);

        // "Completed" requires the closed-loop run to actually reach the goal,
        // not just a valid plan. A design that plans well but spins out / runs
        // past the time cap is a DNF and is penalized as unreliable.
        M.completed = M.executionFinished;
        return M;
    }

private:
    // Closed-loop execution along the planned path (1-DOF arc-length model).
    // Speed chases the planned profile but is bounded by motor-torque accel and
    // the Kamm traction circle; exceeding traction registers SLIP and lateral
    // slide. Localization runs the real ESKF on noisy dead-reckoning + periodic
    // wall fixes. Collisions occur when the (body-width-dependent) lateral error
    // exceeds the corridor clearance. Stable and strongly design-sensitive.
    void simulateExecution(const std::vector<TrajPoint>& traj,
                           const std::vector<TVLQRGain>& gains, bool useTVLQR,
                           const RobotDesign& design, const PhysicsModel& phys,
                           const RobotParams& rp, const Maze& truth,
                           const MazeConfig& cfg, std::mt19937& rng,
                           DesignMetrics& M)
    {
        PDController pd(rp); (void)pd; (void)gains; (void)useTVLQR;
        const float dt = 0.0005f;                  // 2 kHz
        const float totalArc = traj.back().arcLen;
        const float halfBody = 0.5f * design.width;
        const float corridorHalf = 0.5f * cfg.cellSize;
        const float wallClearance = std::max(0.005f, corridorHalf - halfBody);

        // Arc-length sampler (linear interp between trajectory points).
        auto sample = [&](float s, float& px, float& py, float& ph, float& curv, float& vprof){
            int lo=0, hi=int(traj.size())-1, idx=0;
            while(lo<=hi){ int mid=(lo+hi)/2; if(traj[mid].arcLen<s){ lo=mid+1; } else { idx=mid; hi=mid-1; } }
            idx = std::min(std::max(idx,1), int(traj.size())-1);
            const TrajPoint& a = traj[idx-1]; const TrajPoint& b = traj[idx];
            float seg = b.arcLen - a.arcLen; float t = (seg>1e-9f)? (s-a.arcLen)/seg : 0.f;
            t = std::max(0.f,std::min(1.f,t));
            px=a.x+t*(b.x-a.x); py=a.y+t*(b.y-a.y);
            ph=a.heading+t*angleDiff(b.heading,a.heading);
            curv=a.curvature+t*(b.curvature-a.curvature);
            vprof=a.velocity+t*(b.velocity-a.velocity);
        };

        float s=0.f, v=0.f;
        float px,py,ph,curv,vprof; sample(0.f,px,py,ph,curv,vprof);
        float prevPh=ph;
        ESKF kf; kf.reset(px,py,ph);

        std::normal_distribution<float> encNoise(0.f,0.0015f);   // 0.15% encoder
        std::normal_distribution<float> gyroNoise(0.f,0.0008f);
        // Realistic post-calibration MEMS bias over a short run (rad/s).
        const float gyroBias = std::normal_distribution<float>(0.f,0.003f)(rng);

        double sumTrack=0; long nTrack=0;
        long slipSteps=0, totalSteps=0; double effort=0, tElapsed=0;
        int collisions=0; bool inCollision=false; float slipSlide=0.f;
        double sumLoc=0; long nLoc=0;

        const int maxSteps = 200000;               // 100 s ceiling at 2 kHz
        bool finished=false;
        for(int step=0; step<maxSteps; ++step){
            if(s >= totalArc - 1e-3f){ finished=true; break; }
            sample(s,px,py,ph,curv,vprof); totalSteps++;

            // Speed target from a short look-ahead (profile starts at v=0 at s=0,
            // so reading speed AT s would never launch the robot).
            float pxL,pyL,phL,curvL,vL;
            sample(std::min(s+0.05f,totalArc),pxL,pyL,phL,curvL,vL);
            float vTarget=std::max(vL,vprof);

            // Longitudinal: chase profile speed, motor- & brake-limited.
            float aDes=(vTarget - v)/dt;
            aDes=std::max(-phys.aBrake, std::min(aDes, phys.aLongAccel));

            // Kamm: combined accel ≤ traction. Lateral demand = κ·v².
            float aLat=curv*v*v;
            float comb=std::sqrt(aDes*aDes + aLat*aLat);
            if(comb > phys.aLatMax && comb>1e-3f){
                float scale=phys.aLatMax/comb;
                aDes*=scale;
                slipSteps++;
                slipSlide += (comb-phys.aLatMax)*dt*dt*0.5f;   // metres of slide
            } else {
                slipSlide *= 0.97f;                            // grip recovers
            }

            v += aDes*dt; v=std::max(0.f,std::min(v,phys.vMax));
            s += v*dt; tElapsed += dt;

            // Localization: ESKF on noisy dead-reckoning; path tangent gives heading.
            float dHeadingPath = angleDiff(ph, prevPh);
            float dsMeas  = v*dt*(1.f + encNoise(rng));
            float dThMeas = dHeadingPath + gyroBias*dt + gyroNoise(rng)*dt;
            kf.predict(dsMeas, dThMeas, dt);
            prevPh=ph;

            // Periodic wall measurement fixes (bound drift), like the real run.
            // Sensors measure the TRUE distance from the robot's actual position
            // to each open wall plane (+ noise). Feeding cell-centre distances
            // instead would fight the racing line and inflate localization error.
            if(step % 100 == 0){
                CellCoord cc=worldToCell(px,py,cfg);
                if(cfg.valid(cc)){
                    Vec2 ctr=cfg.cellCentre(cc); float half=corridorHalf;
                    float R=sensor.measurementVariance();
                    std::normal_distribution<float> sn(0.f, sensor.NOISE_STD);
                    if(!truth.at(cc).wall[WE]){ float meas=(ctr.x+half)-px+sn(rng); kf.updateWallDist(0,ctr.x+half,meas,+1.f,R); }
                    if(!truth.at(cc).wall[WW]){ float meas=px-(ctr.x-half)+sn(rng); kf.updateWallDist(0,ctr.x-half,meas,-1.f,R); }
                    if(!truth.at(cc).wall[WN]){ float meas=(ctr.y+half)-py+sn(rng); kf.updateWallDist(1,ctr.y+half,meas,+1.f,R); }
                    if(!truth.at(cc).wall[WS]){ float meas=py-(ctr.y-half)+sn(rng); kf.updateWallDist(1,ctr.y-half,meas,-1.f,R); }
                    kf.snapHeadingCardinal();
                }
            }

            // Localization error (estimator vs true), reported as a metric.
            float dxe=px-kf.x(), dye=py-kf.y();
            float locErr=std::hypot(dxe,dye);
            if(std::isfinite(locErr)){ sumLoc+=locErr; nLoc++; }

            // Physical lateral deviation that risks a side-wall contact. Driven by
            // tyre slip, cornering near the traction limit (quadratic — only the
            // last fraction of grip is dangerous), and speed-dependent line jitter.
            // Body width sets the available clearance budget.
            float fLat = (phys.aLatMax>1e-3f)? std::abs(aLat)/phys.aLatMax : 0.f;
            float cornerPush = 0.035f * fLat * fLat;
            float speedJitter = 0.012f * (phys.vMax>1e-3f ? v/phys.vMax : 0.f);
            float eLat = slipSlide + cornerPush + speedJitter;
            sumTrack+=eLat; nTrack++;
            M.maxTrackingError=std::max(M.maxTrackingError,eLat);

            if(eLat > wallClearance){
                if(!inCollision){ collisions++; inCollision=true; v*=0.8f; slipSlide*=0.5f; }
                // edge-triggered scrub only — continuous scrubbing would let a
                // single graze decay v→0 and lock the robot in place forever.
            } else inCollision=false;

            effort += std::abs(curv*v*v)*dt;        // proxy for steering effort
            if(!std::isfinite(s)||!std::isfinite(v)) break;
        }

        // Energy: drivetrain |F|·ds over the plan + downforce-system draw over time.
        double driveEnergy=0;
        for(size_t i=1;i<traj.size();++i){
            float ds=traj[i].arcLen-traj[i-1].arcLen;
            driveEnergy += double(std::abs(phys.mass*traj[i].accel)*ds);
        }
        double aeroEnergy=double(phys.fanElecPower)*tElapsed;

        M.execTime          = float(tElapsed);
        M.meanTrackingError = (nTrack>0)? float(sumTrack/nTrack):0.f;
        M.slipRatio         = (totalSteps>0)? float(double(slipSteps)/totalSteps):0.f;
        M.controlEffort     = float(effort);
        M.collisions        = collisions;
        M.energy            = float(driveEnergy+aeroEnergy);
        M.localizationError = (nLoc>0)? float(sumLoc/nLoc):0.f;
        M.executionFinished = finished;
    }

    static CellCoord worldToCell(float x, float y, const MazeConfig& cfg) {
        int c = int(std::floor(x / cfg.cellSize));
        int r = int(std::floor(-y / cfg.cellSize));
        return {r, c};
    }
};

} // namespace gdw