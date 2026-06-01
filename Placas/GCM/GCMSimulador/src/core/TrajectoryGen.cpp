// ═══════════════════════════════════════════════════════════════════════════
//  src/core/TrajectoryGen.cpp
// ═══════════════════════════════════════════════════════════════════════════
#include "TrajectoryGen.h"
#include "Types.h"
#include <algorithm>
#include <cmath>
#include <numeric>

// ══════════════════════════════════════════════════════════════════════════
//  ClothoidSeg
// ══════════════════════════════════════════════════════════════════════════
ClothoidSeg::State ClothoidSeg::eval(float s) const noexcept
{
    if (s <= 0.f) return {x0, y0, theta0, kappa0};
    float dkds = (length > 1e-9f) ? (kappaEnd - kappa0) / length : 0.f;

    auto thetaAt = [&](float t) noexcept {
        return theta0 + kappa0*t + 0.5f*dkds*t*t;
    };

    float mid = s * 0.5f, hr = s * 0.5f;
    float px = x0, py = y0;
    for (int i=0; i<8; i++) {
        float t1 = mid + hr*GL_XI[i];
        float t2 = mid - hr*GL_XI[i];
        px += GL_W[i]*hr*(std::cos(thetaAt(t1)) + std::cos(thetaAt(t2)));
        py += GL_W[i]*hr*(std::sin(thetaAt(t1)) + std::sin(thetaAt(t2)));
    }
    return {px, py, thetaAt(s), kappa0 + dkds*s};
}

// ══════════════════════════════════════════════════════════════════════════
//  PathUtils
// ══════════════════════════════════════════════════════════════════════════
std::vector<Waypoint> PathUtils::pathToWaypoints(
    const std::vector<CellCoord>& path, const MazeConfig& cfg)
{
    std::vector<Waypoint> wps;
    int N = int(path.size());
    for (int i=0; i<N; i++) {
        Vec2 pos = cfg.cellCentre(path[i]);
        float hdg = wps.empty() ? 0.f : wps.back().heading;
        if (i+1 < N) {
            Vec2 nxt = cfg.cellCentre(path[i+1]);
            hdg = std::atan2(nxt.y - pos.y, nxt.x - pos.x);
        }
        wps.push_back({pos.x, pos.y, hdg});
    }
    if (wps.size() >= 2) wps.back().heading = wps[wps.size()-2].heading;
    return wps;
}

// ══════════════════════════════════════════════════════════════════════════
//  RacingLineOptimiser
// ══════════════════════════════════════════════════════════════════════════
std::vector<Waypoint> RacingLineOptimiser::optimise(
    std::vector<Waypoint> wps,
    float halfWidth,
    float margin,
    int   maxIter)
{
    int N = int(wps.size());
    if (N < 3) return wps;
    float hw = halfWidth - margin;
    if (hw <= 0.f) return wps;

    std::vector<Waypoint> centres = wps;
    float step = hw * 0.15f;

    for (int iter=0; iter<maxIter; iter++) {
        float totalChange = 0.f;
        for (int i=1; i<N-1; i++) {
            // Second-difference gradient (curvature minimisation)
            float gx=0.f, gy=0.f;
            if (i > 1) {
                gx += wps[i-2].x - 2.f*wps[i-1].x + wps[i].x;
                gy += wps[i-2].y - 2.f*wps[i-1].y + wps[i].y;
            }
            gx += wps[i-1].x - 2.f*wps[i].x + wps[i+1].x;
            gy += wps[i-1].y - 2.f*wps[i].y + wps[i+1].y;
            if (i < N-2) {
                gx += wps[i].x - 2.f*wps[i+1].x + wps[i+2].x;
                gy += wps[i].y - 2.f*wps[i+1].y + wps[i+2].y;
            }

            float nx = wps[i].x - step * gx * 2.f;
            float ny = wps[i].y - step * gy * 2.f;
            // Clamp to corridor
            nx = clamp(nx, centres[i].x - hw, centres[i].x + hw);
            ny = clamp(ny, centres[i].y - hw, centres[i].y + hw);
            totalChange += std::abs(nx-wps[i].x) + std::abs(ny-wps[i].y);
            wps[i].x = nx;
            wps[i].y = ny;
        }
        // Recompute headings
        for (int i=0; i<N-1; i++) {
            float dx=wps[i+1].x-wps[i].x, dy=wps[i+1].y-wps[i].y;
            wps[i].heading = std::atan2(dy, dx);
        }
        if (N >= 2) wps[N-1].heading = wps[N-2].heading;
        // Adaptive step size
        if (iter % 15 == 14) step *= 0.7f;
        if (totalChange < 1e-7f) break;
    }
    return wps;
}

// ══════════════════════════════════════════════════════════════════════════
//  TrajGen — clothoid-arc-clothoid
//  FIX-C: arc direction, FIX-D: clothoid length lower bound
// ══════════════════════════════════════════════════════════════════════════
std::vector<TrajPoint> TrajGen::generate(
    const std::vector<Waypoint>& wps, const RobotParams& robot)
{
    std::vector<TrajPoint> traj;
    int N = int(wps.size());
    if (N < 2) return traj;

    float cumArc = 0.f;
    float kPrev  = 0.f;

    auto emit = [&](float x, float y, float hdg, float k) {
        float arc = 0.f;
        if (!traj.empty()) {
            float dx=x-traj.back().x, dy=y-traj.back().y;
            arc = std::sqrt(dx*dx+dy*dy);
        }
        cumArc += arc;
        traj.push_back({x, y, hdg, k, robot.maxVelocity, cumArc, 0,0,0});
    };

    for (int wi=0; wi+1<N; wi++) {
        const Waypoint& wa = wps[wi];
        const Waypoint& wb = wps[wi+1];
        float dhdg   = angleDiff(wb.heading, wa.heading);
        float segLen = std::hypot(wb.x-wa.x, wb.y-wa.y);
        if (segLen < 1e-7f) continue;
        int startIdx = (wi==0) ? 0 : 1;

        if (std::abs(dhdg) < 5e-3f) {
            // Straight segment
            for (int i=startIdx; i<=SAMPLES_STRAIGHT; i++) {
                float t = float(i)/float(SAMPLES_STRAIGHT);
                emit(wa.x+t*(wb.x-wa.x), wa.y+t*(wb.y-wa.y), wa.heading, 0.f);
            }
            kPrev = 0.f;
        } else {
            // Clothoid – Arc – Clothoid
            float R = segLen / (2.f * std::sin(std::abs(dhdg)*0.5f));
            R = std::max(R, robot.cellSize * 0.05f);
            float kTurn = (1.f/R) * (dhdg>0.f ? 1.f : -1.f);

            // FIX-D: lower bound enforced before capping
            float dkappa  = std::abs(kTurn - kPrev);
            float L_c_min = dkappa * robot.maxVelocity / robot.steeringBandwidth;
            float L_c_max = segLen * 0.45f;
            float L_c     = std::max(L_c_min, 0.005f);  // FIX-D: max before cap
            L_c           = std::min(L_c, L_c_max);

            // Entry clothoid
            ClothoidSeg entry{wa.x, wa.y, wa.heading, kPrev, kTurn, L_c};
            for (int i=startIdx; i<=SAMPLES_CLOTHOID; i++) {
                float s = float(i)/float(SAMPLES_CLOTHOID)*L_c;
                auto st = entry.eval(s);
                emit(st.x, st.y, st.theta, st.kappa);
            }

            // Arc (FIX-C: CCW = angle increases)
            auto eEnd = entry.eval(L_c);
            float sign = (dhdg > 0.f) ? 1.f : -1.f;
            float perpAngle = eEnd.theta + sign * HALF_PI;
            float cx = eEnd.x + R * std::cos(perpAngle);
            float cy = eEnd.y + R * std::sin(perpAngle);

            float clothoidAngle = L_c / R;
            float arcAngle = std::abs(dhdg) - 2.f * clothoidAngle;

            if (arcAngle > 1e-4f) {
                float startArc = std::atan2(eEnd.y-cy, eEnd.x-cx);
                for (int i=1; i<=SAMPLES_ARC; i++) {
                    float t = float(i)/float(SAMPLES_ARC);
                    // FIX-C: ang = startArc + sign*t*arcAngle
                    float ang = startArc + sign * t * arcAngle;
                    float px  = cx + R * std::cos(ang);
                    float py  = cy + R * std::sin(ang);
                    float hdg = wrapAngle(ang + sign * HALF_PI);
                    emit(px, py, hdg, kTurn);
                }
            }

            // Exit clothoid
            const TrajPoint& ae = traj.back();
            ClothoidSeg exitSeg{ae.x, ae.y, ae.heading, kTurn, 0.f, L_c};
            for (int i=1; i<=SAMPLES_CLOTHOID; i++) {
                float s = float(i)/float(SAMPLES_CLOTHOID)*L_c;
                auto st = exitSeg.eval(s);
                emit(st.x, st.y, st.theta, st.kappa);
            }
            kPrev = 0.f;
        }
    }
    return traj;
}

// ══════════════════════════════════════════════════════════════════════════
//  VelocityProfile
//  FIX-F: prevBrk initialised once outside convergence loop
// ══════════════════════════════════════════════════════════════════════════
float VelocityProfile::kammLong(float kappa, float v, float aTotal) noexcept
{
    float aLat2 = (kappa*v*v)*(kappa*v*v);
    float aT2   = aTotal*aTotal;
    return (aLat2 >= aT2) ? 0.f : std::sqrt(aT2 - aLat2);
}

float VelocityProfile::vMaxCurv(float kappa, float aTotal) noexcept
{
    return (std::abs(kappa) < 1e-7f) ? INF_F : std::sqrt(aTotal/std::abs(kappa));
}

void VelocityProfile::curvatureCeilings(std::vector<TrajPoint>& traj,
                                          float vMax, float aTotal)
{
    for (auto& tp : traj)
        tp.velocity = std::min(vMax, vMaxCurv(tp.curvature, aTotal));
}

void VelocityProfile::globalBrakingPass(std::vector<TrajPoint>& traj,
                                          float aTotal, float aBrakeMax)
{
    int N = int(traj.size());
    if (N < 2) return;
    for (int pass=0; pass<3; pass++) {
        for (int i=N-2; i>=0; i--) {
            float ds = traj[i+1].arcLen - traj[i].arcLen;
            if (ds < 1e-9f) continue;
            float v1   = traj[i+1].velocity;
            float aBrk = std::min(kammLong(traj[i].curvature, traj[i].velocity, aTotal),
                                   aBrakeMax);
            float vMax = std::sqrt(v1*v1 + 2.f*aBrk*ds);
            if (vMax < traj[i].velocity) traj[i].velocity = vMax;
        }
    }
}

void VelocityProfile::backwardPass(std::vector<TrajPoint>& traj,
                                     float maxJerk, float aTotal, float aBrakeMax,
                                     int maxIter)
{
    int N = int(traj.size());
    if (N < 2) return;
    traj.back().velocity = 0.f;

    for (int iter=0; iter<maxIter; iter++) {
        float maxChange = 0.f;
        float prevBrk   = 0.f;  // FIX-F: single init per sweep

        for (int i=N-2; i>=0; i--) {
            float ds = traj[i+1].arcLen - traj[i].arcLen;
            if (ds < 1e-9f) continue;
            float v1  = traj[i+1].velocity;
            float aBrk = std::min({kammLong(traj[i+1].curvature, v1, aTotal),
                                    aBrakeMax,
                                    prevBrk + maxJerk*ds});
            prevBrk = aBrk;
            float vMax = std::sqrt(v1*v1 + 2.f*aBrk*ds);
            float vNew = std::min(traj[i].velocity, vMax);
            maxChange = std::max(maxChange, std::abs(vNew-traj[i].velocity));
            traj[i].velocity = vNew;
        }
        if (maxChange < 1e-5f) break;
    }
}

void VelocityProfile::forwardPass(std::vector<TrajPoint>& traj,
                                    float maxJerk, float aTotal, float aAccelMax)
{
    if (traj.empty()) return;
    traj.front().velocity = 0.f;
    float prevAccel = 0.f;
    for (int i=1; i<int(traj.size()); i++) {
        float ds = traj[i].arcLen - traj[i-1].arcLen;
        if (ds < 1e-9f) {
            traj[i].velocity = std::min(traj[i].velocity, traj[i-1].velocity);
            continue;
        }
        float v0  = traj[i-1].velocity;
        float aAv = std::min({kammLong(traj[i-1].curvature, v0, aTotal),
                               aAccelMax,
                               prevAccel + maxJerk*ds});
        float vAc = std::sqrt(v0*v0 + 2.f*aAv*ds);
        float vNw = std::min(traj[i].velocity, vAc);
        traj[i].velocity = vNw;
        prevAccel = (v0+vNw > 1e-6f) ? (vNw*vNw - v0*v0)/(2.f*ds) : 0.f;
        prevAccel = clamp(prevAccel, -aTotal, aTotal);
        traj[i].accel = prevAccel;
    }
}

void VelocityProfile::computeJerk(std::vector<TrajPoint>& traj)
{
    for (int i=1; i<int(traj.size()); i++) {
        float ds   = traj[i].arcLen - traj[i-1].arcLen;
        float vAvg = 0.5f*(traj[i].velocity + traj[i-1].velocity);
        float dt   = (vAvg > 1e-4f) ? ds/vAvg : 1e-3f;
        float dv   = traj[i].velocity - traj[i-1].velocity;
        float a_now = dv / std::max(dt, 1e-6f);
        float a_prv = (i>1) ? traj[i-1].accel : 0.f;
        traj[i].jerk = (a_now - a_prv) / std::max(dt, 1e-6f);
        float dkds = (ds > 1e-9f) ? (traj[i].curvature - traj[i-1].curvature)/ds : 0.f;
        traj[i].ff_steer_rate = dkds * vAvg;
    }
}

float VelocityProfile::estimateTime(const std::vector<TrajPoint>& traj)
{
    float t = 0.f;
    for (int i=1; i<int(traj.size()); i++) {
        float ds   = traj[i].arcLen - traj[i-1].arcLen;
        float vAvg = 0.5f*(traj[i].velocity + traj[i-1].velocity);
        if (vAvg > 1e-6f) t += ds/vAvg;
    }
    return t;
}

float VelocityProfile::peakLatAccel(const std::vector<TrajPoint>& traj)
{
    float pk=0.f;
    for (auto& tp : traj) pk = std::max(pk, std::abs(tp.curvature)*tp.velocity*tp.velocity);
    return pk;
}

float VelocityProfile::peakLongAccel(const std::vector<TrajPoint>& traj)
{
    float pk=0.f;
    for (auto& tp : traj) pk = std::max(pk, std::abs(tp.accel));
    return pk;
}

float VelocityProfile::peakJerk(const std::vector<TrajPoint>& traj)
{
    float pk=0.f;
    for (auto& tp : traj) pk = std::max(pk, std::abs(tp.jerk));
    return pk;
}
