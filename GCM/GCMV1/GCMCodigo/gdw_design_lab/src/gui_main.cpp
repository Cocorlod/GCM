//  genetic.hpp — GA over RobotDesign
//
//  Genome  : a fixed-length float vector of the tunable genes (see encode/decode)
//  Fitness : evaluated across a set of mazes (averaged) to avoid overfitting.
//            fitness = mean speed-run time
//                      + penalties(collisions, slip, instability, energy, DNF)
//            Lower is better. The optimizer MINIMIZES completion time while
//            penalizing the failure modes the spec lists.
//  Parallel: each individual's multi-maze evaluation is dispatched via std::async.
// ═══════════════════════════════════════════════════════════════════════════
#pragma once
#include "robot_design.hpp"
#include "evaluator.hp…
[17:55, 6/1/2026] Federico: // ═══════════════════════════════════════════════════════════════════════════
//  gui_main.cpp — GDW Design Lab visualizer  (SDL2 + Dear ImGui)
//
//  ┌───────────────────────────────────────────────────────────────────────┐
//  │  UNVERIFIED IN THE AUTHOR'S SANDBOX.                                    │
//  │  The headless optimizer (main_headless.cpp) is compiled, run, and       │
//  │  verified. This GUI could NOT be compiled where it was written (no      │
//  │  SDL2, no Dear ImGui, no network to fetch them). It follows the         │
//  │  canonical imgui_impl_sdl2 + imgui_impl_sdlrenderer2 pattern and        │
//  │  reuses the SAME verified core headers, so it should build with at      │
//  │  most minor local fix-ups:                                             │
//  │      cmake -DBUILD_GUI=ON .. && cmake --build .                         │
//  └───────────────────────────────────────────────────────────────────────┘
//
//  What it shows (per the spec's VISUALIZATION section):
//    • the maze (ground-truth walls) and the centre goal
//    • the discovered best design's racing line + clothoid trajectory
//    • an animated robot body (oriented), with IR sensor rays
//    • the scout/exploration path
//    • ESKF estimate vs ground-truth pose during the speed run
//    • optimization progress (live fitness-convergence plot)
//    • best-design parameters + derived physics + population statistics
//
//  Controls: Start / Pause the optimizer, set population / mazes / generations,
//  scrub the trajectory playback, re-roll the visualization maze.
// ═══════════════════════════════════════════════════════════════════════════
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"
#include <SDL.h>

#include "genetic.hpp"
#include "evaluator.hpp"
#include "export.hpp"

#include <atomic>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <thread>
#include <vector>

using namespace gdw;

// ── Shared optimizer state (written by GA thread, read by render thread) ────
struct SharedState {
    std::mutex             mtx;
    std::atomic<bool>      running{false};
    std::atomic<bool>      quit{false};
    std::atomic<int>       generation{0};
    int                    totalGens   = 40;
    int                    populationSz = 30;
    int                    mazesPerEval = 12;
    bool                   useTVLQR     = true;

    // snapshot of the current best (guarded by mtx)
    Individual             best;
    bool                   haveBest = false;
    std::vector<float>     convergence;       // best fitness per generation
    // population fitness snapshot for the stats panel
    std::vector<float>     popFitness;
};

static SharedState g_state;

// ── GA worker thread ────────────────────────────────────────────────────────
static void optimizerThread() {
    while (!g_state.quit.load()) {
        if (!g_state.running.load()) { std::this_thread::sleep_for(std::chrono::milliseconds(30)); continue; }

        int pop, mazes, gens; bool tvlqr;
        { std::lock_guard<std::mutex> lk(g_state.mtx);
          pop=g_state.populationSz; mazes=g_state.mazesPerEval; gens=g_state.totalGens; tvlqr=g_state.useTVLQR; }

        GA ga(uint32_t(SDL_GetTicks()));
        ga.populationSize = pop; ga.mazesPerEval = mazes; ga.useTVLQR = tvlqr;
        auto seeds = makeMazeSeeds(mazes, 4242u);
        auto popv  = ga.initialPopulation();

        { std::lock_guard<std::mutex> lk(g_state.mtx); g_state.convergence.clear(); }

        for (int gd=0; gd<gens && g_state.running.load() && !g_state.quit.load(); ++gd) {
            ga.step(popv, seeds);                 // evaluate + breed
            const Individual& b = popv.front();
            std::vector<float> fits; fits.reserve(popv.size());
            for (auto& ind : popv) fits.push_back(ind.fitness);
            { std::lock_guard<std::mutex> lk(g_state.mtx);
              if (!g_state.haveBest || b.fitness < g_state.best.fitness) g_state.best = b;
              g_state.haveBest = true;
              g_state.convergence.push_back(b.fitness);
              g_state.popFitness = std::move(fits);
            }
            g_state.generation.store(gd+1);
        }
        g_state.running.store(false);             // finished a full run
    }
}

// ── A visualization "trace" of one design on one maze ───────────────────────
struct Trace {
    MazeConfig cfg;
    Maze       truth;
    std::vector<CellCoord>  scoutPath;
    std::vector<TrajPoint>  traj;        // speed-run reference
    // playback of the closed-loop run: true pose + ESKF estimate
    std::vector<Vec2>  truePos, estPos;
    std::vector<float> trueHdg;
    bool valid=false;
};

// Recompute a trace for the current best design (mirrors Evaluator's pipeline,
// plus records pose histories for playback).
static Trace buildTrace(const RobotDesign& design, uint32_t mazeSeed) {
    Trace T;
    MazeConfig cfg; T.cfg = cfg;
    PhysicsModel phys = physics::derive(design);
    RobotParams  rp   = physics::toRobotParams(design, phys, cfg.cellSize);

    generateRandomMaze(T.truth, cfg, mazeSeed);
    Maze bot; bot.init(cfg);
    ESKF kf; Vec2 sp = cfg.cellCentre(cfg.startCell); kf.reset(sp.x, sp.y, HALF_PI);
    SensorModel sensor; WallCenteringPID wc(rp);
    T.scoutPath = Explorer::explore(bot, T.truth, kf, cfg, wc, sensor);

    bool reached=false; for (auto& c : T.scoutPath) if (cfg.isGoal(c)) { reached=true; break; }
    if (!reached) return T;

    std::vector<TrajPoint> best; float bestT=INF_F;
    for (const auto& gc : cfg.goalCells) {
        FloodFill::solve(bot, {gc}, false);
        auto path = ThetaStar::findPath(bot, cfg.startCell, false);
        if (path.size()<2) continue;
        std::vector<TrajPoint> tr; RunStats st = profilePath(path, cfg, rp, rp.maxVelocity, &tr);
        if (st.valid && st.estimatedTime<bestT) { bestT=st.estimatedTime; best=std::move(tr); }
    }
    if (best.size()<2) return T;
    T.traj = best;

    // Playback: integrate along the path (mirrors Evaluator's 1-DOF executor)
    float totalArc = T.traj.back().arcLen, s=0.f, v=0.f;
    auto sample=[&](float a,float&px,float&py,float&ph,float&cv,float&vp){
        int lo=0,hi=int(T.traj.size())-1,idx=0;
        while(lo<=hi){int m=(lo+hi)/2; if(T.traj[m].arcLen<a)lo=m+1; else{idx=m;hi=m-1;}}
        idx=std::min(std::max(idx,1),int(T.traj.size())-1);
        auto&A=T.traj[idx-1];auto&B=T.traj[idx]; float seg=B.arcLen-A.arcLen,t=seg>1e-9f?(a-A.arcLen)/seg:0;
        t=std::max(0.f,std::min(1.f,t));
        px=A.x+t*(B.x-A.x);py=A.y+t*(B.y-A.y);ph=A.heading+t*angleDiff(B.heading,A.heading);
        cv=A.curvature+t*(B.curvature-A.curvature);vp=A.velocity+t*(B.velocity-A.velocity);
    };
    float px,py,ph,cv,vp; sample(0,px,py,ph,cv,vp); float prevPh=ph;
    ESKF k2; k2.reset(px,py,ph);
    std::mt19937 rng(1234);
    std::normal_distribution<float> enc(0,0.0015f), gy(0,0.0008f);
    float bias=std::normal_distribution<float>(0,0.003f)(rng);
    const float dt=0.0005f; int step=0;
    while (s<totalArc-1e-3f && step<200000) {
        sample(s,px,py,ph,cv,vp);
        float pxL,pyL,phL,cvL,vL; sample(std::min(s+0.05f,totalArc),pxL,pyL,phL,cvL,vL);
        float vT=std::max(vL,vp);
        float aDes=std::max(-phys.aBrake,std::min((vT-v)/dt,phys.aLongAccel));
        float aLat=cv*v*v, comb=std::sqrt(aDes*aDes+aLat*aLat);
        if(comb>phys.aLatMax&&comb>1e-3f) aDes*=phys.aLatMax/comb;
        v+=aDes*dt; v=std::max(0.f,std::min(v,phys.vMax)); s+=v*dt;
        float dHead=angleDiff(ph,prevPh);
        k2.predict(v*dt*(1+enc(rng)), dHead+bias*dt+gy(rng)*dt, dt); prevPh=ph;
        if(step%100==0){ CellCoord cc{int(std::floor(-py/cfg.cellSize)),int(std::floor(px/cfg.cellSize))};
            if(cfg.valid(cc)){ Vec2 ct=cfg.cellCentre(cc); float h=cfg.cellSize*0.5f, R=sensor.measurementVariance();
                std::normal_distribution<float> sn(0,sensor.NOISE_STD);
                if(!T.truth.at(cc).wall[WE]) k2.updateWallDist(0,ct.x+h,(ct.x+h)-px+sn(rng),+1.f,R);
                if(!T.truth.at(cc).wall[WW]) k2.updateWallDist(0,ct.x-h,px-(ct.x-h)+sn(rng),-1.f,R);
                if(!T.truth.at(cc).wall[WN]) k2.updateWallDist(1,ct.y+h,(ct.y+h)-py+sn(rng),+1.f,R);
                if(!T.truth.at(cc).wall[WS]) k2.updateWallDist(1,ct.y-h,py-(ct.y-h)+sn(rng),-1.f,R);
                k2.snapHeadingCardinal(); } }
        if(step%8==0){ T.truePos.push_back({px,py}); T.trueHdg.push_back(ph); T.estPos.push_back({k2.x(),k2.y()}); }
        step++;
    }
    T.valid = true;
    return T;
}

// ── World→screen mapping for the maze viewport ──────────────────────────────
static float T_cfgSpan(const Trace& T);   // maze span in metres (defined below)
struct View { float ox, oy, scale; };
static SDL_FPoint W2S(const View& v, float wx, float wy) { return { v.ox + wx*v.scale, v.oy + (-wy)*v.scale }; }

static void drawMaze(SDL_Renderer* r, const View& v, const Trace& T) {
    const MazeConfig& cfg = T.cfg; float cs = cfg.cellSize;
    // goal shading
    SDL_SetRenderDrawColor(r, 40, 70, 50, 255);
    for (auto& g : cfg.goalCells) {
        SDL_FPoint p = W2S(v, g.c*cs, -(g.r*cs));
        SDL_FRect rc{ p.x, p.y, cs*v.scale, cs*v.scale }; SDL_RenderFillRectF(r, &rc);
    }
    // walls
    SDL_SetRenderDrawColor(r, 200, 200, 210, 255);
    for (int rr=0; rr<cfg.size; ++rr) for (int cc=0; cc<cfg.size; ++cc) {
        const Cell& cell = T.truth.at(rr,cc);
        float x0=cc*cs, y0=-(rr*cs), x1=(cc+1)*cs, y1=-((rr+1)*cs);
        SDL_FPoint a,b;
        if (cell.wall[WN]) { a=W2S(v,x0,y0); b=W2S(v,x1,y0); SDL_RenderDrawLineF(r,a.x,a.y,b.x,b.y); }
        if (cell.wall[WS]) { a=W2S(v,x0,y1); b=W2S(v,x1,y1); SDL_RenderDrawLineF(r,a.x,a.y,b.x,b.y); }
        if (cell.wall[WW]) { a=W2S(v,x0,y0); b=W2S(v,x0,y1); SDL_RenderDrawLineF(r,a.x,a.y,b.x,b.y); }
        if (cell.wall[WE]) { a=W2S(v,x1,y0); b=W2S(v,x1,y1); SDL_RenderDrawLineF(r,a.x,a.y,b.x,b.y); }
    }
}

static void drawPolyline(SDL_Renderer* r, const View& v, const std::vector<SDL_FPoint>& pts) {
    for (size_t i=1;i<pts.size();++i) SDL_RenderDrawLineF(r, pts[i-1].x, pts[i-1].y, pts[i].x, pts[i].y);
}

int main(int, char**) {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) { std::fprintf(stderr,"SDL init: %s\n", SDL_GetError()); return 1; }
    SDL_Window*   win = SDL_CreateWindow("GDW Design Lab", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                         1280, 800, SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    SDL_Renderer* ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_PRESENTVSYNC | SDL_RENDERER_ACCELERATED);

    IMGUI_CHECKVERSION(); ImGui::CreateContext(); ImGui::StyleColorsDark();
    ImGui_ImplSDL2_InitForSDLRenderer(win, ren);
    ImGui_ImplSDLRenderer2_Init(ren);

    std::thread worker(optimizerThread);

    // visualization state
    Trace trace; uint32_t vizMaze = 4242u;
    RobotDesign vizDesign;                 // default until first best arrives
    bool firstTrace=false; float prevBestFit=INF_F;
    float playT = 0.f; bool playing = true; float playSpeed = 1.0f;

    bool done=false;
    while (!done) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            ImGui_ImplSDL2_ProcessEvent(&e);
            if (e.type == SDL_QUIT) done=true;
        }

        // Refresh the trace when a new best design appears.
        { std::lock_guard<std::mutex> lk(g_state.mtx);
          if (g_state.haveBest && (g_state.best.fitness < prevBestFit - 1e-4f || !firstTrace)) {
              vizDesign = g_state.best.design; prevBestFit = g_state.best.fitness; firstTrace=true;
              trace = buildTrace(vizDesign, vizMaze); playT=0.f;
          }
        }

        ImGui_ImplSDLRenderer2_NewFrame(); ImGui_ImplSDL2_NewFrame(); ImGui::NewFrame();

        // ── Control panel ────────────────────────────────────────────────
        ImGui::Begin("Optimizer");
        {
            bool running = g_state.running.load();
            std::lock_guard<std::mutex> lk(g_state.mtx);
            ImGui::SliderInt("Population", &g_state.populationSz, 8, 200);
            ImGui::SliderInt("Mazes/eval", &g_state.mazesPerEval, 1, 120);
            ImGui::SliderInt("Generations", &g_state.totalGens, 1, 200);
            ImGui::Checkbox("TVLQR", &g_state.useTVLQR);
            ImGui::Separator();
            if (!running) { if (ImGui::Button("Start")) { g_state.generation.store(0); g_state.running.store(true); } }
            else          { if (ImGui::Button("Pause")) g_state.running.store(false); }
            ImGui::SameLine();
            ImGui::Text("gen %d / %d", g_state.generation.load(), g_state.totalGens);
            if (!g_state.convergence.empty())
                ImGui::PlotLines("fitness", g_state.convergence.data(), int(g_state.convergence.size()),
                                 0, nullptr, FLT_MAX, FLT_MAX, ImVec2(0,80));
        }
        ImGui::End();

        // ── Best design + physics + population stats ─────────────────────
        ImGui::Begin("Best design");
        { std::lock_guard<std::mutex> lk(g_state.mtx);
          if (g_state.haveBest) {
            const RobotDesign& d = g_state.best.design; PhysicsModel p = physics::derive(d);
            ImGui::Text("fitness        %.3f", g_state.best.fitness);
            ImGui::Text("exec time      %.3f s", g_state.best.meanExecTime);
            ImGui::Text("peak velocity  %.2f m/s", g_state.best.meanPeakV);
            ImGui::Text("reliability    %.0f %%", g_state.best.reliability*100);
            ImGui::Separator();
            ImGui::Text("length  %.1f mm   width %.1f mm", d.length*1000, d.width*1000);
            ImGui::Text("wheel d %.1f mm   track %.1f mm", d.wheelDiameter*1000, d.trackWidth*1000);
            ImGui::Text("mu      %.2f      mass  %.1f g", d.tireFriction, d.totalMass()*1000);
            ImGui::Text("fan     %s %.1f W  downforce %.2f N", d.hasFan?"ON":"off", d.fanPower_W, p.downforce);
            ImGui::Separator();
            ImGui::Text("a_lat_max %.2f m/s^2 (%.2f g)", p.aLatMax, p.aLatMax/GRAV);
            ImGui::Text("a_long    %.2f m/s^2", p.aLongAccel);
            ImGui::Text("v_max     %.2f m/s", p.vMax);
            ImGui::Separator();
            ImGui::Text("mean slip      %.3f", g_state.best.meanSlip);
            ImGui::Text("mean collisions %.2f", g_state.best.meanCollisions);
            ImGui::Text("localization   %.1f mm", g_state.best.meanLoc*1000);
            if (!g_state.popFitness.empty())
                ImGui::PlotHistogram("pop fitness", g_state.popFitness.data(),
                                     int(g_state.popFitness.size()), 0, nullptr, FLT_MAX, FLT_MAX, ImVec2(0,70));
          } else ImGui::Text("Press Start to begin optimization.");
        }
        ImGui::End();

        // ── Maze / robot viewport (ImGui window hosting raw SDL draws) ────
        ImGui::Begin("Maze & run");
        ImGui::Checkbox("play", &playing); ImGui::SameLine();
        ImGui::SliderFloat("speed", &playSpeed, 0.1f, 4.0f); ImGui::SameLine();
        if (ImGui::Button("re-roll maze")) { vizMaze = uint32_t(SDL_GetTicks()); trace = buildTrace(vizDesign, vizMaze); playT=0.f; }
        ImGui::SliderFloat("scrub", &playT, 0.f, 1.f);
        ImGui::Text(trace.valid ? "trace OK" : "no valid run for this design/maze");
        ImGui::End();

        // advance playback
        if (playing && trace.valid && !trace.truePos.empty()) {
            playT += playSpeed * 0.004f; if (playT>1.f) playT=0.f;
        }

        // ── Render ────────────────────────────────────────────────────────
        ImGui::Render();
        SDL_SetRenderDrawColor(ren, 18, 18, 22, 255); SDL_RenderClear(ren);

        // Maze viewport occupies the right portion of the window.
        int W,H; SDL_GetRendererOutputSize(ren, &W, &H);
        float span = T_cfgSpan(trace);
        float side = std::min(float(H)-40.f, float(W)*0.55f);
        View view{ float(W) - side - 20.f, 20.f, side / span };

        if (trace.valid || true) {
            drawMaze(ren, view, trace);

            // racing line
            if (!trace.traj.empty()) {
                SDL_SetRenderDrawColor(ren, 90, 160, 255, 255);
                std::vector<SDL_FPoint> rl; rl.reserve(trace.traj.size());
                for (auto& tp : trace.traj) rl.push_back(W2S(view, tp.x, tp.y));
                drawPolyline(ren, view, rl);
            }
            // scout path
            if (!trace.scoutPath.empty()) {
                SDL_SetRenderDrawColor(ren, 120, 120, 90, 160);
                std::vector<SDL_FPoint> sp; sp.reserve(trace.scoutPath.size());
                for (auto& c : trace.scoutPath) { Vec2 q=trace.cfg.cellCentre(c); sp.push_back(W2S(view,q.x,q.y)); }
                drawPolyline(ren, view, sp);
            }
            // ESKF estimate (red) vs ground truth (green) + robot body + sensor rays
            if (!trace.truePos.empty()) {
                int n = int(trace.truePos.size());
                int i = std::min(n-1, std::max(0, int(playT*(n-1))));
                // estimate trail
                SDL_SetRenderDrawColor(ren, 230, 80, 80, 200);
                for (int k=1;k<=i;k++){ auto a=W2S(view,trace.estPos[k-1].x,trace.estPos[k-1].y),
                                             b=W2S(view,trace.estPos[k].x,trace.estPos[k].y);
                                        SDL_RenderDrawLineF(ren,a.x,a.y,b.x,b.y); }
                // true robot body (oriented rectangle)
                Vec2 tp = trace.truePos[i]; float th = trace.trueHdg[i];
                float hl = vizDesign.length*0.5f, hw = vizDesign.width*0.5f;
                float c=std::cos(th), s=std::sin(th);
                Vec2 corners[4] = {
                    { tp.x + c*hl - s*hw,  tp.y + s*hl + c*hw },
                    { tp.x + c*hl + s*hw,  tp.y + s*hl - c*hw },
                    { tp.x - c*hl + s*hw,  tp.y - s*hl - c*hw },
                    { tp.x - c*hl - s*hw,  tp.y - s*hl + c*hw },
                };
                SDL_SetRenderDrawColor(ren, 90, 230, 120, 255);
                for (int k=0;k<4;k++){ auto a=W2S(view,corners[k].x,corners[k].y),
                                            b=W2S(view,corners[(k+1)%4].x,corners[(k+1)%4].y);
                                       SDL_RenderDrawLineF(ren,a.x,a.y,b.x,b.y); }
                // IR sensor rays (front + side + diagonals)
                SDL_SetRenderDrawColor(ren, 255, 200, 60, 170);
                for (int sIdx=0;sIdx<5;sIdx++){
                    float ax = vizDesign.sensorAng[sIdx];
                    float rx = tp.x + std::cos(th+ax)*0.12f, ry = tp.y + std::sin(th+ax)*0.12f;
                    auto a=W2S(view,tp.x,tp.y), b=W2S(view,rx,ry);
                    SDL_RenderDrawLineF(ren,a.x,a.y,b.x,b.y);
                }
            }
        }

        ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), ren);
        SDL_RenderPresent(ren);

        if (g_state.quit.load()) done=true;
    }

    g_state.running.store(false); g_state.quit.store(true);
    if (worker.joinable()) worker.join();

    ImGui_ImplSDLRenderer2_Shutdown(); ImGui_ImplSDL2_Shutdown(); ImGui::DestroyContext();
    SDL_DestroyRenderer(ren); SDL_DestroyWindow(win); SDL_Quit();
    return 0;
}

// maze span in metres (free function; declared late, defined here for clarity)
static float T_cfgSpan(const Trace& T) { return T.cfg.size * T.cfg.cellSize; }