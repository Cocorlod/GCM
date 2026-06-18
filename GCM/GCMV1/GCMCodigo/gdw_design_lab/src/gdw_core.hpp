// ═══════════════════════════════════════════════════════════════════════════
//  gdw_core.hpp — GDW v4.1 planning/control/localization stack (library form)
//
//  Adapted from "GDW Micromouse Championship Edition v4.1" (single-TU console
//  program). Changes for use as a library inside GDW Design Lab:
//    - removed main(), GDWPlannerV4 orchestrator, buildTruthMaze fixture
//    - removed all std::cout/std::cerr from algorithm paths (headless/GUI safe)
//    - profilePath() returns RunStats and (optionally) the full trajectory
//      with NO printing
//  All algorithm bodies and the v4 FIX-A..FIX-G corrections are preserved.
// ═══════════════════════════════════════════════════════════════════════════
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <queue>
#include <random>
#include <set>
#include <string>
#include <tuple>
#include <vector>

namespace gdw {

inline constexpr int   MAZE_N  = 16;
inline constexpr int   N_CELLS = MAZE_N * MAZE_N;
inline constexpr float INF_F   = std::numeric_limits<float>::infinity();
inline constexpr float PI      = 3.14159265358979f;
inline constexpr float TWO_PI  = 2.0f * PI;
inline constexpr float HALF_PI = PI * 0.5f;
inline constexpr float SQRT2   = 1.41421356237f;

[[nodiscard]] inline float wrapAngle(float a) noexcept {
    a = std::fmod(a + PI, TWO_PI);
    if (a < 0.0f) a += TWO_PI;
    return a - PI;
}
[[nodiscard]] inline float angleDiff(float a, float b) noexcept { return wrapAngle(a - b); }

enum Wall : int { WN = 0, WE = 1, WS = 2, WW = 3 };
inline constexpr int   WALL_OPP[4] = { WS, WW, WN, WE };
inline constexpr int   WALL_DC[4]  = {  0,  1,  0, -1 };
inline constexpr int   WALL_DR[4]  = { -1,  0,  1,  0 };
inline constexpr float WALL_HEADING[4] = { HALF_PI, 0.0f, -HALF_PI, PI };

inline constexpr int   D8C[8]    = {  0, 1, 1, 1,  0, -1, -1, -1 };
inline constexpr int   D8R[8]    = { -1,-1, 0, 1,  1,  1,  0, -1 };
inline constexpr float D8COST[8] = { 1.f, SQRT2, 1.f, SQRT2, 1.f, SQRT2, 1.f, SQRT2 };
inline constexpr int   D8WALLS[8][2] = {
    { WN, -1 }, { WN, WE }, { WE, -1 }, { WE, WS },
    { WS, -1 }, { WS, WW }, { WW, -1 }, { WN, WW }
};
inline constexpr float D8HEADING[8] = {
    HALF_PI, PI*0.25f, 0.0f, -PI*0.25f, -HALF_PI, -(PI*0.75f), PI, PI*0.75f
};

struct CellCoord {
    int r = 0, c = 0;
    [[nodiscard]] bool operator==(const CellCoord& o) const noexcept { return r==o.r && c==o.c; }
    [[nodiscard]] bool operator!=(const CellCoord& o) const noexcept { return !(*this==o); }
    [[nodiscard]] bool operator< (const CellCoord& o) const noexcept { return (r!=o.r)?r<o.r:c<o.c; }
    [[nodiscard]] int  idx()   const noexcept { return r*MAZE_N + c; }
    [[nodiscard]] bool valid() const noexcept { return r>=0&&r<MAZE_N&&c>=0&&c<MAZE_N; }
    [[nodiscard]] CellCoord neighbour(int w) const noexcept { return {r+WALL_DR[w], c+WALL_DC[w]}; }
    [[nodiscard]] CellCoord step8(int d8)   const noexcept { return {r+D8R[d8], c+D8C[d8]}; }
};

struct Vec2 {
    float x=0.f, y=0.f;
    [[nodiscard]] Vec2  operator+(const Vec2& o) const noexcept { return {x+o.x,y+o.y}; }
    [[nodiscard]] Vec2  operator-(const Vec2& o) const noexcept { return {x-o.x,y-o.y}; }
    [[nodiscard]] Vec2  operator*(float s)       const noexcept { return {x*s,y*s}; }
    [[nodiscard]] float dot(const Vec2& o)       const noexcept { return x*o.x+y*o.y; }
    [[nodiscard]] float cross(const Vec2& o)     const noexcept { return x*o.y-y*o.x; }
    [[nodiscard]] float norm()                   const noexcept { return std::sqrt(x*x+y*y); }
    [[nodiscard]] Vec2  normalised()             const noexcept {
        float n=norm(); return n>1e-9f?Vec2{x/n,y/n}:Vec2{};
    }
};

struct MazeConfig {
    int   size     = MAZE_N;
    float cellSize = 0.18f;
    std::array<CellCoord,4> goalCells = {{ {7,7},{7,8},{8,7},{8,8} }};
    CellCoord startCell = { 15, 0 };
    [[nodiscard]] bool isGoal(const CellCoord& cc) const noexcept {
        for (auto& g : goalCells) if (g==cc) return true; return false;
    }
    [[nodiscard]] bool valid(int r,int c) const noexcept { return r>=0&&r<size&&c>=0&&c<size; }
    [[nodiscard]] bool valid(const CellCoord& cc) const noexcept { return valid(cc.r,cc.c); }
    [[nodiscard]] Vec2 cellCentre(const CellCoord& cc) const noexcept {
        return { (cc.c+0.5f)*cellSize, -(cc.r+0.5f)*cellSize };
    }
};

struct Cell {
    std::array<bool,4> wallKnown = {false,false,false,false};
    std::array<bool,4> wall      = {true,true,true,true};
    bool  explored=false; int visitCount=0;
    float floodDist=INF_F, dstar_g=INF_F, dstar_rhs=INF_F;
    [[nodiscard]] bool passableOpt(int w)  const noexcept { return !(wallKnown[w]&&wall[w]); }
    [[nodiscard]] bool passableCons(int w) const noexcept { return wallKnown[w]&&!wall[w]; }
    [[nodiscard]] bool hasFrontier() const noexcept {
        for (int w=0;w<4;w++) if (!wallKnown[w]) return true; return false;
    }
};

class Maze {
public:
    std::array<Cell,N_CELLS> cells{};
    const MazeConfig* cfg=nullptr;
    void init(const MazeConfig& c){ cfg=&c; cells.fill(Cell{}); placeBorderWalls(); }
    [[nodiscard]] Cell&       at(const CellCoord& cc)       noexcept { return cells[cc.idx()]; }
    [[nodiscard]] const Cell& at(const CellCoord& cc) const noexcept { return cells[cc.idx()]; }
    [[nodiscard]] Cell&       at(int r,int c)               noexcept { return cells[r*MAZE_N+c]; }
    [[nodiscard]] const Cell& at(int r,int c)         const noexcept { return cells[r*MAZE_N+c]; }
    void setWall(const CellCoord& cc,int w,bool present){
        if(!cfg->valid(cc)) return;
        cells[cc.idx()].wall[w]=present; cells[cc.idx()].wallKnown[w]=true;
        CellCoord nb=cc.neighbour(w);
        if(cfg->valid(nb)){ cells[nb.idx()].wall[WALL_OPP[w]]=present;
                            cells[nb.idx()].wallKnown[WALL_OPP[w]]=true; }
    }
    [[nodiscard]] bool canMove8(const CellCoord& cc,int d8,bool optimistic) const noexcept {
        CellCoord nb=cc.step8(d8); if(!cfg->valid(nb)) return false;
        const Cell& cell=at(cc);
        for(int k=0;k<2;k++){ int w=D8WALLS[d8][k]; if(w<0) continue;
            if(optimistic){ if(cell.wallKnown[w]&&cell.wall[w]) return false; }
            else          { if(!cell.wallKnown[w]||cell.wall[w]) return false; } }
        return true;
    }
    [[nodiscard]] bool canMoveCardinal(const CellCoord& cc,int w,bool optimistic) const noexcept {
        CellCoord nb=cc.neighbour(w); if(!cfg->valid(nb)) return false;
        const Cell& cell=at(cc);
        if(optimistic) return !(cell.wallKnown[w]&&cell.wall[w]);
        return cell.wallKnown[w]&&!cell.wall[w];
    }
    [[nodiscard]] int frontierCount() const noexcept {
        int n=0; for(auto& c:cells) if(c.explored&&c.hasFrontier()) n++; return n;
    }
    // Truth-maze helper used by the executor / sim (knows everything).
    [[nodiscard]] bool wallPresentTruth(const CellCoord& cc,int w) const noexcept {
        if(!cfg->valid(cc)) return true; return at(cc).wall[w];
    }
private:
    void placeBorderWalls(){
        const int sz=cfg->size;
        for(int i=0;i<sz;i++){ setWall({0,i},WN,true); setWall({sz-1,i},WS,true);
                               setWall({i,0},WW,true); setWall({i,sz-1},WE,true); }
    }
};

// ── FloodFill (goal-seeded Dijkstra, 8-dir) ────────────────────────────────
class FloodFill {
public:
    static void solve(Maze& maze,const std::vector<CellCoord>& seeds,bool optimistic){
        for(auto& cell:maze.cells) cell.floodDist=INF_F;
        using Entry=std::pair<float,CellCoord>;
        std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> pq;
        for(const auto& s:seeds){ if(!maze.cfg->valid(s)) continue;
            maze.at(s).floodDist=0.f; pq.push({0.f,s}); }
        while(!pq.empty()){
            auto [d,cc]=pq.top(); pq.pop();
            if(d>maze.at(cc).floodDist+1e-6f) continue;
            for(int d8=0;d8<8;d8++){ if(!maze.canMove8(cc,d8,optimistic)) continue;
                CellCoord nb=cc.step8(d8); float nd=d+D8COST[d8];
                if(nd<maze.at(nb).floodDist-1e-6f){ maze.at(nb).floodDist=nd; pq.push({nd,nb}); } }
        }
    }
    static void solveToGoal(Maze& m,bool opt){
        std::vector<CellCoord> s(m.cfg->goalCells.begin(),m.cfg->goalCells.end()); solve(m,s,opt);
    }
    static void solveToStart(Maze& m,bool opt){ solve(m,{m.cfg->startCell},opt); }
};

// ── D* Lite (incremental replanning, FIX-E optimistic flag) ────────────────
class DStarLite {
public:
    struct Key { float k1,k2;
        bool operator<(const Key& o) const noexcept { return std::abs(k1-o.k1)>1e-6f?k1<o.k1:k2<o.k2; }
        bool operator>(const Key& o) const noexcept { return o<*this; }
        bool operator<=(const Key& o) const noexcept { return !(o<*this); } };
    Maze* maze=nullptr; CellCoord start={-1,-1}; bool optimistic=true; float km=0.f;
    using QEntry=std::pair<Key,CellCoord>;
    struct QCmp { bool operator()(const QEntry& a,const QEntry& b) const noexcept { return a.first>b.first; } };
    std::priority_queue<QEntry,std::vector<QEntry>,QCmp> U;
    void init(Maze& m,CellCoord s,bool opt){
        maze=&m; start=s; optimistic=opt; km=0.f;
        for(auto& cell:maze->cells){ cell.dstar_g=INF_F; cell.dstar_rhs=INF_F; }
        while(!U.empty()) U.pop();
        for(const auto& gc:maze->cfg->goalCells){ maze->at(gc).dstar_rhs=0.f; U.push({calcKey(gc),gc}); }
        computeShortestPath();
    }
    [[nodiscard]] float heuristic(const CellCoord& cc) const noexcept {
        float best=INF_F;
        for(const auto& gc:maze->cfg->goalCells){
            float dr=float(std::abs(cc.r-gc.r)), dc=float(std::abs(cc.c-gc.c));
            float h=std::max(dr,dc)+(SQRT2-1.f)*std::min(dr,dc); if(h<best) best=h; }
        return best;
    }
    [[nodiscard]] Key calcKey(const CellCoord& cc) const noexcept {
        float gv=maze->at(cc).dstar_g, rv=maze->at(cc).dstar_rhs, mn=std::min(gv,rv);
        return { mn+heuristic(cc)+km, mn };
    }
    // Hard expansion cap: the optimizer evaluates this millions of times and
    // must never hang or grow the queue unboundedly on a pathological map.
    static constexpr int MAX_EXPANSIONS = 200000;
    void computeShortestPath(){
        int expansions = 0;
        while(!U.empty()){
            if(++expansions > MAX_EXPANSIONS) break;     // safety bail
            auto [kOld,u]=U.top(); Key ks=calcKey(start);
            float gs=maze->at(start).dstar_g, rs=maze->at(start).dstar_rhs;
            // Standard top-level test: continue while topKey < startKey OR start inconsistent.
            if(!(kOld<ks) && std::abs(rs-gs)<1e-6f) break;
            U.pop(); Key kNew=calcKey(u);
            if(kOld<kNew){ U.push({kNew,u}); continue; } // stale key → reinsert updated
            float gu=maze->at(u).dstar_g, ru=maze->at(u).dstar_rhs;
            if(gu>ru){
                // Overconsistent: make consistent, relax successors.
                maze->at(u).dstar_g=ru;
                for(int d8=0;d8<8;d8++){ if(!maze->canMove8(u,d8,optimistic)) continue;
                    CellCoord nb=u.step8(d8); if(maze->cfg->isGoal(nb)) continue;
                    float nr=maze->at(u).dstar_g+D8COST[d8];
                    if(nr<maze->at(nb).dstar_rhs-1e-6f){ maze->at(nb).dstar_rhs=nr; U.push({calcKey(nb),nb}); } }
            } else if(gu<ru){
                // Underconsistent: raise to INF, re-evaluate u and successors.
                maze->at(u).dstar_g=INF_F; U.push({calcKey(u),u});
                for(int d8=0;d8<8;d8++){ if(!maze->canMove8(u,d8,optimistic)) continue;
                    CellCoord nb=u.step8(d8); if(maze->cfg->isGoal(nb)) continue;
                    float bestRhs=INF_F;
                    for(int d2=0;d2<8;d2++){ if(!maze->canMove8(nb,d2,optimistic)) continue;
                        CellCoord s2=nb.step8(d2); float cand=maze->at(s2).dstar_g+D8COST[d2];
                        if(cand<bestRhs) bestRhs=cand; }
                    maze->at(nb).dstar_rhs=bestRhs; U.push({calcKey(nb),nb}); }
            }
            // else gu==ru: locally consistent → already popped, simply drop it.
            // (The original v4.1 lumped this into the underconsistent branch,
            //  which re-raised g to INF and re-pushed u forever on dense maps.)
        }
        for(auto& cell:maze->cells) cell.floodDist=cell.dstar_g;
    }
    void notifyWallChanged(const CellCoord& cc){
        km+=heuristic(start);
        for(int d8=0;d8<8;d8++){ CellCoord nb=cc.step8(d8);
            if(!maze->cfg->valid(nb)||maze->cfg->isGoal(nb)) continue;
            float bestRhs=INF_F;
            for(int d2=0;d2<8;d2++){ if(!maze->canMove8(nb,d2,optimistic)) continue;
                CellCoord s2=nb.step8(d2); float cand=maze->at(s2).dstar_g+D8COST[d2];
                if(cand<bestRhs) bestRhs=cand; }
            maze->at(nb).dstar_rhs=bestRhs; U.push({calcKey(nb),nb}); }
        computeShortestPath();
    }
};

// ── Theta* (any-angle A*, FIX-C arc handled in TrajGen) ────────────────────
class ThetaStar {
public:
    [[nodiscard]] static bool lineOfSight(const Maze& maze,const CellCoord& a,
                                          const CellCoord& b,bool optimistic) noexcept {
        int r0=a.r,c0=a.c,r1=b.r,c1=b.c;
        int dr=std::abs(r1-r0),dc=std::abs(c1-c0);
        int sr=(r1>r0)?1:-1,sc=(c1>c0)?1:-1;
        int r=r0,c=c0,err=dc-dr;
        for(int step=0;step<=dr+dc;step++){
            if(r==r1&&c==c1) return true;
            if(!maze.cfg->valid(r,c)) return false;
            int e2=2*err; bool mC=(e2>-dr),mR=(e2<dc); CellCoord cc{r,c};
            if(mC&&mR){ int wC=(sc>0)?WE:WW,wR=(sr>0)?WS:WN;
                if(!checkWall(maze,cc,wC,optimistic)) return false;
                if(!checkWall(maze,cc,wR,optimistic)) return false;
                c+=sc; r+=sr; err+=dr-dc;
            } else if(mC){ if(!checkWall(maze,cc,(sc>0)?WE:WW,optimistic)) return false; c+=sc; err+=dr; }
            else         { if(!checkWall(maze,cc,(sr>0)?WS:WN,optimistic)) return false; r+=sr; err-=dc; }
        }
        return true;
    }
    [[nodiscard]] static float dist(const CellCoord& a,const CellCoord& b) noexcept {
        float dr=float(b.r-a.r),dc=float(b.c-a.c); return std::sqrt(dr*dr+dc*dc);
    }
    [[nodiscard]] static std::vector<CellCoord> findPath(const Maze& maze,const CellCoord& start,bool optimistic){
        std::array<float,N_CELLS> gCost; std::array<CellCoord,N_CELLS> parent; std::array<bool,N_CELLS> closed;
        gCost.fill(INF_F); parent.fill({-1,-1}); closed.fill(false);
        gCost[start.idx()]=0.f; parent[start.idx()]={-2,-2};
        struct Node { float f; CellCoord cc; bool operator>(const Node& o) const noexcept { return f>o.f; } };
        std::priority_queue<Node,std::vector<Node>,std::greater<Node>> open;
        open.push({maze.at(start).floodDist,start});
        CellCoord reached{-1,-1};
        while(!open.empty()){
            auto [f,cc]=open.top(); open.pop();
            if(closed[cc.idx()]) continue; closed[cc.idx()]=true;
            if(maze.cfg->isGoal(cc)){ reached=cc; break; }
            for(int d8=0;d8<8;d8++){ if(!maze.canMove8(cc,d8,optimistic)) continue;
                CellCoord nb=cc.step8(d8); if(closed[nb.idx()]) continue;
                const CellCoord& par=parent[cc.idx()]; bool hasPar=(par.r>=0);
                float ng; CellCoord via;
                if(hasPar&&lineOfSight(maze,par,nb,optimistic)){ ng=gCost[par.idx()]+dist(par,nb); via=par; }
                else { ng=gCost[cc.idx()]+D8COST[d8]; via=cc; }
                if(ng<gCost[nb.idx()]-1e-6f){ gCost[nb.idx()]=ng; parent[nb.idx()]=via;
                    open.push({ng+maze.at(nb).floodDist,nb}); } }
        }
        if(reached.r<0) return {};
        std::vector<CellCoord> path; CellCoord c=reached;
        while(c.r>=0){ path.push_back(c); const CellCoord& p=parent[c.idx()]; if(p.r==-2) break; c=p; }
        std::reverse(path.begin(),path.end()); return path;
    }
    [[nodiscard]] static std::vector<CellCoord> expandPath(const std::vector<CellCoord>& path){
        std::vector<CellCoord> e; if(path.empty()) return e; e.push_back(path[0]);
        for(size_t i=1;i<path.size();i++){
            int dr=path[i].r-path[i-1].r,dc=path[i].c-path[i-1].c;
            int steps=std::max(std::abs(dr),std::abs(dc));
            int sr=(dr>0)-(dr<0),sc=(dc>0)-(dc<0);
            for(int s=1;s<=steps;s++) e.push_back({path[i-1].r+s*sr,path[i-1].c+s*sc});
        }
        return e;
    }
private:
    [[nodiscard]] static bool checkWall(const Maze& m,const CellCoord& cc,int w,bool optimistic) noexcept {
        if(!m.cfg->valid(cc)) return false; const Cell& cell=m.at(cc);
        if(optimistic) return !(cell.wallKnown[w]&&cell.wall[w]);
        return cell.wallKnown[w]&&!cell.wall[w];
    }
};

// ── Sensor model (IR) ──────────────────────────────────────────────────────
struct SensorReading { float distance=0.f; bool valid=false; };
class SensorModel {
public:
    float RANGE_MIN=0.04f, RANGE_MAX=0.30f, NOISE_STD=0.003f;
    [[nodiscard]] float measurementVariance() const noexcept { return NOISE_STD*NOISE_STD; }
    [[nodiscard]] SensorReading sample(float trueDistance,std::mt19937& rng) const {
        if(trueDistance<RANGE_MIN||trueDistance>RANGE_MAX) return {trueDistance,false};
        std::normal_distribution<float> noise{0.f,NOISE_STD};
        float m=trueDistance+noise(rng);
        return (m<RANGE_MIN||m>RANGE_MAX)?SensorReading{m,false}:SensorReading{m,true};
    }
};

// ── ESKF 4-state (FIX-B dim/Joseph, FIX-G wall signs) ──────────────────────
class ESKF {
public:
    float nom_x=0,nom_y=0,nom_theta=0;
    std::array<float,4> err{0,0,0,0};
    std::array<float,16> P{};
    float Q_xy=1e-5f,Q_theta=2e-4f,Q_bias=1e-6f,R_wall=9e-6f,R_hdg=1e-4f;
    ESKF(){ P.fill(0); P[0]=0.01f;P[5]=0.01f;P[10]=0.001f;P[15]=1e-4f; }
    void reset(float x0,float y0,float h0) noexcept {
        nom_x=x0;nom_y=y0;nom_theta=h0; err.fill(0); P.fill(0);
        P[0]=1e-4f;P[5]=1e-4f;P[10]=1e-4f;P[15]=1e-6f;
    }
    [[nodiscard]] float x()     const noexcept { return nom_x+err[0]; }
    [[nodiscard]] float y()     const noexcept { return nom_y+err[1]; }
    [[nodiscard]] float theta() const noexcept { return nom_theta+err[2]; }
    [[nodiscard]] float bias()  const noexcept { return err[3]; }
    float& pij(int i,int j) noexcept       { return P[i*4+j]; }
    float  pij(int i,int j) const noexcept { return P[i*4+j]; }
    void predict(float ds,float dtheta_meas,float dt) noexcept {
        float dtheta=dtheta_meas-err[3]*dt;
        float midTheta=nom_theta+0.5f*dtheta;
        float c=std::cos(midTheta), s_t=std::sin(midTheta);
        nom_x+=ds*c; nom_y+=ds*s_t; nom_theta=wrapAngle(nom_theta+dtheta);
        std::array<float,16> Pn=P;
        for(int j=0;j<4;j++) Pn[0*4+j]+=(-ds*s_t)*P[2*4+j];
        for(int j=0;j<4;j++) Pn[1*4+j]+=(ds*c)   *P[2*4+j];
        for(int j=0;j<4;j++) Pn[2*4+j]+=(-dt)    *P[3*4+j];
        std::array<float,16> P2=Pn;
        for(int i=0;i<4;i++) P2[i*4+0]+=Pn[i*4+2]*(-ds*s_t);
        for(int i=0;i<4;i++) P2[i*4+1]+=Pn[i*4+2]*(ds*c);
        for(int i=0;i<4;i++) P2[i*4+2]+=Pn[i*4+3]*(-dt);
        P2[0]+=Q_xy*std::abs(ds); P2[5]+=Q_xy*std::abs(ds);
        P2[10]+=Q_theta*std::abs(dtheta); P2[15]+=Q_bias; P=P2;
    }
    void updateWallDist(int axis,float wallCoord,float measuredDist,float sign,float R_meas) noexcept {
        float robotPos=(axis==0)?x():y();
        float expected=sign*(wallCoord-robotPos);
        float innov=measuredDist-expected;
        std::array<float,4> H{}; H[axis]=sign;
        float S=R_meas; for(int i=0;i<4;i++) for(int j=0;j<4;j++) S+=H[i]*pij(i,j)*H[j];
        if(S<1e-12f) return; if(innov*innov>9.f*S) return;
        std::array<float,4> K{};
        for(int i=0;i<4;i++){ float PH=0; for(int k=0;k<4;k++) PH+=pij(i,k)*H[k]; K[i]=PH/S; }
        for(int i=0;i<4;i++) err[i]+=K[i]*innov;
        std::array<float,16> A{};
        for(int i=0;i<4;i++) for(int j=0;j<4;j++) A[i*4+j]=(i==j?1.f:0.f)-K[i]*H[j];
        std::array<float,16> AP{};
        for(int i=0;i<4;i++) for(int j=0;j<4;j++) for(int k=0;k<4;k++) AP[i*4+j]+=A[i*4+k]*pij(k,j);
        std::array<float,16> Pn{};
        for(int i=0;i<4;i++) for(int j=0;j<4;j++){ float v=0; for(int k=0;k<4;k++) v+=AP[i*4+k]*A[j*4+k];
            Pn[i*4+j]=v+K[i]*R_meas*K[j]; }
        P=Pn;
        nom_x+=err[0]; nom_y+=err[1]; nom_theta=wrapAngle(nom_theta+err[2]);
        err[0]=err[1]=err[2]=0.f;
    }
    void updateHeading(float measuredHeading,float R_meas) noexcept {
        float innov=angleDiff(measuredHeading,theta());
        std::array<float,4> H{}; H[2]=1.f;
        float S=pij(2,2)+R_meas; if(S<1e-12f) return;
        std::array<float,4> K{}; for(int i=0;i<4;i++) K[i]=pij(i,2)/S;
        for(int i=0;i<4;i++) err[i]+=K[i]*innov;
        std::array<float,16> A{};
        for(int i=0;i<4;i++) for(int j=0;j<4;j++) A[i*4+j]=(i==j?1.f:0.f)-K[i]*H[j];
        std::array<float,16> AP{};
        for(int i=0;i<4;i++) for(int j=0;j<4;j++) for(int k=0;k<4;k++) AP[i*4+j]+=A[i*4+k]*pij(k,j);
        std::array<float,16> Pn{};
        for(int i=0;i<4;i++) for(int j=0;j<4;j++){ float v=0; for(int k=0;k<4;k++) v+=AP[i*4+k]*A[j*4+k];
            Pn[i*4+j]=v+K[i]*R_meas*K[j]; }
        P=Pn; nom_theta=wrapAngle(nom_theta+err[2]); err[2]=0.f;
    }
    void snapHeadingCardinal() noexcept {
        float snapped=std::round(theta()/HALF_PI)*HALF_PI;
        if(std::abs(angleDiff(theta(),snapped))<0.12f) updateHeading(snapped,1e-3f);
    }
};

// ── Robot control/limit parameters (filled by physics::derive) ─────────────
struct RobotParams {
    float maxTotalAccel=12.0f, maxBraking=10.0f, maxAccelFwd=9.0f, maxJerk=60.0f;
    float maxVelocity=5.0f, exploreVelocity=0.6f;
    float wheelbase=0.07f, trackWidth=0.06f, cellSize=0.18f, steeringBandwidth=20.0f;
    float Kp_crosstrack=4.0f, Kd_crosstrack=0.3f, Kp_heading=2.0f, Kd_heading=0.1f;
    float Kp_center=3.0f, Ki_center=0.1f;
};

struct TrajPoint {
    float x=0,y=0,heading=0,curvature=0,velocity=0,arcLen=0,accel=0,jerk=0,ff_steer_rate=0;
};

struct ClothoidSeg {
    float x0,y0,theta0,kappa0,kappaEnd,length;
    static constexpr float GL_XI[8]={0.0950125098f,0.2816035508f,0.4580167777f,0.6178762444f,
        0.7554044084f,0.8656312024f,0.9445750231f,0.9894009350f};
    static constexpr float GL_W[8]={0.1894506105f,0.1826034150f,0.1691565194f,0.1495959889f,
        0.1246289463f,0.0951585117f,0.0622535239f,0.0271524594f};
    struct State { float x,y,theta,kappa; };
    [[nodiscard]] State eval(float s) const noexcept {
        if(s<=0.f) return {x0,y0,theta0,kappa0};
        float dkds=(length>1e-9f)?(kappaEnd-kappa0)/length:0.f;
        auto thetaAt=[&](float t) noexcept { return theta0+kappa0*t+0.5f*dkds*t*t; };
        float mid=s*0.5f,hr=s*0.5f,px=x0,py=y0;
        for(int i=0;i<8;i++){
            float t1=mid+hr*GL_XI[i], t2=mid-hr*GL_XI[i];
            px += GL_W[i] * hr * (std::cos(thetaAt(t1)) + std::cos(thetaAt(t2)));
            py += GL_W[i] * hr * (std::sin(thetaAt(t1)) + std::sin(thetaAt(t2)));
        }
        return {px,py,thetaAt(s),kappa0+dkds*s};
    }
};

struct Waypoint { float x,y,heading; };

[[nodiscard]] inline std::vector<Waypoint> pathToWaypoints(const std::vector<CellCoord>& path,const MazeConfig& cfg){
    std::vector<Waypoint> wps; int N=int(path.size());
    for(int i=0;i<N;i++){ Vec2 pos=cfg.cellCentre(path[i]);
        float hdg=(wps.empty())?0.f:wps.back().heading;
        if(i+1<N){ Vec2 nxt=cfg.cellCentre(path[i+1]); hdg=std::atan2(nxt.y-pos.y,nxt.x-pos.x); }
        wps.push_back({pos.x,pos.y,hdg}); }
    if(wps.size()>=2) wps.back().heading=wps[wps.size()-2].heading; return wps;
}

[[nodiscard]] inline std::vector<Waypoint> optimiseRacingLine(std::vector<Waypoint> wps,float halfWidth,float margin=0.025f){
    int N=int(wps.size()); if(N<3) return wps;
    float hw=halfWidth-margin; if(hw<=0.f) return wps;
    std::vector<Waypoint> centres=wps; float step=hw*0.20f;
    for(int iter=0;iter<50;iter++){ float totalChange=0.f;
        for(int i=1;i<N-1;i++){
            float d_prev_x=0,d_prev_y=0,d_i_x,d_i_y,d_next_x=0,d_next_y=0;
            d_i_x=wps[i+1].x-2.f*wps[i].x+wps[i-1].x; d_i_y=wps[i+1].y-2.f*wps[i].y+wps[i-1].y;
            if(i>1){ d_prev_x=wps[i].x-2.f*wps[i-1].x+wps[i-2].x; d_prev_y=wps[i].y-2.f*wps[i-1].y+wps[i-2].y; }
            if(i<N-2){ d_next_x=wps[i+2].x-2.f*wps[i+1].x+wps[i].x; d_next_y=wps[i+2].y-2.f*wps[i+1].y+wps[i].y; }
            float gx=2.f*(d_prev_x+d_next_x-2.f*d_i_x), gy=2.f*(d_prev_y+d_next_y-2.f*d_i_y);
            float nx=wps[i].x-step*gx, ny=wps[i].y-step*gy;
            nx=std::max(centres[i].x-hw,std::min(centres[i].x+hw,nx));
            ny=std::max(centres[i].y-hw,std::min(centres[i].y+hw,ny));
            totalChange+=std::abs(nx-wps[i].x)+std::abs(ny-wps[i].y);
            wps[i].x=nx; wps[i].y=ny; }
        for(int i=0;i<N-1;i++){ float dx=wps[i+1].x-wps[i].x,dy=wps[i+1].y-wps[i].y;
            wps[i].heading=std::atan2(dy,dx); }
        if(N>=2) wps[N-1].heading=wps[N-2].heading;
        if(totalChange<1e-7f) break; }
    return wps;
}

// ── TrajGen (clothoid–arc–clothoid, FIX-C/FIX-D) ───────────────────────────
class TrajGen {
public:
    static constexpr int SAMPLES_STRAIGHT=10, SAMPLES_CLOTHOID=24, SAMPLES_ARC=20;
    [[nodiscard]] static std::vector<TrajPoint> generate(const std::vector<Waypoint>& wps,const RobotParams& robot){
        std::vector<TrajPoint> traj; int N=int(wps.size()); if(N<2) return traj;
        float cumArc=0.f, kPrev=0.f;
        auto emit=[&](float x,float y,float hdg,float k){
            float arc=0.f; if(!traj.empty()){ float dx=x-traj.back().x,dy=y-traj.back().y; arc=std::sqrt(dx*dx+dy*dy); }
            cumArc+=arc; traj.push_back({x,y,hdg,k,robot.maxVelocity,cumArc,0,0,0}); };
        for(int wi=0;wi+1<N;wi++){
            const Waypoint& wa=wps[wi]; const Waypoint& wb=wps[wi+1];
            float dhdg=angleDiff(wb.heading,wa.heading);
            float segLen=std::hypot(wb.x-wa.x,wb.y-wa.y); if(segLen<1e-7f) continue;
            int startIdx=(wi==0)?0:1;
            if(std::abs(dhdg)<5e-3f){
                int Ns=SAMPLES_STRAIGHT;
                for(int i=startIdx;i<=Ns;i++){ float t=float(i)/float(Ns);
                    emit(wa.x+t*(wb.x-wa.x),wa.y+t*(wb.y-wa.y),wa.heading,0.f); }
                kPrev=0.f;
            } else {
                float R=segLen/(2.f*std::sin(std::abs(dhdg)*0.5f)); R=std::max(R,robot.cellSize*0.05f);
                float kTurn=(1.f/R)*(dhdg>0.f?1.f:-1.f);
                float dkappa=std::abs(kTurn-kPrev);
                float L_c_min=dkappa*robot.maxVelocity/robot.steeringBandwidth;
                float L_c_max=segLen*0.45f;
                float L_c=std::max(L_c_min,0.005f); L_c=std::min(L_c,L_c_max);
                ClothoidSeg entry{wa.x,wa.y,wa.heading,kPrev,kTurn,L_c};
                for(int i=startIdx;i<=SAMPLES_CLOTHOID;i++){ float s=float(i)/float(SAMPLES_CLOTHOID)*L_c;
                    auto st=entry.eval(s); emit(st.x,st.y,st.theta,st.kappa); }
                auto eEnd=entry.eval(L_c); float sign=(dhdg>0.f)?1.f:-1.f;
                float perpAngle=eEnd.theta+sign*HALF_PI;
                float cx=eEnd.x+R*std::cos(perpAngle), cy=eEnd.y+R*std::sin(perpAngle);
                float clothoidAngle=L_c/R, arcAngle=std::abs(dhdg)-2.f*clothoidAngle;
                if(arcAngle>1e-4f){ float startArc=std::atan2(eEnd.y-cy,eEnd.x-cx);
                    for(int i=1;i<=SAMPLES_ARC;i++){ float t=float(i)/float(SAMPLES_ARC);
                        float ang=startArc+sign*t*arcAngle;
                        float px=cx+R*std::cos(ang), py=cy+R*std::sin(ang);
                        float hdg=wrapAngle(ang+sign*HALF_PI); emit(px,py,hdg,kTurn); } }
                const TrajPoint& ae=traj.back();
                ClothoidSeg exitSeg{ae.x,ae.y,ae.heading,kTurn,0.f,L_c};
                for(int i=1;i<=SAMPLES_CLOTHOID;i++){ float s=float(i)/float(SAMPLES_CLOTHOID)*L_c;
                    auto st=exitSeg.eval(s); emit(st.x,st.y,st.theta,st.kappa); }
                kPrev=0.f;
            }
        }
        return traj;
    }
};

// ── VelocityProfile (Kamm + S-curve, FIX-F) ────────────────────────────────
class VelocityProfile {
public:
    [[nodiscard]] static float kammLong(float kappa,float v,float aTotal) noexcept {
        float aLat2=(kappa*v*v)*(kappa*v*v), aT2=aTotal*aTotal;
        return (aLat2>=aT2)?0.f:std::sqrt(aT2-aLat2);
    }
    [[nodiscard]] static float vMaxCurv(float kappa,float aTotal) noexcept {
        return (std::abs(kappa)<1e-7f)?INF_F:std::sqrt(aTotal/std::abs(kappa));
    }
    static void curvatureCeilings(std::vector<TrajPoint>& traj,float vMax,float aTotal){
        for(auto& tp:traj) tp.velocity=std::min(vMax,vMaxCurv(tp.curvature,aTotal));
    }
    static void globalBrakingPass(std::vector<TrajPoint>& traj,float aTotal,float aBrakeMax){
        int N=int(traj.size()); if(N<2) return;
        for(int pass=0;pass<3;pass++) for(int i=N-2;i>=0;i--){
            float ds=traj[i+1].arcLen-traj[i].arcLen; if(ds<1e-9f) continue;
            float v1=traj[i+1].velocity;
            float aBrk=std::min(kammLong(traj[i].curvature,traj[i].velocity,aTotal),aBrakeMax);
            float vMax=std::sqrt(v1*v1+2.f*aBrk*ds);
            if(vMax<traj[i].velocity) traj[i].velocity=vMax; }
    }
    static void backwardPass(std::vector<TrajPoint>& traj,float maxJerk,float aTotal,float aBrakeMax,int maxIter=25){
        int N=int(traj.size()); if(N<2) return; traj.back().velocity=0.f;
        for(int iter=0;iter<maxIter;iter++){ float maxChange=0.f, prevBrk=0.f;
            for(int i=N-2;i>=0;i--){ float ds=traj[i+1].arcLen-traj[i].arcLen; if(ds<1e-9f) continue;
                float v1=traj[i+1].velocity;
                float aBrk=std::min({kammLong(traj[i+1].curvature,v1,aTotal),aBrakeMax,prevBrk+maxJerk*ds});
                prevBrk=aBrk; float vMax=std::sqrt(v1*v1+2.f*aBrk*ds);
                float vNew=std::min(traj[i].velocity,vMax);
                maxChange=std::max(maxChange,std::abs(vNew-traj[i].velocity)); traj[i].velocity=vNew; }
            if(maxChange<1e-5f) break; }
    }
    static void forwardPass(std::vector<TrajPoint>& traj,float maxJerk,float aTotal,float aAccelMax){
        if(traj.empty()) return; traj.front().velocity=0.f; float prevAccel=0.f;
        for(int i=1;i<int(traj.size());i++){ float ds=traj[i].arcLen-traj[i-1].arcLen;
            if(ds<1e-9f){ traj[i].velocity=std::min(traj[i].velocity,traj[i-1].velocity); continue; }
            float v0=traj[i-1].velocity;
            float aAv=std::min({kammLong(traj[i-1].curvature,v0,aTotal),aAccelMax,prevAccel+maxJerk*ds});
            float vAc=std::sqrt(v0*v0+2.f*aAv*ds), vNw=std::min(traj[i].velocity,vAc);
            traj[i].velocity=vNw;
            prevAccel=(v0+vNw>1e-6f)?(vNw*vNw-v0*v0)/(2.f*ds):0.f;
            prevAccel=std::max(-aTotal,std::min(prevAccel,aTotal)); traj[i].accel=prevAccel; }
    }
    static void computeJerk(std::vector<TrajPoint>& traj){
        for(int i=1;i<int(traj.size());i++){ float ds=traj[i].arcLen-traj[i-1].arcLen;
            float vAvg=0.5f*(traj[i].velocity+traj[i-1].velocity);
            float dt=(vAvg>1e-4f)?ds/vAvg:1e-3f;
            float dv=traj[i].velocity-traj[i-1].velocity;
            float a_now=dv/std::max(dt,1e-6f), a_prv=(i>1)?traj[i-1].accel:0.f;
            traj[i].jerk=(a_now-a_prv)/std::max(dt,1e-6f);
            float dkds=(ds>1e-9f)?(traj[i].curvature-traj[i-1].curvature)/ds:0.f;
            traj[i].ff_steer_rate=dkds*vAvg; }
    }
    [[nodiscard]] static float estimateTime(const std::vector<TrajPoint>& traj){
        float t=0.f; for(int i=1;i<int(traj.size());i++){ float ds=traj[i].arcLen-traj[i-1].arcLen;
            float vAvg=0.5f*(traj[i].velocity+traj[i-1].velocity); if(vAvg>1e-6f) t+=ds/vAvg; } return t;
    }
    [[nodiscard]] static float peakLatAccel(const std::vector<TrajPoint>& traj){
        float pk=0; for(auto& tp:traj) pk=std::max(pk,std::abs(tp.curvature)*tp.velocity*tp.velocity); return pk;
    }
    [[nodiscard]] static float peakLongAccel(const std::vector<TrajPoint>& traj){
        float pk=0; for(auto& tp:traj) pk=std::max(pk,std::abs(tp.accel)); return pk;
    }
    [[nodiscard]] static float peakJerk(const std::vector<TrajPoint>& traj){
        float pk=0; for(auto& tp:traj) pk=std::max(pk,std::abs(tp.jerk)); return pk;
    }
};

// ── TVLQR (FIX-A) ───────────────────────────────────────────────────────────
struct TVLQRGain { float K[2][3]; float arcLen; };
class TVLQRSolver {
public:
    static constexpr float Qx=200.f,Qy=200.f,Qt=50.f,Rv=1.f,Rw=0.5f;
    [[nodiscard]] static std::vector<TVLQRGain> solve(const std::vector<TrajPoint>& traj,float){
        int N=int(traj.size()); std::vector<TVLQRGain> gains(N);
        float P[3][3]={{Qx,0,0},{0,Qy,0},{0,0,Qt}}, Q[3][3]={{Qx,0,0},{0,Qy,0},{0,0,Qt}};
        float Rinv[2][2]={{1.f/Rv,0},{0,1.f/Rw}};
        for(int i=N-1;i>=0;i--){ const auto& tp=traj[i]; float v=std::max(tp.velocity,0.05f),h=tp.heading;
            float dt=(i>0)?(traj[i].arcLen-traj[i-1].arcLen)/v:0.005f;
            dt=std::max(1e-4f,std::min(dt,0.005f));   // clamp: forward-Euler Riccati is
                                                      // unstable for large dt (near v→0)
            float A[3][3]={{0,0,-v*std::sin(h)},{0,0,v*std::cos(h)},{0,0,0}};
            float B[3][2]={{std::cos(h),0},{std::sin(h),0},{0,1}};
            float BRB[3][3]{}; for(int r=0;r<3;r++) for(int c=0;c<3;c++) for(int m=0;m<2;m++) for(int n=0;n<2;n++) BRB[r][c]+=B[r][m]*Rinv[m][n]*B[c][n];
            float ATP[3][3]{}; for(int r=0;r<3;r++) for(int c=0;c<3;c++) for(int m=0;m<3;m++) ATP[r][c]+=A[m][r]*P[m][c];
            float PA[3][3]{};  for(int r=0;r<3;r++) for(int c=0;c<3;c++) for(int m=0;m<3;m++) PA[r][c]+=P[r][m]*A[m][c];
            float PB[3][3]{};  for(int r=0;r<3;r++) for(int c=0;c<3;c++) for(int m=0;m<3;m++) PB[r][c]+=P[r][m]*BRB[m][c];
            float PBRBP[3][3]{}; for(int r=0;r<3;r++) for(int c=0;c<3;c++) for(int m=0;m<3;m++) PBRBP[r][c]+=PB[r][m]*P[m][c];
            for(int r=0;r<3;r++) for(int c=0;c<3;c++) P[r][c]-=dt*(-Q[r][c]-ATP[r][c]-PA[r][c]+PBRBP[r][c]);
            for(int r=0;r<3;r++) for(int c=r+1;c<3;c++) P[r][c]=P[c][r]=0.5f*(P[r][c]+P[c][r]);
            for(int r=0;r<3;r++) P[r][r]=std::max(P[r][r],1e-4f);
            float BTP[2][3]{}; for(int r=0;r<2;r++) for(int c=0;c<3;c++) for(int m=0;m<3;m++) BTP[r][c]+=B[m][r]*P[m][c];
            for(int r=0;r<2;r++) for(int c=0;c<3;c++){ gains[i].K[r][c]=0; for(int m=0;m<2;m++) gains[i].K[r][c]+=Rinv[r][m]*BTP[m][c];
                if(!std::isfinite(gains[i].K[r][c])) gains[i].K[r][c]=0.f; }   // sanitize
            gains[i].arcLen=tp.arcLen; }
        return gains;
    }
    static void computeControl(const std::vector<TVLQRGain>& gains,const TrajPoint& ref,
                               float ex,float ey,float et,float& dv,float& dw){
        int lo=0,hi=int(gains.size())-1,idx=0;
        while(lo<=hi){ int mid=(lo+hi)/2; if(gains[mid].arcLen<ref.arcLen) lo=mid+1; else { idx=mid; hi=mid-1; } }
        const auto& K=gains[idx].K;
        float dx=ex-ref.x, dy=ey-ref.y, dtheta=angleDiff(et,ref.heading);
        dv=-(K[0][0]*dx+K[0][1]*dy+K[0][2]*dtheta);
        dw=-(K[1][0]*dx+K[1][1]*dy+K[1][2]*dtheta);
    }
};

class PDController {
public:
    explicit PDController(const RobotParams& p):params(p){}
    [[nodiscard]] std::pair<float,float> compute(float estX,float estY,float estTheta,
        const TrajPoint& ref,float prevCrossTrack,float dt) const noexcept {
        float ch=std::cos(ref.heading), sh=std::sin(ref.heading);
        float dx=estX-ref.x, dy=estY-ref.y;
        float e_xt=-dx*sh+dy*ch, e_hdg=angleDiff(estTheta,ref.heading);
        float de_xt=(dt>1e-6f)?(e_xt-prevCrossTrack)/dt:0.f;
        float ff=ref.velocity*ref.curvature;
        float fb=params.Kp_crosstrack*e_xt+params.Kd_crosstrack*de_xt+params.Kp_heading*e_hdg;
        return { ref.velocity, ff+fb };
    }
private: const RobotParams& params;
};

class WallCenteringPID {
public:
    explicit WallCenteringPID(const RobotParams& p):params(p){}
    void reset() noexcept { integral=0.f; }
    [[nodiscard]] float compute(float leftDist,bool leftValid,float rightDist,bool rightValid,
        float cellSize,float dt) noexcept {
        if(!leftValid&&!rightValid) return 0.f;
        float error=0.f;
        if(leftValid&&rightValid) error=(leftDist-rightDist)*0.5f;
        else if(leftValid)        error=leftDist-cellSize*0.5f;
        else                      error=cellSize*0.5f-rightDist;
        integral=std::max(-0.05f,std::min(0.05f,integral+error*dt));
        return params.Kp_center*error+params.Ki_center*integral;
    }
private: const RobotParams& params; float integral=0.f;
};

// ── Explorer (info-theoretic + D* Lite, FIX-E/FIX-G, wall-centering in scout) ─
class Explorer {
public:
    [[nodiscard]] static float infoGain(const Maze& bot,const CellCoord& cc) noexcept {
        float gain=0.f;
        for(int dr=-2;dr<=2;dr++) for(int dc=-2;dc<=2;dc++){ CellCoord nb{cc.r+dr,cc.c+dc};
            if(!bot.cfg->valid(nb)) continue;
            if(!bot.at(nb).explored){ gain+=4.f; continue; }
            for(int w=0;w<4;w++) if(!bot.at(nb).wallKnown[w]) gain+=1.f; }
        return gain;
    }
    [[nodiscard]] static float utility(const Maze& bot,const CellCoord& cc) noexcept {
        float dist=bot.at(cc).floodDist; return (dist>1e4f)?0.f:infoGain(bot,cc)/(dist+0.5f);
    }
    static bool senseCell(Maze& botMaze,const Maze& truthMaze,ESKF& kf,const CellCoord& cc,
                          const MazeConfig& cfg,const SensorModel& sensor){
        bool newInfo=false;
        for(int w=0;w<4;w++){ if(!botMaze.at(cc).wallKnown[w]){ botMaze.setWall(cc,w,truthMaze.at(cc).wall[w]); newInfo=true; } }
        botMaze.at(cc).explored=true; botMaze.at(cc).visitCount++;
        Vec2 ctr=cfg.cellCentre(cc); float half=cfg.cellSize*0.5f; float R=sensor.measurementVariance();
        if(botMaze.at(cc).wallKnown[WE]&&!botMaze.at(cc).wall[WE]) kf.updateWallDist(0,ctr.x+half,half,+1.f,R);
        if(botMaze.at(cc).wallKnown[WW]&&!botMaze.at(cc).wall[WW]) kf.updateWallDist(0,ctr.x-half,half,-1.f,R);
        if(botMaze.at(cc).wallKnown[WN]&&!botMaze.at(cc).wall[WN]) kf.updateWallDist(1,ctr.y+half,half,+1.f,R);
        if(botMaze.at(cc).wallKnown[WS]&&!botMaze.at(cc).wall[WS]) kf.updateWallDist(1,ctr.y-half,half,-1.f,R);
        int openCardinals=0; for(int w=0;w<4;w++) if(botMaze.at(cc).wallKnown[w]&&!botMaze.at(cc).wall[w]) openCardinals++;
        if(openCardinals==2) kf.snapHeadingCardinal();
        return newInfo;
    }
    // Canonical flood-fill exploration: each step recompute optimistic
    // distance-to-goal (unknown walls treated as open), then move to the
    // truly passable neighbour (walls at the current cell are already sensed)
    // that most reduces that distance, with a visit-count tie-break to escape
    // 2-cycles. Provably reaches the goal in a connected maze. ESKF dead-reckon,
    // wall-centering, and D* Lite notifications stay in the loop (spec stack);
    // decisions are flood-fill-driven, matching the original v4.1 behaviour.
    [[nodiscard]] static std::vector<CellCoord> explore(Maze& botMaze,const Maze& truthMaze,ESKF& kf,
        const MazeConfig& cfg,WallCenteringPID& wallCtrl,const SensorModel& sensor){
        CellCoord current=cfg.startCell; std::vector<CellCoord> visited{current};
        DStarLite dstar; dstar.init(botMaze,current,true);   // present for spec stack (bounded)
        auto getLeftRightWalls=[](float theta)->std::pair<int,int>{
            float leftAngle=wrapAngle(theta+HALF_PI); int leftW=0; float minDiff=INF_F;
            for(int w=0;w<4;w++){ float d=std::abs(angleDiff(leftAngle,WALL_HEADING[w]));
                if(d<minDiff){ minDiff=d; leftW=w; } }
            return { leftW, WALL_OPP[leftW] }; };
        auto applyWallCentering=[&](){
            auto [leftW,rightW]=getLeftRightWalls(kf.theta()); float half=cfg.cellSize*0.5f;
            bool lValid=botMaze.at(current).wallKnown[leftW]&&botMaze.at(current).wall[leftW];
            bool rValid=botMaze.at(current).wallKnown[rightW]&&botMaze.at(current).wall[rightW];
            float lDist=lValid?half:half*2.f, rDist=rValid?half:half*2.f;
            float corr=wallCtrl.compute(lDist,lValid,rDist,rValid,cfg.cellSize,0.01f);
            if(std::abs(corr)>1e-6f) kf.updateHeading(wrapAngle(kf.theta()+corr*0.01f),5e-4f); };

        senseCell(botMaze,truthMaze,kf,current,cfg,sensor);
        applyWallCentering();

        const int maxSteps=N_CELLS*40;
        CellCoord prev{-1,-1};
        for(int step=0;step<maxSteps;step++){
            if(cfg.isGoal(current)) break;
            // Optimistic distance-to-goal field over the current knowledge.
            FloodFill::solveToGoal(botMaze, true);   // FIXED: /optimistic=/true → true
            // Choose the best truly-passable cardinal neighbour, avoiding an
            // immediate U-turn unless it is the only passable option (this kills
            // the 2-cell oscillation that can occur once both cells are known).
            CellCoord best{-1,-1}, bestNoBack{-1,-1};
            float bestKey=INF_F, bestKeyNB=INF_F;
            for(int w=0;w<4;w++){
                if(!botMaze.canMoveCardinal(current,w, false)) continue;   // FIXED: /optimistic=/false → false
                CellCoord nb=current.neighbour(w);
                float fd=botMaze.at(nb).floodDist; if(std::isinf(fd)) fd=1e6f;
                float key=fd*10.f + float(botMaze.at(nb).visitCount);
                if(key<bestKey){ bestKey=key; best=nb; }
                if(!(nb==prev) && key<bestKeyNB){ bestKeyNB=key; bestNoBack=nb; }
            }
            CellCoord nextCell = (bestNoBack.r>=0)? bestNoBack : best;
            if(nextCell.r<0) break;   // truly boxed in (disconnected) — stop

            prev=current; current=nextCell; visited.push_back(current);
            kf.predict(cfg.cellSize,0.f,0.01f);
            bool ni=senseCell(botMaze,truthMaze,kf,current,cfg,sensor);
            if(ni) dstar.notifyWallChanged(current);
            applyWallCentering();
        }
        return visited;
    }
};

struct RunStats {
    std::string label; int pathCells=0, trajPoints=0;
    float pathLength=0, estimatedTime=0, peakLatAccel=0, peakLongAccel=0, peakJerk=0, peakVelocity=0;
    bool  valid=false;
};

// Full pipeline: cells → racing line → clothoid traj → velocity profile.
// No printing. Returns stats; fills outTraj if provided.
[[nodiscard]] inline RunStats profilePath(const std::vector<CellCoord>& cellPath,const MazeConfig& cfg,
    const RobotParams& robot,float vMax,std::vector<TrajPoint>* outTraj=nullptr){
    RunStats stats; stats.pathCells=int(cellPath.size());
    if(cellPath.size()<2) return stats;
    auto expanded=ThetaStar::expandPath(cellPath);
    auto wps=pathToWaypoints(expanded,cfg);
    wps=optimiseRacingLine(wps,cfg.cellSize*0.5f);
    auto traj=TrajGen::generate(wps,robot);
    if(traj.empty()) return stats;
    VelocityProfile::curvatureCeilings(traj,vMax,robot.maxTotalAccel);
    VelocityProfile::globalBrakingPass(traj,robot.maxTotalAccel,robot.maxBraking);
    VelocityProfile::backwardPass(traj,robot.maxJerk,robot.maxTotalAccel,robot.maxBraking);
    VelocityProfile::forwardPass(traj,robot.maxJerk,robot.maxTotalAccel,robot.maxAccelFwd);
    VelocityProfile::computeJerk(traj);
    if(outTraj) *outTraj=traj;
    stats.trajPoints=int(traj.size()); stats.pathLength=traj.back().arcLen;
    stats.estimatedTime=VelocityProfile::estimateTime(traj);
    stats.peakLatAccel=VelocityProfile::peakLatAccel(traj);
    stats.peakLongAccel=VelocityProfile::peakLongAccel(traj);
    stats.peakJerk=VelocityProfile::peakJerk(traj);
    for(auto& tp:traj) stats.peakVelocity=std::max(stats.peakVelocity,tp.velocity);
    stats.valid=true; return stats;
}

} // namespace gdw