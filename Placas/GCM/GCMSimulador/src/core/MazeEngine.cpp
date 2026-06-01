// ═══════════════════════════════════════════════════════════════════════════
//  src/core/MazeEngine.cpp
// ═══════════════════════════════════════════════════════════════════════════
#include "MazeEngine.h"
#include <cassert>
#include <algorithm>
#include <stack>

// ══════════════════════════════════════════════════════════════════════════
//  Maze
// ══════════════════════════════════════════════════════════════════════════

void Maze::init(const MazeConfig& c)
{
    cfg = &c;
    cells.fill(Cell{});
    placeBorderWalls();
}

void Maze::reset()
{
    for (auto& cell : cells) {
        cell.wallKnown.fill(false);
        cell.explored   = false;
        cell.visitCount = 0;
        cell.floodDist  = INF_F;
        cell.dstar_g    = INF_F;
        cell.dstar_rhs  = INF_F;
        cell.colorHeat  = 0;
    }
    // Re-expose border walls (they are always known)
    placeBorderWalls();
}

Cell&       Maze::at(const CellCoord& cc)       noexcept { return cells[cc.idx()]; }
const Cell& Maze::at(const CellCoord& cc) const noexcept { return cells[cc.idx()]; }
Cell&       Maze::at(int r, int c)              noexcept { return cells[r*MAZE_N+c]; }
const Cell& Maze::at(int r, int c)        const noexcept { return cells[r*MAZE_N+c]; }

void Maze::setWall(const CellCoord& cc, int w, bool present)
{
    if (!cfg->valid(cc)) return;
    cells[cc.idx()].wall[w]      = present;
    cells[cc.idx()].wallKnown[w] = true;
    CellCoord nb = cc.neighbour(w);
    if (cfg->valid(nb)) {
        cells[nb.idx()].wall[WALL_OPP[w]]      = present;
        cells[nb.idx()].wallKnown[WALL_OPP[w]] = true;
    }
}

void Maze::setWallKnown(const CellCoord& cc, int w, bool known)
{
    if (cfg->valid(cc)) cells[cc.idx()].wallKnown[w] = known;
}

bool Maze::canMove8(const CellCoord& cc, int d8, bool optimistic) const noexcept
{
    CellCoord nb = cc.step8(d8);
    if (!cfg->valid(nb)) return false;
    const Cell& cell = at(cc);
    for (int k=0; k<2; k++) {
        int w = D8WALLS[d8][k];
        if (w < 0) continue;
        if (optimistic) { if (cell.wallKnown[w] && cell.wall[w]) return false; }
        else            { if (!cell.wallKnown[w] || cell.wall[w]) return false; }
    }
    return true;
}

bool Maze::canMoveCardinal(const CellCoord& cc, int w, bool optimistic) const noexcept
{
    CellCoord nb = cc.neighbour(w);
    if (!cfg->valid(nb)) return false;
    const Cell& cell = at(cc);
    if (optimistic) return !(cell.wallKnown[w] && cell.wall[w]);
    return cell.wallKnown[w] && !cell.wall[w];
}

int Maze::frontierCount() const noexcept
{
    int n=0;
    for (auto& c : cells) if (c.explored && c.hasFrontier()) n++;
    return n;
}

int Maze::exploredCount() const noexcept
{
    int n=0;
    for (auto& c : cells) if (c.explored) n++;
    return n;
}

bool Maze::fullyExplored() const noexcept
{
    for (auto& c : cells) if (!c.explored) return false;
    return true;
}

float Maze::trueWallDistance(const CellCoord& cc, int w) const noexcept
{
    // Distance from cell centre to given wall
    return cfg->cellSize * 0.5f;  // always half-cell for cardinal walls
}

void Maze::placeBorderWalls()
{
    const int sz = cfg->size;
    for (int i=0; i<sz; i++) {
        setWall({0,    i}, WN, true);
        setWall({sz-1, i}, WS, true);
        setWall({i,    0}, WW, true);
        setWall({i, sz-1}, WE, true);
    }
}

// ══════════════════════════════════════════════════════════════════════════
//  FloodFill
// ══════════════════════════════════════════════════════════════════════════

void FloodFill::solve(Maze& maze,
                      const std::vector<CellCoord>& seeds,
                      bool optimistic)
{
    for (auto& cell : maze.cells) cell.floodDist = INF_F;

    using Entry = std::pair<float, CellCoord>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;

    for (const auto& s : seeds) {
        if (!maze.cfg->valid(s)) continue;
        maze.at(s).floodDist = 0.f;
        pq.push({0.f, s});
    }

    while (!pq.empty()) {
        auto [d, cc] = pq.top(); pq.pop();
        if (d > maze.at(cc).floodDist + 1e-6f) continue;
        for (int d8=0; d8<8; d8++) {
            if (!maze.canMove8(cc, d8, optimistic)) continue;
            CellCoord nb = cc.step8(d8);
            float nd = d + D8COST[d8];
            if (nd < maze.at(nb).floodDist - 1e-6f) {
                maze.at(nb).floodDist = nd;
                pq.push({nd, nb});
            }
        }
    }
}

void FloodFill::solveToGoal(Maze& maze, bool optimistic)
{
    std::vector<CellCoord> seeds(maze.cfg->goalCells.begin(),
                                  maze.cfg->goalCells.end());
    solve(maze, seeds, optimistic);
}

void FloodFill::solveToStart(Maze& maze, bool optimistic)
{
    solve(maze, {maze.cfg->startCell}, optimistic);
}

// ══════════════════════════════════════════════════════════════════════════
//  DStarLite
// ══════════════════════════════════════════════════════════════════════════

float DStarLite::heuristic(const CellCoord& cc) const noexcept
{
    float best = INF_F;
    for (const auto& gc : maze->cfg->goalCells) {
        float dr = float(std::abs(cc.r - gc.r));
        float dc = float(std::abs(cc.c - gc.c));
        float h  = std::max(dr,dc) + (SQRT2-1.f)*std::min(dr,dc);
        if (h < best) best = h;
    }
    return best;
}

DStarLite::Key DStarLite::calcKey(const CellCoord& cc) const noexcept
{
    float gv = maze->at(cc).dstar_g;
    float rv = maze->at(cc).dstar_rhs;
    float mn = std::min(gv, rv);
    return {mn + heuristic(cc) + km, mn};
}

void DStarLite::init(Maze& m, CellCoord s, bool opt)
{
    maze       = &m;
    start      = s;
    optimistic = opt;
    km         = 0.f;

    for (auto& cell : maze->cells) {
        cell.dstar_g   = INF_F;
        cell.dstar_rhs = INF_F;
    }
    while (!U.empty()) U.pop();

    for (const auto& gc : maze->cfg->goalCells) {
        maze->at(gc).dstar_rhs = 0.f;
        U.push({calcKey(gc), gc});
    }
    computeShortestPath();
}

void DStarLite::computeShortestPath()
{
    while (!U.empty()) {
        auto [kOld, u] = U.top();
        Key ks = calcKey(start);
        float gs = maze->at(start).dstar_g;
        float rs = maze->at(start).dstar_rhs;
        if (!(kOld <= ks) && std::abs(rs - gs) < 1e-6f) break;
        U.pop();

        Key kNew = calcKey(u);
        if (kOld < kNew) { U.push({kNew, u}); continue; }

        float gu = maze->at(u).dstar_g;
        float ru = maze->at(u).dstar_rhs;

        if (gu > ru) {
            maze->at(u).dstar_g = ru;
            for (int d8=0; d8<8; d8++) {
                if (!maze->canMove8(u, d8, optimistic)) continue;
                CellCoord nb = u.step8(d8);
                if (maze->cfg->isGoal(nb)) continue;
                float nr = maze->at(u).dstar_g + D8COST[d8];
                if (nr < maze->at(nb).dstar_rhs - 1e-6f) {
                    maze->at(nb).dstar_rhs = nr;
                    U.push({calcKey(nb), nb});
                }
            }
        } else {
            maze->at(u).dstar_g = INF_F;
            U.push({calcKey(u), u});
            for (int d8=0; d8<8; d8++) {
                if (!maze->canMove8(u, d8, optimistic)) continue;
                CellCoord nb = u.step8(d8);
                if (maze->cfg->isGoal(nb)) continue;
                float bestRhs = INF_F;
                for (int d2=0; d2<8; d2++) {
                    if (!maze->canMove8(nb, d2, optimistic)) continue;
                    CellCoord s2 = nb.step8(d2);
                    float cand = maze->at(s2).dstar_g + D8COST[d2];
                    if (cand < bestRhs) bestRhs = cand;
                }
                maze->at(nb).dstar_rhs = bestRhs;
                U.push({calcKey(nb), nb});
            }
        }
    }
    for (auto& cell : maze->cells)
        cell.floodDist = cell.dstar_g;
}

void DStarLite::notifyWallChanged(const CellCoord& cc)
{
    km += heuristic(start);
    for (int d8=0; d8<8; d8++) {
        CellCoord nb = cc.step8(d8);
        if (!maze->cfg->valid(nb) || maze->cfg->isGoal(nb)) continue;
        float bestRhs = INF_F;
        for (int d2=0; d2<8; d2++) {
            if (!maze->canMove8(nb, d2, optimistic)) continue;
            CellCoord s2 = nb.step8(d2);
            float cand = maze->at(s2).dstar_g + D8COST[d2];
            if (cand < bestRhs) bestRhs = cand;
        }
        maze->at(nb).dstar_rhs = bestRhs;
        U.push({calcKey(nb), nb});
    }
    computeShortestPath();
}

// ══════════════════════════════════════════════════════════════════════════
//  ThetaStar
// ══════════════════════════════════════════════════════════════════════════

bool ThetaStar::checkWall(const Maze& m, const CellCoord& cc,
                            int w, bool optimistic) noexcept
{
    if (!m.cfg->valid(cc)) return false;
    const Cell& cell = m.at(cc);
    if (optimistic) return !(cell.wallKnown[w] && cell.wall[w]);
    return cell.wallKnown[w] && !cell.wall[w];
}

bool ThetaStar::lineOfSight(const Maze& maze,
                              const CellCoord& a,
                              const CellCoord& b,
                              bool optimistic) noexcept
{
    int r0=a.r, c0=a.c, r1=b.r, c1=b.c;
    int dr=std::abs(r1-r0), dc=std::abs(c1-c0);
    int sr=(r1>r0)?1:-1, sc=(c1>c0)?1:-1;
    int r=r0, c=c0, err=dc-dr;

    for (int step=0; step<=dr+dc; step++) {
        if (r==r1&&c==c1) return true;
        if (!maze.cfg->valid(r,c)) return false;
        int e2=2*err;
        bool mC=(e2>-dr), mR=(e2<dc);
        CellCoord cc2{r,c};
        if (mC&&mR) {
            int wC=(sc>0)?WE:WW, wR=(sr>0)?WS:WN;
            if (!checkWall(maze,cc2,wC,optimistic)) return false;
            if (!checkWall(maze,cc2,wR,optimistic)) return false;
            c+=sc; r+=sr; err+=dr-dc;
        } else if (mC) {
            if (!checkWall(maze,cc2,(sc>0)?WE:WW,optimistic)) return false;
            c+=sc; err+=dr;
        } else {
            if (!checkWall(maze,cc2,(sr>0)?WS:WN,optimistic)) return false;
            r+=sr; err-=dc;
        }
    }
    return true;
}

float ThetaStar::dist(const CellCoord& a, const CellCoord& b) noexcept
{
    float dr=float(b.r-a.r), dc=float(b.c-a.c);
    return std::sqrt(dr*dr+dc*dc);
}

std::vector<CellCoord> ThetaStar::findPath(
    const Maze& maze, const CellCoord& start, bool optimistic)
{
    std::array<float,     N_CELLS> gCost;
    std::array<CellCoord, N_CELLS> parent;
    std::array<bool,      N_CELLS> closed;
    gCost.fill(INF_F);
    parent.fill({-1,-1});
    closed.fill(false);

    gCost[start.idx()]  = 0.f;
    parent[start.idx()] = {-2,-2};  // start sentinel

    struct Node { float f; CellCoord cc; bool operator>(const Node& o) const noexcept { return f>o.f; } };
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
    open.push({maze.at(start).floodDist, start});

    CellCoord reached{-1,-1};

    while (!open.empty()) {
        auto [f, cc] = open.top(); open.pop();
        if (closed[cc.idx()]) continue;
        closed[cc.idx()] = true;
        if (maze.cfg->isGoal(cc)) { reached=cc; break; }

        for (int d8=0; d8<8; d8++) {
            if (!maze.canMove8(cc, d8, optimistic)) continue;
            CellCoord nb = cc.step8(d8);
            if (closed[nb.idx()]) continue;

            const CellCoord& par = parent[cc.idx()];
            bool hasPar = (par.r >= 0);

            float ng; CellCoord via;
            if (hasPar && lineOfSight(maze, par, nb, optimistic)) {
                ng  = gCost[par.idx()] + dist(par, nb);
                via = par;
            } else {
                ng  = gCost[cc.idx()] + D8COST[d8];
                via = cc;
            }

            if (ng < gCost[nb.idx()] - 1e-6f) {
                gCost[nb.idx()]  = ng;
                parent[nb.idx()] = via;
                open.push({ng + maze.at(nb).floodDist, nb});
            }
        }
    }

    if (reached.r < 0) return {};

    std::vector<CellCoord> path;
    CellCoord c = reached;
    while (c.r >= 0) {
        path.push_back(c);
        const CellCoord& p = parent[c.idx()];
        if (p.r == -2) break;
        c = p;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<CellCoord> ThetaStar::expandPath(const std::vector<CellCoord>& path)
{
    std::vector<CellCoord> expanded;
    if (path.empty()) return expanded;
    expanded.push_back(path[0]);
    for (size_t i=1; i<path.size(); i++) {
        int dr=path[i].r-path[i-1].r;
        int dc=path[i].c-path[i-1].c;
        int steps=std::max(std::abs(dr),std::abs(dc));
        int sr=(dr>0)-(dr<0), sc=(dc>0)-(dc<0);
        for (int s=1; s<=steps; s++)
            expanded.push_back({path[i-1].r+s*sr, path[i-1].c+s*sc});
    }
    return expanded;
}

// ══════════════════════════════════════════════════════════════════════════
//  MazeGenerator
// ══════════════════════════════════════════════════════════════════════════

Maze MazeGenerator::generate(std::mt19937& rng, const MazeConfig& cfg)
{
    Maze maze;
    maze.init(cfg);
    // Start fully walled
    for (int r=0; r<cfg.size; r++)
        for (int c=0; c<cfg.size; c++)
            for (int w=0; w<4; w++)
                maze.at(r,c).wall[w] = true;
    maze.init(cfg); // re-sets border walls

    carve(maze, cfg, rng);
    ensureGoalAccess(maze, cfg, rng);

    // Mark all walls as known (this is the truth maze)
    for (auto& cell : maze.cells)
        cell.wallKnown.fill(true);

    return maze;
}

void MazeGenerator::carve(Maze& maze, const MazeConfig& cfg, std::mt19937& rng)
{
    // Recursive backtracker (depth-first)
    std::vector<bool> visited(N_CELLS, false);
    std::stack<CellCoord> stack;

    CellCoord start = cfg.startCell;
    visited[start.idx()] = true;
    stack.push(start);

    while (!stack.empty()) {
        CellCoord cur = stack.top();

        // Build list of unvisited cardinal neighbours
        std::vector<std::pair<int,CellCoord>> nbrs;
        for (int w=0; w<4; w++) {
            CellCoord nb = cur.neighbour(w);
            if (cfg.valid(nb) && !visited[nb.idx()])
                nbrs.push_back({w, nb});
        }

        if (nbrs.empty()) {
            stack.pop();
            continue;
        }

        // Pick random unvisited neighbour
        std::uniform_int_distribution<int> pick(0, int(nbrs.size())-1);
        auto [w, nb] = nbrs[pick(rng)];
        // Remove wall between cur and nb
        maze.setWall(cur, w, false);
        visited[nb.idx()] = true;
        stack.push(nb);
    }

    // Add random extra passages for multiple paths (30% of walls)
    std::uniform_real_distribution<float> u(0.f, 1.f);
    for (int r=1; r<cfg.size-1; r++) {
        for (int c=1; c<cfg.size-1; c++) {
            CellCoord cc{r,c};
            for (int w : {WE, WS}) {
                CellCoord nb = cc.neighbour(w);
                if (!cfg.valid(nb)) continue;
                if (maze.at(cc).wall[w] && u(rng) < 0.12f)
                    maze.setWall(cc, w, false);
            }
        }
    }
}

void MazeGenerator::ensureGoalAccess(Maze& maze, const MazeConfig& cfg, std::mt19937& rng)
{
    // Ensure at least one open passage into each goal cell
    for (const auto& gc : cfg.goalCells) {
        bool hasAccess = false;
        for (int w=0; w<4; w++) {
            CellCoord nb = gc.neighbour(w);
            if (cfg.valid(nb) && !maze.at(gc).wall[w]) { hasAccess=true; break; }
        }
        if (!hasAccess) {
            // Open a random wall (not border)
            std::vector<int> candidates;
            for (int w=0; w<4; w++) {
                CellCoord nb = gc.neighbour(w);
                if (cfg.valid(nb)) candidates.push_back(w);
            }
            if (!candidates.empty()) {
                std::uniform_int_distribution<int> pick(0,int(candidates.size())-1);
                maze.setWall(gc, candidates[pick(rng)], false);
            }
        }
    }
    // Open a few walls between goal cells themselves for a clean open area
    maze.setWall(cfg.goalCells[0], WE, false);
    maze.setWall(cfg.goalCells[0], WS, false);
    maze.setWall(cfg.goalCells[1], WS, false);
    maze.setWall(cfg.goalCells[2], WE, false);
}

Maze MazeGenerator::officialGDW(const MazeConfig& cfg)
{
    Maze truth;
    truth.init(cfg);

    // Internal walls from GDW v4.1 reference (from original single-file code)
    using T3 = std::tuple<int,int,int>;
    for (auto& [r,c,w] : std::vector<T3>{
        {15,1,WS},{14,1,WS},{13,2,WE},{12,1,WE},{12,2,WS},
        {10,5,WS},{10,1,WE},{ 9,2,WN},{ 8,0,WS},{ 8,1,WE},
        { 7,2,WW},{ 6,1,WS},{ 6,2,WE},{ 4,11,WN},{ 5,10,WE},
        { 3,12,WW},{ 9,6,WS},{11,4,WE},{13,8,WS},{ 7,2,WE},
        { 5,10,WN},{ 2,13,WW},{ 9,10,WS},{ 3,12,WE},{ 6,9,WN},
        {10,14,WE},{11,4,WS},{ 1,14,WN},{ 2,13,WW},{13,2,WS},
        { 8,7,WN},{ 7,5,WE},{ 6,6,WS},{ 5,7,WW},{ 4,8,WN},
        {11,4,WE},{10,5,WN},{ 9,6,WE},{ 8,7,WS},
        { 3,12,WS},{ 2,13,WE},{ 1,14,WN}
    }) truth.setWall({r,c}, w, true);

    for (auto& cell : truth.cells) cell.wallKnown.fill(true);
    return truth;
}

void MazeGenerator::copyStructure(const Maze& src, Maze& dst)
{
    for (int i=0; i<N_CELLS; i++) {
        dst.cells[i].wall      = src.cells[i].wall;
        dst.cells[i].wallKnown.fill(true);
    }
}
