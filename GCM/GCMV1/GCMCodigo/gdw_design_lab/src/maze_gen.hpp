// ═══════════════════════════════════════════════════════════════════════════
//  maze_gen.hpp — ground-truth maze generation
//
//  generateRandomMaze(): carves a fully-walled maze with a randomized DFS
//  (recursive backtracker) so every cell is reachable, then guarantees the
//  centre 2×2 goal is open internally and connected. Also a few extra openings
//  are punched to create multiple routes (a perfect maze has exactly one path;
//  real contest mazes have loops, which is where racing-line choices matter).
//
//  loadClassicMaze(): a deterministic structured maze for reproducible runs.
//
//  A "truth" Maze starts with all internal walls present; we remove walls to
//  carve passages (Maze::setWall(.,.,false)).
// ═══════════════════════════════════════════════════════════════════════════
#pragma once
#include "gdw_core.hpp"
#include <random>
#include <stack>
#include <vector>

namespace gdw {

inline void fillAllInternalWalls(Maze& m) {
    const int sz = m.cfg->size;
    for (int r=0;r<sz;r++) for (int c=0;c<sz;c++) {
        CellCoord cc{r,c};
        for (int w=0;w<4;w++) m.setWall(cc, w, true);
    }
}

// Randomized DFS carve. seed makes each maze reproducible.
inline void generateRandomMaze(Maze& truth, const MazeConfig& cfg, uint32_t seed) {
    truth.init(cfg);
    fillAllInternalWalls(truth);
    std::mt19937 rng(seed);

    std::vector<char> visited(N_CELLS, 0);
    std::stack<CellCoord> st;
    CellCoord start = cfg.startCell;
    st.push(start);
    visited[start.idx()] = 1;

    auto carve = [&](const CellCoord& a, int w) {
        truth.setWall(a, w, false);   // also clears neighbour's opposite wall
    };

    while (!st.empty()) {
        CellCoord cur = st.top();
        int order[4] = {WN,WE,WS,WW};
        for (int i=3;i>0;i--) std::swap(order[i], order[std::uniform_int_distribution<int>(0,i)(rng)]);
        bool advanced = false;
        for (int k=0;k<4;k++) {
            int w = order[k];
            CellCoord nb = cur.neighbour(w);
            if (!cfg.valid(nb) || visited[nb.idx()]) continue;
            carve(cur, w);
            visited[nb.idx()] = 1;
            st.push(nb);
            advanced = true;
            break;
        }
        if (!advanced) st.pop();
    }

    // Open the centre 2×2 goal region internally so the mouse can sit in it.
    CellCoord g0 = cfg.goalCells[0];
    int gr = std::min({cfg.goalCells[0].r,cfg.goalCells[1].r,cfg.goalCells[2].r,cfg.goalCells[3].r});
    int gc = std::min({cfg.goalCells[0].c,cfg.goalCells[1].c,cfg.goalCells[2].c,cfg.goalCells[3].c});
    truth.setWall({gr,gc},     WE, false);
    truth.setWall({gr,gc},     WS, false);
    truth.setWall({gr,gc+1},   WS, false);
    truth.setWall({gr+1,gc},   WE, false);
    (void)g0;

    // Punch a handful of extra openings to create loops/alternate routes.
    std::uniform_int_distribution<int> rd(1, cfg.size-2), wd(0,3);
    int extra = cfg.size;                 // ~16 extra openings
    for (int i=0;i<extra;i++) {
        CellCoord cc{ rd(rng), rd(rng) };
        int w = wd(rng);
        CellCoord nb = cc.neighbour(w);
        if (cfg.valid(nb)) truth.setWall(cc, w, false);
    }

    // Ensure the goal is reachable from start under the truth map. If the
    // extra openings somehow isolated it (extremely unlikely), carve a corridor.
    {
        Maze probe = truth;
        for (auto& cell : probe.cells) { // make all known so canMoveCons works
            cell.wallKnown = {true,true,true,true};
        }
        FloodFill::solveToGoal(probe, false);   // FIXED: /optimistic=/false → false
        if (std::isinf(probe.at(cfg.startCell).floodDist)) {
            // Fallback: straight corridor up the first column then across.
            for (int r=cfg.size-1;r>gr;r--) truth.setWall({r,0}, WN, false);
            for (int c=0;c<gc;c++)          truth.setWall({gr,c}, WE, false);
        }
    }
}

// Deterministic structured maze for reproducible single-run testing.
inline void loadClassicMaze(Maze& truth, const MazeConfig& cfg) {
    generateRandomMaze(truth, cfg, 0xC1A551Cu);
}

} // namespace gdw