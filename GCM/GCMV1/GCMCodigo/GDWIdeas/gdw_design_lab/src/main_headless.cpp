// ═══════════════════════════════════════════════════════════════════════════
//  main_headless.cpp — GDW Design Lab (headless optimizer)
//
//  Usage:
//    gdw_design_lab [--budget N] [--mazes M] [--pop P] [--gens G]
//                   [--no-tvlqr] [--seed S] [--out DIR]
//
//  --budget N   : approximate number of design evaluations (100/1000/10000…).
//                 With pop P and mazes M, gens ≈ N / (P·M) unless --gens given.
//  --mazes  M   : mazes averaged per fitness eval (raise to 100+ to avoid
//                 overfitting; default kept small for a quick demo run).
//
//  Produces in --out: best_design.json, best_design.spec.txt, hall_of_fame.csv,
//  best_design.cad.txt, convergence.csv
// ═══════════════════════════════════════════════════════════════════════════
#include "genetic.hpp"
#include "export.hpp"
#include <chrono>
#include <cstring>
#include <iostream>
#include <string>

using namespace gdw;

struct Args {
    int budget = 1000;
    int mazes  = -1;       // if <0, auto from defaults
    int pop    = -1;
    int gens   = -1;
    bool tvlqr = true;
    uint32_t seed = 12345u;
    std::string out = ".";
};

static Args parse(int argc, char** argv) {
    Args a;
    for (int i=1;i<argc;i++) {
        std::string s = argv[i];
        auto next=[&](int& iref)->std::string{ return (iref+1<argc)?argv[++iref]:std::string(); };
        if      (s=="--budget") a.budget = std::stoi(next(i));
        else if (s=="--mazes")  a.mazes  = std::stoi(next(i));
        else if (s=="--pop")    a.pop    = std::stoi(next(i));
        else if (s=="--gens")   a.gens   = std::stoi(next(i));
        else if (s=="--seed")   a.seed   = uint32_t(std::stoul(next(i)));
        else if (s=="--out")    a.out    = next(i);
        else if (s=="--no-tvlqr") a.tvlqr=false;
        else if (s=="--help") { std::cout<<"see header of main_headless.cpp\n"; std::exit(0); }
    }
    return a;
}

int main(int argc, char** argv) {
    Args args = parse(argc, argv);

    GA ga(args.seed);
    ga.useTVLQR = args.tvlqr;
    if (args.pop   > 0) ga.populationSize = args.pop;
    if (args.mazes > 0) ga.mazesPerEval   = args.mazes;

    // gens from budget if not explicitly given
    int evalsPerGen = ga.populationSize * ga.mazesPerEval;
    int gens = (args.gens>0) ? args.gens
                             : std::max(1, args.budget / std::max(1, ga.populationSize));
    long totalDesignEvals = long(gens) * ga.populationSize;
    long totalSims        = totalDesignEvals * ga.mazesPerEval;

    std::cout << "GDW Design Lab — headless optimizer\n";
    std::cout << "-----------------------------------\n";
    std::cout << "population        : " << ga.populationSize << "\n";
    std::cout << "mazes per eval    : " << ga.mazesPerEval   << "\n";
    std::cout << "generations       : " << gens              << "\n";
    std::cout << "TVLQR             : " << (ga.useTVLQR?"on":"off") << "\n";
    std::cout << "design evaluations: " << totalDesignEvals  << "\n";
    std::cout << "total maze sims   : " << totalSims         << "\n";
    std::cout << "(evals/gen=" << evalsPerGen << ")\n\n";

    auto mazeSeeds = makeMazeSeeds(ga.mazesPerEval, args.seed);
    auto pop = ga.initialPopulation();

    std::ofstream conv(args.out + "/convergence.csv");
    conv << "generation,best_fitness,best_exec_time_s,best_peak_v,best_reliability,"
            "best_slip,best_collisions\n";

    Individual best; best.fitness = INF_F;
    auto t0 = std::chrono::steady_clock::now();

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "gen |  best fit | exec(s) | peakV | reli | slip  | coll | mu   | fan(W) | aLat | vMax\n";
    std::cout << "----+-----------+---------+-------+------+-------+------+------+--------+------+-----\n";

    for (int g=0; g<gens; ++g) {
        ga.step(pop, mazeSeeds);              // evaluates + produces next gen
        // After step(), pop is the NEW generation (unevaluated). The elites at
        // the front carry their evaluated fitness; pop[0] is the current best.
        const Individual& b = pop.front();
        if (b.fitness < best.fitness) best = b;

        PhysicsModel pm = physics::derive(b.design);
        std::cout << std::setw(3) << g << " | "
                  << std::setw(9) << b.fitness << " | "
                  << std::setw(7) << b.meanExecTime << " | "
                  << std::setw(5) << b.meanPeakV << " | "
                  << std::setw(4) << b.reliability << " | "
                  << std::setw(5) << b.meanSlip << " | "
                  << std::setw(4) << b.meanCollisions << " | "
                  << std::setw(4) << b.design.tireFriction << " | "
                  << std::setw(6) << b.design.fanPower_W << " | "
                  << std::setw(4) << pm.aLatMax << " | "
                  << std::setw(4) << pm.vMax << "\n";

        conv << g << "," << b.fitness << "," << b.meanExecTime << "," << b.meanPeakV
             << "," << b.reliability << "," << b.meanSlip << "," << b.meanCollisions << "\n";
    }

    // Final evaluation pass on the last population to lock in best metrics.
    ga.evaluatePopulationParallel(pop, mazeSeeds);
    std::sort(pop.begin(), pop.end(),
              [](const Individual& a,const Individual& c){ return a.fitness<c.fitness; });
    if (pop.front().fitness < best.fitness) best = pop.front();

    auto t1 = std::chrono::steady_clock::now();
    double secs = std::chrono::duration<double>(t1-t0).count();

    std::cout << "\n" << specReport(best) << "\n";
    std::cout << "optimization wall time: " << secs << " s\n";

    // Hall of fame = top 10 of final population.
    std::vector<Individual> hof(pop.begin(), pop.begin()+std::min<size_t>(10,pop.size()));
    writeJSON(args.out + "/best_design.json", best);
    writeSpec(args.out + "/best_design.spec.txt", best);
    writeCSV (args.out + "/hall_of_fame.csv", hof);
    auto cad = makeCADExporter("parametric");
    cad->exportDesign(best.design, args.out + "/best_design.cad.txt");

    std::cout << "\nwrote: best_design.json, best_design.spec.txt, hall_of_fame.csv,\n"
                 "       best_design.cad.txt, convergence.csv  (in " << args.out << ")\n";
    return 0;
}