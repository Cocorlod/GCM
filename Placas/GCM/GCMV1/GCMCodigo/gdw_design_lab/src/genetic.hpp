// ═══════════════════════════════════════════════════════════════════════════
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
#include "evaluator.hpp"
#include <algorithm>
#include <future>
#include <numeric>
#include <random>
#include <vector>

namespace gdw {

// ── Fitness weights (penalty coefficients) ─────────────────────────────────
struct FitnessWeights {
    float wSpeed      = 1.0f;     // s  — PRIMARY objective (minimize time)
    float wCollision  = 1.5f;     // per collision (s-equivalent)
    float wSlip       = 3.0f;     // per unit slip ratio
    float wJerk       = 0.00008f; // per m/s³ peak jerk — light instability regularizer
                                  // (peak jerk reaches 1e3–1e4 at clothoid joins; a
                                  //  larger weight would dominate time and reward
                                  //  pointlessly slow designs)
    float wTrack      = 5.0f;     // per metre mean lateral deviation
    float wEnergy     = 0.0015f;  // per J — light efficiency regularizer
    float dnfPenalty  = 50.0f;    // s, per maze the design fails to complete
};

struct Individual {
    RobotDesign design;
    float fitness = INF_F;
    // aggregated metrics (means over mazes)
    float meanSpeedTime=0, meanExecTime=0, meanSlip=0, meanTrack=0, meanLoc=0,
          meanEnergy=0, meanJerk=0, meanPeakV=0;
    float reliability=0;          // fraction of mazes completed w/o "DNF"
    float meanCollisions=0;
};

class GA {
public:
    DesignBounds  bounds;
    FitnessWeights weights;
    int    populationSize = 48;
    int    eliteCount     = 4;
    float  mutationRate   = 0.25f;
    float  mutationSigma  = 0.15f;   // fraction of gene range
    float  crossoverAlpha = 0.5f;    // BLX-α
    int    mazesPerEval   = 8;       // <-- raise to 100+ for the final run
    bool   useTVLQR       = true;

    explicit GA(uint32_t seed=12345u) : rng(seed) {}

    // ── genome <-> design ────────────────────────────────────────────────
    static constexpr int GENES = 17;
    [[nodiscard]] std::array<float,GENES> encode(const RobotDesign& d) const {
        return {
            d.length, d.width, d.wheelDiameter, d.wheelWidth, d.trackWidth, d.wheelbase,
            d.tireFriction, d.motorStallTorque, d.motorFreeSpeed, d.battInternalR,
            d.massChassis, d.massBattery, d.fanPower_W, d.suctionPressure,
            d.comOffsetX, d.inertiaScale, d.hasFan ? 1.f : 0.f
        };
    }
    [[nodiscard]] RobotDesign decode(const std::array<float,GENES>& g) const {
        RobotDesign d;
        d.length=g[0]; d.width=g[1]; d.wheelDiameter=g[2]; d.wheelWidth=g[3];
        d.trackWidth=g[4]; d.wheelbase=g[5]; d.tireFriction=g[6];
        d.motorStallTorque=g[7]; d.motorFreeSpeed=g[8]; d.battInternalR=g[9];
        d.massChassis=g[10]; d.massBattery=g[11];
        d.fanPower_W=g[12]; d.suctionPressure=g[13];
        d.comOffsetX=g[14]; d.inertiaScale=g[15];
        d.hasFan     = g[16] > 0.5f;
        d.hasSuction = g[13] > 200.0f;     // suction "on" if appreciable pressure
        // keep sensor/motor placements at defaults scaled to body
        d.motorY = 0.5f*d.trackWidth;
        return clampDesign(d);
    }

    [[nodiscard]] std::array<std::pair<float,float>,GENES> geneRanges() const {
        return {{
            {bounds.lengthMin,bounds.lengthMax}, {bounds.widthMin,bounds.widthMax},
            {bounds.wheelDiaMin,bounds.wheelDiaMax}, {bounds.wheelWidMin,bounds.wheelWidMax},
            {bounds.trackMin,bounds.trackMax}, {bounds.wheelbaseMin,bounds.wheelbaseMax},
            {bounds.muMin,bounds.muMax}, {bounds.stallMin,bounds.stallMax},
            {bounds.freeSpdMin,bounds.freeSpdMax}, {bounds.battRmin,bounds.battRmax},
            {bounds.massChMin,bounds.massChMax}, {bounds.massBtMin,bounds.massBtMax},
            {bounds.fanPwrMin,bounds.fanPwrMax}, {bounds.suctPaMin,bounds.suctPaMax},
            {bounds.comMin,bounds.comMax}, {bounds.inertiaMin,bounds.inertiaMax},
            {0.f,1.f}
        }};
    }

    [[nodiscard]] RobotDesign clampDesign(RobotDesign d) const {
        auto C=[](float v,float lo,float hi){ return std::max(lo,std::min(hi,v)); };
        d.length=C(d.length,bounds.lengthMin,bounds.lengthMax);
        d.width =C(d.width,bounds.widthMin,bounds.widthMax);
        d.wheelDiameter=C(d.wheelDiameter,bounds.wheelDiaMin,bounds.wheelDiaMax);
        d.wheelWidth=C(d.wheelWidth,bounds.wheelWidMin,bounds.wheelWidMax);
        d.trackWidth=C(d.trackWidth,bounds.trackMin,bounds.trackMax);
        d.wheelbase=C(d.wheelbase,bounds.wheelbaseMin,bounds.wheelbaseMax);
        d.tireFriction=C(d.tireFriction,bounds.muMin,bounds.muMax);
        d.motorStallTorque=C(d.motorStallTorque,bounds.stallMin,bounds.stallMax);
        d.motorFreeSpeed=C(d.motorFreeSpeed,bounds.freeSpdMin,bounds.freeSpdMax);
        d.battInternalR=C(d.battInternalR,bounds.battRmin,bounds.battRmax);
        d.massChassis=C(d.massChassis,bounds.massChMin,bounds.massChMax);
        d.massBattery=C(d.massBattery,bounds.massBtMin,bounds.massBtMax);
        d.fanPower_W=C(d.fanPower_W,bounds.fanPwrMin,bounds.fanPwrMax);
        d.suctionPressure=C(d.suctionPressure,bounds.suctPaMin,bounds.suctPaMax);
        d.comOffsetX=C(d.comOffsetX,bounds.comMin,bounds.comMax);
        d.inertiaScale=C(d.inertiaScale,bounds.inertiaMin,bounds.inertiaMax);
        return d;
    }

    [[nodiscard]] RobotDesign randomDesign() {
        auto ranges = geneRanges();
        std::array<float,GENES> g{};
        for (int i=0;i<GENES;i++) {
            std::uniform_real_distribution<float> u(ranges[i].first, ranges[i].second);
            g[i]=u(rng);
        }
        return decode(g);
    }

    // ── fitness across maze set ───────────────────────────────────────────
    void evaluateIndividual(Individual& ind, const std::vector<uint32_t>& mazeSeeds,
                            uint32_t evalSeed) {
        Evaluator ev;
        std::mt19937 r(evalSeed);
        int n = int(mazeSeeds.size());
        float sumSpeed=0, sumExec=0, sumSlip=0, sumTrack=0, sumLoc=0,
              sumEnergy=0, sumJerk=0, sumColl=0, sumPeakV=0;
        int completed=0;
        float penaltyAccum=0;
        for (uint32_t ms : mazeSeeds) {
            DesignMetrics M = ev.evaluate(ind.design, ms, r, useTVLQR);
            if (!M.completed) { penaltyAccum += weights.dnfPenalty; continue; }
            completed++;
            sumSpeed += M.speedTime; sumExec += M.execTime; sumSlip += M.slipRatio;
            sumTrack += M.meanTrackingError; sumLoc += M.localizationError;
            sumEnergy += M.energy; sumJerk += M.peakJerk; sumColl += M.collisions;
            sumPeakV += M.peakVelocity;
        }
        float invC = completed>0 ? 1.f/float(completed) : 0.f;
        ind.meanSpeedTime  = sumSpeed*invC;
        ind.meanExecTime   = sumExec*invC;
        ind.meanSlip       = sumSlip*invC;
        ind.meanTrack      = sumTrack*invC;
        ind.meanLoc        = sumLoc*invC;
        ind.meanEnergy     = sumEnergy*invC;
        ind.meanJerk       = sumJerk*invC;
        ind.meanCollisions = sumColl*invC;
        ind.meanPeakV      = sumPeakV*invC;
        ind.reliability    = float(completed)/float(n);

        if (completed == 0) { ind.fitness = weights.dnfPenalty * 10.f; return; }

        float f = weights.wSpeed     * ind.meanExecTime
                + weights.wCollision * ind.meanCollisions
                + weights.wSlip      * ind.meanSlip
                + weights.wJerk      * ind.meanJerk
                + weights.wTrack     * ind.meanTrack
                + weights.wEnergy    * ind.meanEnergy
                + penaltyAccum / float(n);
        ind.fitness = f;
    }

    void evaluatePopulationParallel(std::vector<Individual>& pop,
                                    const std::vector<uint32_t>& mazeSeeds) {
        std::vector<std::future<void>> futs;
        futs.reserve(pop.size());
        uint32_t base = rng();
        for (size_t i=0;i<pop.size();++i) {
            uint32_t es = base + uint32_t(i)*2654435761u;
            futs.push_back(std::async(std::launch::async,
                [this,&pop,i,&mazeSeeds,es](){ evaluateIndividual(pop[i], mazeSeeds, es); }));
        }
        for (auto& f : futs) f.get();
    }

    // ── operators ─────────────────────────────────────────────────────────
    [[nodiscard]] Individual crossover(const Individual& a, const Individual& b) {
        auto ga=encode(a.design), gb=encode(b.design), gc=ga;
        std::uniform_real_distribution<float> u(-crossoverAlpha, 1.f+crossoverAlpha);
        for (int i=0;i<GENES;i++) { float t=u(rng); gc[i]=ga[i]+t*(gb[i]-ga[i]); }
        Individual c; c.design=decode(gc); return c;
    }
    void mutate(Individual& ind) {
        auto g=encode(ind.design); auto ranges=geneRanges();
        std::uniform_real_distribution<float> p(0.f,1.f);
        std::normal_distribution<float> n(0.f,1.f);
        for (int i=0;i<GENES;i++) {
            if (p(rng) < mutationRate) {
                float span = ranges[i].second - ranges[i].first;
                g[i] += n(rng) * mutationSigma * span;
            }
        }
        ind.design = decode(g);
    }
    [[nodiscard]] const Individual& tournament(const std::vector<Individual>& pop, int k=3) {
        std::uniform_int_distribution<int> u(0,int(pop.size())-1);
        int best=u(rng);
        for (int i=1;i<k;i++){ int c=u(rng); if(pop[c].fitness<pop[best].fitness) best=c; }
        return pop[best];
    }

    // ── one generation; returns sorted population (best first) ─────────────
    void step(std::vector<Individual>& pop, const std::vector<uint32_t>& mazeSeeds) {
        evaluatePopulationParallel(pop, mazeSeeds);
        std::sort(pop.begin(), pop.end(),
                  [](const Individual& a,const Individual& b){ return a.fitness<b.fitness; });
        std::vector<Individual> next;
        next.reserve(pop.size());
        for (int i=0;i<eliteCount && i<int(pop.size());++i) next.push_back(pop[i]);
        while (int(next.size()) < populationSize) {
            const Individual& pa = tournament(pop);
            const Individual& pb = tournament(pop);
            Individual child = crossover(pa, pb);
            mutate(child);
            next.push_back(child);
        }
        pop = std::move(next);
    }

    [[nodiscard]] std::vector<Individual> initialPopulation() {
        std::vector<Individual> pop(populationSize);
        for (auto& ind : pop) ind.design = randomDesign();
        return pop;
    }

    std::mt19937 rng;
};

// Build a deterministic set of maze seeds for training/validation.
inline std::vector<uint32_t> makeMazeSeeds(int n, uint32_t base=777u) {
    std::vector<uint32_t> v; v.reserve(n);
    std::mt19937 r(base);
    for (int i=0;i<n;i++) v.push_back(r());
    return v;
}

} // namespace gdw