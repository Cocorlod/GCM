# GDW Design Lab

A C++20 application that **discovers the fastest physically-achievable micromouse
design for the GDW v4.1 stack** by generating thousands of robot designs,
simulating each across many randomly-generated mazes, and evolving them with a
genetic algorithm. It is a digital twin + physics layer + optimization engine +
design generator wrapped around the real GDW v4.1 planning/control/localization
code.

---

## Honest status (read this first)

* *The headless optimizer is real and verified.* It was compiled with
  g++ 13.3 -std=c++20, run, and debugged to a working state. A typical run
  (--pop 30 --mazes 8 --gens 18) executes ~4,300 closed-loop maze simulations
  in ~20 s on one core and writes a coherent best design plus CSV/JSON/spec/CAD
  outputs.
* *The GUI (src/gui_main.cpp, SDL2 + Dear ImGui) is NOT verified.* It could
  not be compiled in the author's sandbox (no SDL2, no Dear ImGui, no network to
  fetch them). It follows the canonical imgui_impl_sdl2 + imgui_impl_sdlrenderer2
  pattern and reuses the same verified core headers, so expect it to build with
  at most minor local fix-ups. It is *OFF by default* in CMake.
* Some subsystems are deliberately *proxy-fidelity* (see What is modeled and
  Limitations). The point of the executor is to produce stable,
  design-sensitive metrics for the optimizer — not to be a full multibody sim.

---

## Build

### Headless optimizer (verified path)

bash
cmake -S . -B build
cmake --build build -j
./build/gdw_design_lab --pop 30 --mazes 8 --gens 18 --seed 2025 --out build


Or without CMake at all:

bash
g++ -std=c++20 -O2 -pthread -Isrc src/main_headless.cpp -o gdw_design_lab
./gdw_design_lab --pop 30 --mazes 8 --gens 18 --out .


### GUI (unverified; needs SDL2, fetches Dear ImGui)

bash
# install SDL2 dev package first, e.g.  sudo apt install libsdl2-dev
cmake -S . -B build -DBUILD_GUI=ON
cmake --build build -j
./build/gdw_design_lab_gui


---

## Headless usage


gdw_design_lab [--budget N] [--mazes M] [--pop P] [--gens G]
               [--no-tvlqr] [--seed S] [--out DIR]


| flag        | meaning                                                            |
|-------------|--------------------------------------------------------------------|
| --pop P   | GA population size (designs per generation)                        |
| --mazes M | mazes averaged per fitness evaluation (raise to 100+ to avoid overfitting) |
| --gens G  | number of generations                                              |
| --budget N| if --gens is omitted, generations ≈ N / population               |
| --no-tvlqr| use the PD fallback instead of TVLQR in the executor               |
| --seed S  | RNG seed (reproducible mazes + GA)                                 |
| --out DIR | output directory                                                   |

Total design evaluations = gens × pop; total maze simulations = that × mazes.
For a serious search use something like --pop 60 --mazes 120 --gens 60.

### Outputs (written to --out)

* best_design.json — design + derived physics + expected performance
* best_design.spec.txt — human-readable spec sheet (mm)
* hall_of_fame.csv — top-10 designs of the final population
* best_design.cad.txt — neutral parametric CAD description
* convergence.csv — best fitness / exec-time / etc. per generation

---

## Pipeline (what each evaluation runs)

For every (design, maze):

1. *Physics derivation* — physics::derive() turns the RobotDesign into
   dynamic limits: traction-limited lateral accel (Kamm circle)
   a_lat = μ·(g + F_down/m), motor-torque-limited accel 2T/(r·m) with battery
   sag, top speed from wheel free-speed, downforce from fan/suction, yaw inertia,
   COM. These feed the planner's RobotParams.
2. *Scout run* — Explorer::explore: flood-fill exploration with the ESKF
   dead-reckoning, wall-centering PID, and D* Lite notifications in the loop.
3. *Speed-run plan* — best-of-4-goal Theta\* path → racing-line optimization →
   clothoid trajectory (Gauss–Legendre) → Kamm-circle S-curve velocity profile.
4. *Closed-loop execution* — a 1-DOF-along-path executor at 2 kHz: speed chases
   the planned profile but is clamped by motor torque and the Kamm traction
   circle (exceeding it registers *slip*); the real ESKF runs on noisy
   dead-reckoning with periodic wall fixes (*localization error); **collisions*
   occur when slip + near-limit cornering + speed jitter exceed the
   body-width-dependent corridor clearance.

The GA averages metrics across the maze set and minimizes:


fitness = exec_time
        + 1.5·collisions + 3.0·slip + 0.00008·peak_jerk
        + 5.0·track_error + 0.0015·energy
        + 50·(mazes the design failed to complete)


All weights live in FitnessWeights (src/genetic.hpp) — tune to taste.

---

## What is modeled

Differential-drive traction/Kamm coupling, motor torque & battery-sag-limited
acceleration, top-speed limit, optional fan/suction downforce (with mass & energy
cost), mass & yaw-inertia from a component budget, tyre friction, wheel geometry,
sensor noise, gyro bias/drift, encoder error, ESKF measurement updates & covariance
propagation, and the full GDW v4.1 planning stack (FloodFill, D\* Lite, Theta\*,
clothoids, velocity profile, TVLQR, PD, wall-centering).

The RobotDesign genome (src/robot_design.hpp) exposes geometry, drivetrain,
placements, mass distribution, tyre μ, motor params, battery, and fan/suction.

---

## Bugs found & fixed while building (vs. the original v4.1 code)

* *D\ Lite infinite loop / bad_alloc** — locally-consistent nodes (g==rhs)
  fell into the underconsistent branch, which re-raised g to ∞ and re-queued the
  node forever on dense mazes. Fixed to handle only the strictly-underconsistent
  case (plus an expansion cap). This bug was latent in the original; it only never
  fired because the original ran D\* on near-empty optimistic maps.
* *Exploration stall* — sensing reveals all four walls on arrival, so no visited
  cell was ever a "frontier"; the frontier set was always empty and exploration
  quit early. Replaced with canonical flood-fill exploration (reaches the goal on
  20/20 random mazes tested).
* *TVLQR NaN* — the forward-Euler Riccati diverged for large dt = ds/v near
  v→0. Clamped dt and sanitized the gains.

---

## Limitations (be aware)

* *Localization error is a proxy* and reads high (dm-scale) because along-corridor
  dead-reckoning is only corrected at sparse wall features; this is reported as a
  metric but only the lateral deviation drives collisions, so it does not distort
  the search. Real micromice localize longitudinally by cell-edge/post counting,
  which is not modeled.
* The executor is *1-DOF along the planned path*, not a full 2-D multibody chase.
  This is a deliberate trade for stability and clean design-sensitivity.
* std::async parallelism was developed on a 1-core sandbox (so it serialized
  there); on a multi-core machine population evaluation runs in parallel.
* The *CAD export* is a neutral parametric description; STEP/Fusion/SolidWorks/
  FreeCAD back-ends are stubbed behind CADExporter (src/export.hpp).
* The *GUI is unverified* — see Honest status.

---

## File layout


CMakeLists.txt            headless target (verified) + optional GUI target
src/gdw_core.hpp          GDW v4.1 stack as a library (planners/ESKF/control)
src/robot_design.hpp      RobotDesign genome + physics::derive()
src/maze_gen.hpp          random + structured ground-truth maze generation
src/evaluator.hpp         per-(design,maze) evaluation + closed-loop executor
src/genetic.hpp           genetic algorithm + fitness + parallel evaluation
src/export.hpp            JSON/CSV/spec/CAD output
src/main_headless.cpp     CLI optimizer  (verified)
src/gui_main.cpp          SDL2 + Dear ImGui visualizer  (UNVERIFIED)