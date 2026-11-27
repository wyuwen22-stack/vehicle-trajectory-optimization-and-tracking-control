# Method and implementation notes

## Saved numerical evidence

The two source MAT files contain `scenario.veh`, `scenario.map`, `scenario.hybridAstar`, `scenario.obca` and `scenario.mpc`. The simple-scene file additionally retains a dynamics-only `obca_step1` result. `scenario.obca.X` is a 4 × (N+1) state matrix; `U` is a 2 × N control matrix. State order is `[x, y, heading, speed]`, and control order is `[steering, acceleration]`. `scenario.mpc.X_actual` is the tracked state history.

Position errors are calculated at matching sample indices, including index zero: `sqrt((x_actual-x_ref)^2 + (y_actual-y_ref)^2)`. RMSE is `sqrt(mean(error^2))`. The stored error arrays were checked against values recomputed from the stored states. Obstacle-avoidance heading errors use the direct heading difference, while parking wraps the difference with `atan2(sin(error), cos(error))`, matching the corresponding MATLAB scripts.

The retained values are 0.14055815676714462 m and 0.12344302030679746 m for position RMSE. Both runs begin with position error `sqrt(0.3^2 + 0.3^2)` m. The two result rows are not a statistical estimate of tracking robustness. They use the same kinematic plant as the controller's underlying nonlinear model, with a prescribed initial-state mismatch and no ongoing disturbances.

`RESULTS.json` also records discrete three-point curvature maxima. These are geometric estimates from sampled x-y positions, using the same formula and endpoint copying as the analysis scripts. They are not identical to steering-derived `tan(delta)/L`, particularly near direction changes. Different sampling densities between the HA and optimized paths affect such comparisons.

## Search and trajectory optimization

The HA seed uses MATLAB's Navigation Toolbox, a 5 cells/m occupancy grid and scenario-specific map inflation. Parking's 0.2 m inflation is smaller than the vehicle half-width. The seed may therefore require substantial correction by rectangular-body optimization.

The trajectory state and control variables follow a forward-Euler bicycle model with fixed 0.4 s steps. The simple-scene optimizer uses 40 intervals and the parking optimizer uses 50. Both impose OBCA dual-separation constraints at nodes, not at interior control-interval poses. There is no adaptive constraint generation or interval collision verifier in this MATLAB package.

The simple-scene objective penalizes steering, acceleration and adjacent control differences. In `run_obca_parking.m`, the position-deviation and speed-difference sums are nested inside the outer control-cost loop, with the loop variable reused. Those sums are therefore accumulated repeatedly, giving horizon-dependent effective weights. This is part of the preserved implementation; the displayed coefficients should not be interpreted as a once-per-trajectory objective identical to later Python variants.

IPOPT uses `tol = 1e-4`, with iteration caps of 5000 for obstacle avoidance and 8000 for parking. The solver's tolerance is separate from the geometric 0.05 m node margin. The simple scene enforces exact terminal pose and zero speed. Parking enforces a terminal pose band and speed between −0.1 and +0.1 m/s; the saved terminal speed is approximately −0.0999998 m/s.

## Controller scope

The LTV controller uses Jacobians of the discrete nonlinear model along the reference, a quadratic tracking cost and linear deviation dynamics. It is formulated through CasADi Opti and solved with IPOPT, although each linearized tracking subproblem has quadratic-program structure. It does not use the MATLAB MPC Toolbox.

Prediction length is at most 10 steps and shrinks near the end of the reference. The controller constrains steering and acceleration. Its declared `v_max` variable is not imposed as a predicted-state bound, and there are no obstacle, terminal or steering-increment constraints in the controller. Thus planning constraints should not be advertised as independently enforced closed-loop guarantees.

If a tracking solve throws an exception, the code sets the correction to zero and applies the clipped reference control. It does not record a per-step solver-status ledger. A smooth tracking plot cannot establish that every optimization subproblem succeeded.

## Clearance and solver-status limitations

The analysis routines evaluate a corner-to-obstacle-edge distance approximation at stored planning nodes. Their sign test checks whether vehicle corners lie inside obstacles. This can miss intersection configurations involving edges or obstacle containment; it is not a complete signed polygon-distance algorithm. It also does not check the swept body between nodes or independently validate the tracked motion. The scripts' console wording about whole-path collision freedom should not be treated as a continuous-safety certificate.

The optimization catch blocks can retrieve debug values and continue saving after a solver failure. Saved results do not contain a persistent native-solver-status field. When running a new experiment, retain the MATLAB console log and distinguish a reported solve success from the existence of a MAT file. No new all-pass acceptance claim is inferred from the bundled arrays alone.

## Animations

The README GIFs are display assets generated from the saved reference and tracked-state arrays. They advance one stored state per frame, with 0.2 s display delays for a 0.4 s simulation step. Start and end frames pause longer. The plots use explicit legend handles and include the complete body extents; no additional solver or plant simulation is run to create them.

The original animation scripts remain available unchanged. They run their own controllers and create different root-level output GIFs. The obstacle animation uses a larger initial mismatch than the saved numerical example. Their automatic legend selection can associate labels with unintended graphics objects. The README therefore uses the separately generated, explicitly labeled GIFs in `docs/media/`.

## Verification and source preservation

All 15 MATLAB source scripts and both MAT files were copied without modification. Their SHA256 fingerprints are recorded in `PROVENANCE.json`. Metrics were recomputed from the bundled arrays, and all animation frames were decoded and inspected at representative times. HTML and Markdown local links and the final archive manifest were checked. The earlier MATLAB verification used R2025b on Windows; this packaging run did not repeat planning, tracking or installation.

The Git metadata, editor autosave file and original dissertation are excluded. The dissertation contains additional personal information. Academic conclusions in this package are limited to the included MATLAB evidence, rather than importing results from Python or a separate 128-case benchmark.
