# Adaptive Cruise Control for a Smart Vehicle

MATLAB and Simulink simulation of a two-mode adaptive cruise controller: it regulates
speed when the road ahead is clear, and separation when it is not.

The full project brief is in [`docs/adaptive_cruise_control_project.md`](docs/adaptive_cruise_control_project.md).
This repository implements its simulation half (sections 3–6). The hardware
demonstration (section 7) is not built yet, but the parameter layer already carries a
robot-scale set so the same code can drive it.

---

## Running it

Requires MATLAB R2024b with Control System Toolbox and Simulink.

```matlab
accSetupPath      % puts src/, scripts/ and tests/ on the path
runAll            % regenerates everything in results/ (several minutes)
```

Or from a shell:

```bash
matlab -batch "accSetupPath; runAll"
matlab -batch "runTests"                       % 60 tests, about 8 minutes
matlab -batch "accSetupPath; runAllScenarios('automotive','ideal')"
```

Individual entry points:

| Script | What it does |
|---|---|
| `runAllScenarios(paramSet, sensorMode)` | The eight scenarios, one figure each plus a metrics CSV |
| `runLinearAnalysis(paramSet)` | Bode, root locus, margins, linearisation validity |
| `runWindupComparison(paramSet)` | Anti-windup strategies compared numerically |
| `buildAccSimulinkModel()` | Generates `results/accModel.slx` from the parameter struct |
| `runSimulinkComparison(scenario, paramSet)` | Simulink against MATLAB, same scenario |
| `runTests()` | Full test suite. `runTests('Coverage', true)` adds statement coverage of `src/`, but the instrumentation slows the integration loop by well over an order of magnitude |

---

## How it works

### The plant

A one-dimensional longitudinal model, integrated with fixed-step RK4 at 1 ms, with the
controller running at 50 Hz zero-order hold so the implementation is discrete from the
start rather than idealised and then discretised later.

```
da_trac/dt = (sat(a_des) - a_trac) / tau        first-order actuator lag
dv_ego/dt  = a_trac
             - 0.5*rho*Cd*A*v|v| / m            aerodynamic drag
             - Cr*g*tanh(v/v_eps)               rolling resistance
             - g*sin(theta(t))                  road grade
```

Drag, rolling resistance and grade are deliberately **not** in the controller's model.
They act as genuine disturbances, which is what makes "proportional action alone leaves a
steady-state offset" and "integral action recovers the set speed on a hill" demonstrable
facts in this repository rather than assertions.

### The controller

Two PID loops run every sample, and the **smaller** of their two accelerations is
commanded:

- **Speed loop** on `v_set - v_ego`, derivative on measurement.
- **Spacing loop** on `gap - (d_0 + t_gap * v_ego)`, with derivative action on closing
  velocity.

There is no threshold and no discrete switch, so there is nothing for the system to
chatter against at the mode boundary. The reported mode is a label for plotting; it never
gates the arithmetic.

Two limit sets bound the command. Comfort limits apply routinely; capability limits are
unlocked when time-to-collision drops below a threshold **or** the gap falls below 60% of
its target — two independent triggers, because a slow but severe encroachment never
produces a small TTC. A jerk rate limit smooths the command, with a much higher ceiling
once emergency authority is unlocked.

### Anti-windup

A two-loop ACC has two windup sources, and both matter:

1. **Actuator saturation** — the classic case.
2. **Losing the min-selection** — the loop that did not win still sees its own error, and
   left alone it charges up while inactive, then dumps that charge the instant it takes
   over.

Both reduce to one rule: a loop integrates only when it is in command *and* free to act.
Three strategies are implemented (`p.control.antiWindupMethod`) and compared by
`runWindupComparison`; conditional integration is the default.

### Sensing

`p.sensor.mode` switches between `ideal` (the controller sees the plant exactly) and
`realistic`, which applies the ultrasonic limitations of section 5: a slower update rate
than the control loop, transport delay, quantisation, noise, dropouts, spurious short
readings and a bounded range. In realistic mode relative velocity is not measured at all
— it is estimated from a noisy range signal by a median filter, a low-pass and an
alpha–beta tracker, which is exactly the constraint that shapes the derivative term.

Running every scenario both ways isolates control-design problems from sensing problems.

---

## Results

`results/` holds a three-panel figure per scenario per configuration — velocities, actual
against desired separation, and control effort against its limits — plus a metrics CSV.
The third panel is the one that matters: good tracking bought with saturated, oscillating
control effort is not a successful design.

Headline numbers, automotive parameters, ideal sensing:

| Scenario | min gap | min TTC | settled gap error | max decel |
|---|---|---|---|---|
| `steadyFollow` | 35.0 m | ∞ | ~0 | 0.26 m/s² |
| `approach` | 18.3 m | 8.4 s | 0.75 m | −0.75 m/s² |
| `leadBraking` | 4.2 m | 2.3 s | 0.84 m | −6.6 m/s² |
| `leadAccelerating` | 27.5 m | ∞ | — | — |
| `cutIn` | 17.9 m | 5.5 s | 0.03 m | −5.8 m/s² |
| `stopAndGo` | 5.0 m | 2.2 s | 0.82 m | −4.2 m/s² |

`gradeDisturbance` and `coldStart` run on an empty road, so their gap columns carry no
meaning; they appear in the CSVs alongside the rest.

No scenario closes the gap to zero, at either parameter scale, with either sensing mode —
this is asserted by the test suite, not just observed.

Stability margins at the tuned gains (linearised at 25 m/s):

| Loop | phase margin | crossover | closed-loop bandwidth |
|---|---|---|---|
| speed | 71.9° | 0.49 rad/s | 0.66 rad/s |
| spacing | 56.0° | 0.93 rad/s | 1.54 rad/s |

The spacing loop is **conditionally stable**: it carries a double integrator at low
frequency — the plant's own, since the gap is the integral of relative velocity, plus the
controller's — so its phase starts at −180° and the derivative term lifts it back.
*Reducing* its loop gain destabilises it. The margin report says so explicitly, because
the usual reading of a gain margin is inverted here.

---

## Layout

```
accSetupPath.m              path setup, called by every entry point
runTests.m                  test runner with coverage
docs/                       the project brief
src/params/                 accParamsAutomotive, accParamsRobot, accLoadParams
src/model/                  plant derivative, saturation, trim
src/control/                controller step, integrator update, spacing policy, TTC
src/sensing/                sensor model and estimator
src/sim/                    RK4 simulation loop, Simulink controller runtime
src/scenarios/              the scenario factory
src/analysis/               linear models, margin report, metrics
src/plot/                   the three-panel result figure
scripts/                    entry points
tests/                      matlab.unittest test classes
results/                    figures, metrics CSVs, generated .slx
```

Every quantity the simulation needs lives in the parameter struct; nothing is hard-coded
downstream, which is what lets the automotive and robot sets share one simulation core.
Scenario severity is scaled to the parameter set too — a lead vehicle that stops in ten
actuator time constants poses a comparable challenge at either scale, whereas a fixed
deceleration in m/s² would make the robot's braking event roughly twelve times harsher
than the car's and compare nothing useful.

---

## Simulink

`buildAccSimulinkModel` generates `results/accModel.slx` programmatically rather than
committing a binary nobody can diff. The plant is native Simulink blocks; the controller
is a MATLAB Function block calling the same control law the `.m` simulation uses, so the
two cannot drift apart.

`runSimulinkComparison` checks them against each other. On `steadyFollow` they agree to
numerical precision; on `leadBraking` to 0.003 m RMS on a gap that swings through 35 m.

That check earned its place: it caught the Simulink actuator being modelled with a
Transfer Fcn block, whose state cannot be initialised, so the block-diagram vehicle
started with no tractive effort while the MATLAB one started at trim. Building the lag
from a summing junction, a gain and an integrator instead removed a 14-fold error.

---

## Not covered

Longitudinal control only — no steering, lane keeping or lane changes. Single lead
vehicle: no multi-target tracking, target selection, or platooning. Driver assistance,
not autonomy. The hardware demonstration of section 7 is not implemented; the robot
parameter values are engineering estimates and are flagged as such in
`accParamsRobot.m`, and must be replaced with measured values before any claim is made
about how well the simulation predicts the physical vehicle.
