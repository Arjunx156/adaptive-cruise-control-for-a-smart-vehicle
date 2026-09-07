# Adaptive Cruise Control for a Smart Vehicle

**Project domain:** Control systems, vehicle dynamics, embedded sensing
**Tools:** MATLAB / Simulink (modelling and simulation), small robotic vehicle with ultrasonic sensing (physical demonstration)

---

## 1. Overview

Adaptive Cruise Control (ACC) is a driver-assistance system that extends conventional cruise control with awareness of the road ahead. Conventional cruise control holds a single set speed and is indifferent to traffic; if a slower vehicle appears in front, the driver must intervene. ACC removes that limitation by adding a forward-looking sensor and a second control objective, so the system regulates both **speed** and **separation**.

The result is a controller with two competing goals. It should travel at the speed the driver selected, and it should never come closer to the vehicle ahead than is safe. These goals conflict whenever the vehicle ahead is slower, and the way that conflict is resolved is the heart of the design.

This project addresses the problem on two levels. A MATLAB/Simulink model captures the vehicle dynamics and control law analytically, allowing behaviour to be examined across scenarios that would be dangerous or impractical to stage physically. A small robotic vehicle fitted with an ultrasonic rangefinder then demonstrates the same control law operating on real hardware, with real sensor noise and real actuator limits.

---

## 2. The Control Problem

### 2.1 Two operating modes

The system exhibits two distinct behaviours depending on the traffic situation.

**Speed mode** applies when the road ahead is clear, or when the vehicle ahead is far enough away or fast enough to be irrelevant. The controller behaves as ordinary cruise control, regulating velocity to the driver's set point.

**Spacing mode** applies when a slower vehicle occupies the space ahead. The controller abandons the set speed and instead regulates the gap, effectively matching the lead vehicle's velocity while holding a chosen separation.

The transition between these modes is not a discrete event with a clean boundary. A vehicle drifting slowly into range, or a lead vehicle whose speed hovers near the set speed, produces an ambiguous situation. A controller that switches abruptly on a threshold will chatter at that boundary, producing oscillating acceleration commands that are uncomfortable and mechanically wasteful. Resolving this smoothly is a genuine design concern rather than an implementation detail.

### 2.2 The spacing policy

Before the gap can be regulated, the system must define what gap it is aiming for. This definition is the **spacing policy**, and the choice carries real consequences.

A **constant distance policy** targets a fixed separation regardless of speed. It is simple but physically wrong: a gap that is generous at walking pace is lethal at highway speed, because stopping distance grows roughly with the square of velocity.

A **constant time gap policy** scales the target separation with the ego vehicle's own speed, expressing the gap as the time that would elapse before reaching the lead vehicle's current position. A standstill offset is added so the target does not collapse to zero when stopped. This matches how human drivers are taught to follow, matches production ACC systems, and has the important theoretical property of **string stability** — disturbances propagating backwards through a line of following vehicles are attenuated rather than amplified. Constant distance policies lack this property, which is why a chain of vehicles using them can produce phantom traffic jams from a single small braking event.

### 2.3 The spacing error

With a target defined, the controller acts on the difference between measured and desired separation. A positive error means the vehicle has fallen too far back and should accelerate; a negative error means it has closed in too tightly and must decelerate.

An important structural feature of this loop is that the gap is the **integral** of relative velocity — the separation changes at a rate given by the difference between the two vehicles' speeds. The plant therefore contains an inherent integrator, which has direct consequences for stability. Purely proportional control on such a plant tends to oscillate, because the controller responds to where the vehicle is without accounting for how fast it is arriving. Damping proportional to closing speed is what makes the approach converge rather than overshoot.

---

## 3. Vehicle Dynamics

### 3.1 Longitudinal model

The relevant physics is one-dimensional: motion along the direction of travel. The net accelerating force on the vehicle is the tractive force from the powertrain, less the resistances acting against it.

**Aerodynamic drag** grows with the square of velocity and dominates at high speed. Its nonlinearity is the main reason the plant is not linear over a wide operating range.

**Rolling resistance** arises from tyre deformation and is roughly proportional to weight, largely independent of speed.

**Grade force** is the component of gravity along the road surface. It acts as an external disturbance the controller must reject — the reason a cruise control system slows on an unexpected hill until the integral action compensates.

### 3.2 Linearisation

Because the model is nonlinear, analysis is normally performed on a linearised version valid near a nominal cruising speed. Under this approximation the quadratic drag term collapses into linear damping, and the vehicle becomes a first-order system: a mass being pushed against velocity-proportional friction. This form is analytically tractable and permits the standard frequency-domain tools — root locus, Bode analysis, gain and phase margins — to be applied.

The approximation is honest only near the operating point at which it was taken. Behaviour far from that speed will diverge from the model, which is one of the more instructive discrepancies the project surfaces.

### 3.3 Actuator dynamics

A vehicle does not produce a commanded force instantly. Engine or motor response, driveline compliance, and torque build-up introduce lag between command and effect. Neglecting this lag produces a model that appears more stable than reality, and gains tuned against it will be too aggressive on hardware. Including a first-order actuator lag is what makes the simulated system a fair predictor of the physical one.

### 3.4 Physical constraints

Two constraints bound what the controller may command.

**Comfort limits** restrict acceleration and deceleration to a band that passengers tolerate. Sharp braking is available but reserved for genuine emergencies; routine gap regulation must stay gentle.

**Capability limits** reflect what the vehicle can physically achieve — available engine torque, and tyre friction beyond which braking demand simply cannot be met.

Both appear in the model as saturation on the control signal, and saturation is what makes the controller a nonlinear system regardless of how linear the plant model is.

---

## 4. The PID Controller

### 4.1 Role of each term

The controller uses proportional, integral, and derivative action, each addressing a distinct deficiency.

The **proportional** term produces a response scaled to the current error. Acting alone it leaves steady-state offset, because a nonzero error is required to generate the force that balances drag and grade. Increasing the gain reduces but never eliminates this offset, and eventually destabilises the loop.

The **integral** term accumulates error over time, driving the steady-state offset to zero. It is what allows the vehicle to hold exactly the set speed on a constant gradient. Its cost is phase lag, which erodes stability margin and tends to increase overshoot.

The **derivative** term responds to the rate at which error is changing, supplying anticipation. In the spacing loop it has a clear physical meaning — it is proportional to closing velocity, so it commands braking when the gap is shrinking quickly even if the gap is still comfortable. This is the term that prevents the vehicle from charging at a gap that is already closing, and it is the primary source of damping in a loop whose plant already contains an integrator.

### 4.2 Integrator windup

When the commanded acceleration exceeds what the actuator can deliver, the applied and commanded values diverge. The integrator, unaware of this, continues accumulating error and grows to a value the system cannot honour. When the situation resolves, that accumulated charge must be unwound before the controller responds correctly, producing a large delayed overshoot.

This behaviour appears specifically in the scenarios that matter — hard braking, aggressive approach, sudden cut-ins — because those are precisely the situations that drive the actuator into saturation. Windup is therefore not a peripheral concern but a central one for any ACC implementation, and mitigating it is a required part of a credible design.

### 4.3 Tuning as a trade-off

Gain selection balances objectives that cannot be simultaneously maximised. Faster response costs stability margin. Tighter gap regulation costs ride comfort. Aggressive disturbance rejection amplifies sensor noise through the derivative path. There is no single correct set of gains, only a set appropriate to a stated specification — and articulating that specification is part of the design work.

---

## 5. Sensing

### 5.1 The ultrasonic rangefinder

The physical demonstration uses ultrasonic ranging: a pulse is emitted, and separation is inferred from the time until its echo returns. The technique is inexpensive and adequate at the speeds and distances a small robot operates over, and it stands in for the radar or lidar used in production systems.

Its limitations, however, shape the design of the demonstration.

**Measurement noise** is significant and includes outright spurious readings, not merely small random variation. This matters most for the derivative term, which amplifies high-frequency content — differentiating an unfiltered ultrasonic signal produces noise rather than useful closing-velocity information.

**Limited update rate** is imposed by the physics of sound propagation and echo decay. The sensor cannot be sampled arbitrarily fast, and this places a ceiling on control loop bandwidth.

**Minimum and maximum range** bound the usable operating envelope. Very close targets fall inside the sensor's blind zone, which constrains how small the standstill gap can safely be.

**Beam spread** means the sensor returns the nearest reflector within a cone rather than a point measurement directly ahead. Walls, furniture, and floor irregularities can all register as obstacles.

**Target dependence** means readings vary with the reflecting surface. Soft materials absorb the pulse and angled surfaces deflect it away, both producing dropouts.

### 5.2 Sensing as a design constraint

These characteristics are not incidental nuisances to be engineered away — they are what distinguishes a real control system from a simulated one. Filtering to suppress noise introduces delay, and delay reduces phase margin. Sensor bandwidth limits achievable closed-loop bandwidth. The sensor and controller cannot be designed independently, and the interaction between them is one of the more valuable things the hardware half of the project exposes.

### 5.3 Speed measurement

Regulating speed requires knowing speed, which on the robot comes from wheel encoders rather than the rangefinder. Without this measurement the speed loop cannot close, and only the spacing behaviour could be demonstrated. Encoder-derived speed is itself imperfect — quantised, and noisier at low speed where few pulses accumulate per sampling interval — which is a second instance of sensing quality bounding control performance.

---

## 6. Simulation

### 6.1 Purpose

Simulation permits the controller to be examined under conditions that cannot be reproduced safely or repeatably in hardware. Emergency braking by a lead vehicle, high-speed following, and sudden cut-ins are all trivially staged in software. Simulation also provides perfect observability: every internal state is available for inspection, including quantities like true relative velocity that no sensor measures directly.

Critically, it isolates the control design from the noise and hardware faults that would otherwise confound it. A controller that fails in simulation has a design problem; a controller that works in simulation but fails on hardware has an implementation or modelling problem. Separating these two failure modes is the main methodological value of the simulation stage.

### 6.2 Scenarios of interest

Several situations exercise distinct aspects of the design.

**Steady-state following** verifies that the desired gap is achieved and held without offset or hunting, confirming the integral action functions correctly in spacing mode.

**Approach from a large gap** exercises the transition from speed mode into spacing mode, and reveals whether the controller decelerates smoothly or overshoots into the lead vehicle.

**Lead vehicle braking** is the safety-critical case. It tests whether the controller responds fast enough under acceleration limits, and whether saturation and windup are handled correctly.

**Lead vehicle accelerating away** tests the reverse transition, back into speed mode, and confirms the controller does not chase indefinitely beyond the set speed.

**Cut-in** models a vehicle entering the lane ahead, presenting the controller with a discontinuous step change in measured gap — an aggressive disturbance rejection test.

**Stop-and-go traffic** stresses low-speed behaviour, where the time-gap policy produces very small targets and where both sensor and actuator limitations are most pronounced.

### 6.3 Interpreting results

Three signals characterise behaviour. **Velocity trajectories** for both vehicles show tracking quality and mode transitions. **Actual against desired separation** shows regulation accuracy and, most importantly, whether the gap ever approaches zero. **Control effort** reveals whether performance was achieved within realistic limits or by commanding accelerations no vehicle could deliver.

Good tracking achieved through saturated, oscillating control effort is not a successful design, and the third plot is what exposes this.

---

## 7. Hardware Demonstration

### 7.1 What the robot contributes

The physical vehicle demonstrates that the control law survives contact with reality. It operates at low speed over short distances, so the parameters differ from the automotive case, but the structure of the problem is identical: a plant with mass and friction, an actuator with limits and lag, a noisy distance sensor, and a controller mediating between speed and separation objectives.

### 7.2 Differences from simulation

Several effects present on hardware are typically absent from the model.

**Motor deadband** means low PWM commands produce no motion at all, because static friction is not overcome. The actuator is therefore not merely saturated at its extremes but dead near zero — a nonlinearity with no counterpart in the linear model.

**Discretisation** replaces the continuous controller with one running at a finite sampling rate. Integral and derivative terms become approximations, and their behaviour depends on timing regularity that embedded code does not automatically provide.

**Unmodelled friction** in bearings and gearboxes, along with surface variation, produces disturbances the linear damping term does not capture.

**Sensor latency** adds delay around the loop, reducing phase margin relative to the simulated design.

The consequence is that gains tuned in simulation rarely transfer unchanged. Understanding and documenting *why* they differ is more valuable than the transfer succeeding, since the discrepancy is where the modelling assumptions become visible.

---

## 8. Scope and Limitations

The project addresses longitudinal control only. Lateral behaviour — steering, lane keeping, lane changes — is outside scope, and the lead vehicle is assumed to remain in the lane ahead.

The system is driver-assistance, not autonomy. It regulates speed and spacing under supervision; it does not perceive traffic signals, pedestrians, or road geometry, and it does not make tactical decisions.

Ultrasonic sensing constrains the demonstration to low speeds and short ranges. Production systems use radar or lidar with far greater range and reliability, and the substitution is a deliberate cost and complexity trade-off appropriate to the demonstration scale.

Single-vehicle following is assumed. Multi-target tracking, target selection among several candidates, and platooning across a chain of vehicles are all natural extensions but are not addressed here.

---

## 9. Relevance

Adaptive cruise control is one of the most widely deployed autonomous functions in production vehicles, and forms a building block for higher levels of driving automation. It is also a well-chosen teaching problem: the physics is tractable enough to model honestly, the control objectives conflict in a way that forces genuine design decisions, and the gap between simulated and physical behaviour is large enough to be instructive without being insurmountable.

The project connects analytical control theory, physical modelling, sensor characteristics, and embedded implementation within a single system where each constrains the others — which is a reasonable approximation of how control engineering actually works.
