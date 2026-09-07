function out = accRunSimulation(scenario, p)
%ACCRUNSIMULATION Run one ACC scenario to completion.
%
%   out = accRunSimulation(scenario, p)
%
%   The plant is integrated with fixed-step RK4 at p.sim.dt. The controller runs
%   at the slower p.control.dt with zero-order hold, so the discrete nature of
%   the implementation is present from the start rather than bolted on later
%   (section 7.2).
%
%   scenario fields (see accScenario):
%       name, description, tEnd, x0, setSpeed, leadAccel, grade, events
%
%   out is a struct of logged signals, all sampled at p.sim.dt.

dtSim  = p.sim.dt;
dtCtrl = p.control.dt;
stepsPerControl = max(1, round(dtCtrl / dtSim));

t = (0:dtSim:scenario.tEnd)';
n = numel(t);

setSpeed = scenario.setSpeed;

x = scenario.x0(:);
gap0 = x(1) - x(3);

ctrl   = accControllerInit(p, x(4));
sensor = accSensorInit(p, gap0, x(4));

% Prime both integrators at the trim operating point so t=0 is a steady state
% rather than a startup transient that would contaminate every scenario.
aTrim = accTrimAcceleration(x(4), scenario.grade(0), p);
ctrl.speedIntegral   = aTrim;
ctrl.spacingIntegral = aTrim;

aApplied = aTrim;
mode = "speed";
satInfo = struct('isEmergency', false, 'isMagnitudeLimited', false, ...
                 'isRateLimited', false, 'isSaturated', false, ...
                 'aMin', p.actuator.aMinComfort, 'aMax', p.actuator.aMaxComfort);
meas = struct('gap', gap0, 'vEgo', x(4), 'vRel', 0, 'isValid', true);
desiredGap = accSpacingPolicy(x(4), p);
ttc = Inf;

log = preallocateLog(n);
pendingEvents = scenario.events;

for k = 1:n
    tk = t(k);

    % --- Discrete scenario events (a cut-in is a jump in the tracked target) --
    [x, pendingEvents] = applyDueEvents(x, tk, pendingEvents);

    gap  = x(1) - x(3);
    vRel = x(2) - x(4);

    % --- Controller update at the slower control rate -----------------------
    if mod(k - 1, stepsPerControl) == 0
        truth = struct('gap', gap, 'vEgo', x(4), 'vRel', vRel);
        [meas, sensor] = accSensorStep(sensor, truth, tk, p);
        meas.setSpeed = setSpeed;

        ttc = accTimeToCollision(meas.gap, meas.vRel);

        [aDes, mode, ctrl] = accControllerStep(ctrl, meas, p);
        gapRatio = meas.gap / max(ctrl.last.desiredGap, eps);
        [aApplied, satInfo] = accSaturation(aDes, aApplied, ttc, gapRatio, dtCtrl, p);
        ctrl = accControllerIntegrate(ctrl, aApplied, satInfo.isMagnitudeLimited, p);

        desiredGap = ctrl.last.desiredGap;
    end

    % --- Log ----------------------------------------------------------------
    log.sLead(k)      = x(1);
    log.vLead(k)      = x(2);
    log.sEgo(k)       = x(3);
    log.vEgo(k)       = x(4);
    log.aTrac(k)      = x(5);
    log.gap(k)        = gap;
    log.gapDesired(k) = desiredGap;
    log.gapMeas(k)    = meas.gap;
    log.vEgoMeas(k)   = meas.vEgo;
    log.vRelMeas(k)   = meas.vRel;
    log.vRel(k)       = vRel;
    log.aCommand(k)   = aApplied;
    log.aSpeedRaw(k)  = ctrl.last.aSpeedRaw;
    log.aGapRaw(k)    = ctrl.last.aSpacingRaw;
    log.speedIntegral(k)   = ctrl.speedIntegral;
    log.spacingIntegral(k) = ctrl.spacingIntegral;
    log.ttc(k)        = ttc;                              % as the controller sees it
    log.ttcTrue(k)    = accTimeToCollision(gap, vRel);    % as it actually is
    log.isSpacingMode(k) = (mode == "spacing");
    log.isSaturated(k)   = satInfo.isSaturated;
    log.isMagnitudeLimited(k) = satInfo.isMagnitudeLimited;
    log.isRateLimited(k)      = satInfo.isRateLimited;
    log.isEmergency(k)   = satInfo.isEmergency;

    % --- Integrate the plant one simulation step (RK4) -----------------------
    if k < n
        x = rk4Step(x, tk, dtSim, aApplied, scenario, p);
    end
end

out = log;
out.t = t;
out.scenario = scenario;
out.params = p;
out.setSpeed = setSpeed;
end

% -------------------------------------------------------------------------
function x = rk4Step(x, t, dt, aApplied, scenario, p)
f = @(tt, xx) accPlantDerivative(xx, scenario.leadAccel(tt, xx), aApplied, ...
                                 scenario.grade(tt), p);
k1 = f(t,          x);
k2 = f(t + dt/2,   x + dt/2 * k1);
k3 = f(t + dt/2,   x + dt/2 * k2);
k4 = f(t + dt,     x + dt   * k3);
x = x + dt/6 * (k1 + 2*k2 + 2*k3 + k4);

% Speeds are non-negative: neither vehicle reverses in this model.
x(2) = max(x(2), 0);
x(4) = max(x(4), 0);
end

% -------------------------------------------------------------------------
function [x, events] = applyDueEvents(x, t, events)
if isempty(events)
    return
end
isDue = [events.time] <= t;
for idx = find(isDue)
    x = events(idx).apply(x);
end
events(isDue) = [];
end

% -------------------------------------------------------------------------
function log = preallocateLog(n)
fields = {'sLead','vLead','sEgo','vEgo','aTrac','gap','gapDesired','gapMeas', ...
          'vEgoMeas','vRelMeas','vRel','aCommand','aSpeedRaw','aGapRaw', ...
          'speedIntegral','spacingIntegral','ttc','ttcTrue'};
log = struct();
for idx = 1:numel(fields)
    log.(fields{idx}) = zeros(n, 1);
end
log.isSpacingMode      = false(n, 1);
log.isSaturated        = false(n, 1);
log.isMagnitudeLimited = false(n, 1);
log.isRateLimited      = false(n, 1);
log.isEmergency        = false(n, 1);
end
