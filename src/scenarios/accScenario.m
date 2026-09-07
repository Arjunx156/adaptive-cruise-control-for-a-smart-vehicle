function scenario = accScenario(name, p)
%ACCSCENARIO Build one scenario of the suite for a given parameter set.
%
%   scenario = accScenario(name, p)
%
%   The suite covers the six situations named in section 6.2 plus two additions:
%   gradeDisturbance isolates the integral action, and coldStart is the sustained
%   saturation case that makes integrator windup visible. See accScenarioNames.
%
%   Every scenario is expressed relative to the parameter set (set speed, spacing
%   policy, actuator limits), so the same definitions run at automotive scale and
%   at robot scale without change.
%
%   Fields:
%       tEnd       s      simulation length
%       x0         5x1    [sLead; vLead; sEgo; vEgo; aTrac]
%       setSpeed   m/s    driver set point
%       leadAccel  fcn    @(t, x) lead vehicle acceleration
%       grade      fcn    @(t) road inclination in radians
%       events     struct array of {time, apply} state jumps

name = string(name);
vSet = p.control.setSpeed;

% Scenario severity is scaled to the parameter set rather than fixed in absolute
% units. The natural time scale of a vehicle is its actuator lag, so a lead
% vehicle that stops in ten actuator time constants poses a comparable challenge
% at either scale -- and the physical braking limit still caps it. Fixing the
% deceleration in m/s^2 instead would make the robot's braking event roughly
% twelve times harsher than the car's, which would compare nothing useful.
aLeadBrake  = min(vSet / (10 * p.actuator.tau), 0.5 * abs(p.actuator.aMinCapability));
aLeadGentle = 0.5 * p.actuator.aMaxComfort;

% A grade that costs the same fraction of available authority at either scale.
gradeAccel = 0.25 * p.actuator.aMaxComfort;
gradeRad   = asin(min(gradeAccel / p.vehicle.gravity, 0.15));

scenario = struct( ...
    'name', name, ...
    'description', "", ...
    'tEnd', 60, ...
    'x0', zeros(5, 1), ...
    'setSpeed', vSet, ...
    'leadAccel', @(t, x) 0, ...
    'grade', @(t) 0, ...
    'events', emptyEvents());

switch name
    case "steadyFollow"
        % Section 6.2: the gap should be achieved and held, with no offset and
        % no hunting -- the spacing loop's integral action doing its job.
        vLead = 0.8 * vSet;
        scenario.description = "Following a constant-speed lead vehicle from the correct gap.";
        scenario.tEnd = 60;
        scenario.x0 = initialState(vLead, vLead, accSpacingPolicy(vLead, p), p);

    case "approach"
        % Transition from speed mode into spacing mode. The question is whether
        % the vehicle decelerates smoothly or overshoots into the lead vehicle.
        vLead = 0.6 * vSet;
        scenario.description = "Closing on a slower lead vehicle from a large initial gap.";
        scenario.tEnd = 90;
        scenario.x0 = initialState(vSet, vLead, 8 * accSpacingPolicy(vSet, p), p);

    case "leadBraking"
        % The safety-critical case. Tests response speed under acceleration
        % limits, and whether saturation and windup are handled correctly.
        vLead = 0.9 * vSet;
        scenario.description = "Lead vehicle brakes hard from steady following.";
        scenario.tEnd = 50;
        scenario.x0 = initialState(vLead, vLead, accSpacingPolicy(vLead, p), p);
        scenario.leadAccel = @(t, x) -aLeadBrake * inWindow(t, 10, 10 + vLead / aLeadBrake);

    case "leadAccelerating"
        % The reverse transition, back into speed mode. The controller must not
        % chase the lead vehicle past the driver's set speed.
        vLead = 0.6 * vSet;
        scenario.description = "Lead vehicle accelerates away above the set speed.";
        scenario.tEnd = 80;
        scenario.x0 = initialState(vLead, vLead, accSpacingPolicy(vLead, p), p);
        scenario.leadAccel = @(t, x) aLeadGentle * inWindow(t, 10, 10 + 0.7 * vSet / aLeadGentle);

    case "cutIn"
        % A vehicle appears in the lane ahead: a discontinuous step in measured
        % gap, and an aggressive disturbance rejection test.
        scenario.description = "A slower vehicle cuts in at half the desired gap.";
        scenario.tEnd = 100;
        scenario.x0 = initialState(vSet, vSet, 20 * accSpacingPolicy(vSet, p), p);
        cutInGap   = 0.5 * accSpacingPolicy(vSet, p);
        cutInSpeed = 0.85 * vSet;
        scenario.events = makeEvent(15, @(x) [x(3) + cutInGap; cutInSpeed; x(3); x(4); x(5)]);

    case "stopAndGo"
        % Low-speed stress: the time-gap policy produces very small targets, and
        % both sensor and actuator limitations are most pronounced here.
        vLead = 0.35 * vSet;
        scenario.description = "Repeated stop-and-go traffic cycles.";
        scenario.tEnd = 140;
        scenario.x0 = initialState(0, 0, accSpacingPolicy(0, p), p);
        scenario.leadAccel = @(t, x) stopAndGoProfile(t, x, vLead, aLeadGentle, aLeadBrake);

    case "coldStart"
        % Engaging cruise control from a standstill on an empty road. The speed
        % error stays large for tens of seconds while the command sits on the
        % comfort limit, which is the classic setting for integrator windup
        % (section 4.2): long saturation, then a demand to stop cleanly.
        scenario.description = "Accelerating to the set speed from rest on an empty road.";
        scenario.tEnd = 120;
        scenario.x0 = initialState(0, 3 * vSet, 1000 * accSpacingPolicy(vSet, p), p);

    case "gradeDisturbance"
        % Speed mode with no target in range. A sustained hill is a pure
        % disturbance rejection test for the integral term (sections 3.1, 4.1).
        scenario.description = sprintf("Cruising with no lead vehicle over a %.1f%% then -%.1f%% grade.", ...
                                       100 * tan(gradeRad), 100 * tan(gradeRad));
        scenario.tEnd = 120;
        scenario.x0 = initialState(vSet, 2 * vSet, 500 * accSpacingPolicy(vSet, p), p);
        scenario.grade = @(t) gradeRad * inWindow(t, 20, 60) - gradeRad * inWindow(t, 70, 110);

    otherwise
        error('accScenario:unknownName', ...
              'Unknown scenario "%s". Valid names: %s.', name, ...
              strjoin(cellstr(accScenarioNames()), ', '));
end
end

% -------------------------------------------------------------------------
function x0 = initialState(vEgo, vLead, gap, p)
%INITIALSTATE Place the vehicles and trim the actuator so t=0 is not a transient.
x0 = [gap; vLead; 0; vEgo; accTrimAcceleration(vEgo, 0, p)];
end

% -------------------------------------------------------------------------
function events = emptyEvents()
events = struct('time', {}, 'apply', {});
end

function events = makeEvent(time, applyFcn)
events = struct('time', time, 'apply', applyFcn);
end

% -------------------------------------------------------------------------
function y = inWindow(t, tStart, tEnd)
y = double(t >= tStart && t < tEnd);
end

% -------------------------------------------------------------------------
function a = stopAndGoProfile(t, x, vCruise, aAccel, aBrake)
%STOPANDGOPROFILE Accelerate, cruise, brake to a stop, wait, repeat.
period = 35;
phase  = mod(t, period);
vLead  = x(2);

if phase < 12
    a = aAccel * double(vLead < vCruise);
elseif phase < 20
    a = 0;
elseif phase < 30
    a = -aBrake;
else
    a = 0;
end
end
