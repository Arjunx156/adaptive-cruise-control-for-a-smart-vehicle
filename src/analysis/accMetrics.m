function m = accMetrics(out)
%ACCMETRICS Quantitative summary of one simulation run (section 6.3).
%
%   Good tracking bought with saturated, oscillating control effort is not a
%   successful design, so the effort and saturation figures are reported
%   alongside the tracking figures rather than beneath them.
%
%   Two conventions matter for reading the table:
%     * Regulation quality (gapErrorRms, speedErrorRms) is measured over the
%       final quarter of the run, i.e. after the transient the scenario staged.
%       Peak error over the whole run is reported separately.
%     * Gap figures are NaN when the run never entered spacing mode, rather than
%       reporting a meaningless number from an empty road.

t  = out.t;
dt = t(2) - t(1);
p  = out.params;

isSpacing = out.isSpacingMode;
isSpeed   = ~isSpacing;
isSettled = t >= 0.75 * t(end);

gapError   = out.gap - out.gapDesired;
speedError = out.setSpeed - out.vEgo;

% Jerk must be measured on the control-rate command, not on the simulation grid:
% a zero-order-hold step differentiated at 1 ms would report the hold, not the
% physical rate of change.
stepsPerControl = max(1, round(p.control.dt / dt));
aControl = out.aCommand(1:stepsPerControl:end);
jerk = diff(aControl) / p.control.dt;

m = struct();
m.scenario        = string(out.scenario.name);
m.paramSet        = string(p.name);
m.sensorMode      = string(p.sensor.mode);

m.minGap          = min(out.gap);
m.isCollision     = m.minGap <= 0;
% Safety is judged on what actually happened, not on what the noisy sensor
% reported. The measured TTC is a controller input, not a verdict.
m.minTtc          = min(out.ttcTrue);
m.ttcBreachFrac   = mean(out.ttcTrue < p.safety.ttcCritical);

m.gapErrorRms     = rmsOver(gapError, isSpacing & isSettled);
m.gapErrorPeak    = maxAbsOver(gapError, isSpacing);
m.gapErrorFinal   = valueIf(gapError(end), isSpacing(end));
m.speedErrorRms   = rmsOver(speedError, isSpeed & isSettled);
m.overshootAboveSet = max(0, max(out.vEgo) - out.setSpeed);

m.maxAccel        = max(out.aCommand);
m.maxDecel        = min(out.aCommand);
m.maxAbsJerk      = max(abs(jerk));
m.magLimitedFrac  = mean(out.isMagnitudeLimited);
m.rateLimitedFrac = mean(out.isRateLimited);
m.emergencyFrac   = mean(out.isEmergency);
m.spacingModeFrac = mean(isSpacing);
end

% -------------------------------------------------------------------------
function r = rmsOver(signal, mask)
if ~any(mask)
    r = NaN;
else
    r = sqrt(mean(signal(mask).^2));
end
end

function v = maxAbsOver(signal, mask)
if ~any(mask)
    v = NaN;
else
    v = max(abs(signal(mask)));
end
end

function v = valueIf(value, condition)
if condition
    v = value;
else
    v = NaN;
end
end
