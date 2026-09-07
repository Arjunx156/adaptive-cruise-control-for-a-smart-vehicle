function [aSat, info] = accSaturation(aDes, aPrev, ttc, gapRatio, dt, p)
%ACCSATURATION Apply magnitude and jerk limits to the acceleration command.
%
%   [aSat, info] = accSaturation(aDes, aPrev, ttc, gapRatio, dt, p)
%
%   gapRatio is measuredGap / desiredGap.
%
%   Two limit sets are defined (section 3.4). Comfort limits apply routinely.
%   When time-to-collision falls below p.safety.ttcEmergency the capability
%   limits are unlocked, so hard braking is available but reserved for genuine
%   emergencies. A jerk rate limit smooths the command in both cases.
%
%   info.isEmergency  true when the capability limits are in force
%   info.isSaturated  true when either limit actually bound the command

% Two independent triggers. Time-to-collision catches a fast approach while the
% gap is still large; the gap ratio catches a slow but severe encroachment that
% never produces a small TTC. Either one unlocks the capability limits.
isEmergency = (ttc < p.safety.ttcEmergency) || (gapRatio < p.safety.gapRatioEmergency);

if isEmergency
    aMin = p.actuator.aMinCapability;
    aMax = p.actuator.aMaxCapability;
else
    aMin = p.actuator.aMinComfort;
    aMax = p.actuator.aMaxComfort;
end

aMagLimited = min(max(aDes, aMin), aMax);

% Jerk limit relative to the previously applied command. Comfort jerk applies
% routinely; an emergency is allowed to build braking effort far faster, which is
% the whole point of holding that authority in reserve.
if isEmergency
    jerkStep = p.actuator.jerkMaxEmergency * dt;
else
    jerkStep = p.actuator.jerkMax * dt;
end
aSat = min(max(aMagLimited, aPrev - jerkStep), aPrev + jerkStep);

% The two limits are reported separately because they mean different things.
% Magnitude limiting says the vehicle cannot deliver what was asked and is the
% condition that causes windup. Rate limiting only says the command is being
% smoothed, which is routine and must not be allowed to stall the integrator.
info = struct( ...
    'isEmergency',        isEmergency, ...
    'isMagnitudeLimited', abs(aMagLimited - aDes) > 1e-9, ...
    'isRateLimited',      abs(aSat - aMagLimited) > 1e-9, ...
    'isSaturated',        abs(aSat - aDes) > 1e-9, ...
    'aMin', aMin, ...
    'aMax', aMax);
end
