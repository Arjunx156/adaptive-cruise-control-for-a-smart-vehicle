function [aDes, mode, ctrl] = accControllerStep(ctrl, meas, p)
%ACCCONTROLLERSTEP One sample of the two-loop ACC control law (section 4).
%
%   [aDes, mode, ctrl] = accControllerStep(ctrl, meas, p)
%
%   meas.vEgo      m/s   measured (or estimated) ego speed
%   meas.gap       m     measured (or estimated) separation
%   meas.vRel      m/s   estimated relative speed, vLead - vEgo
%   meas.setSpeed  m/s   driver set point
%
%   Both loops run every sample and the smaller of the two accelerations is
%   commanded. There is no threshold and no discrete switch, so there is nothing
%   for the system to chatter against at the mode boundary (section 2.1). The
%   returned mode is a label for plotting only; it never gates the arithmetic.
%
%   Integrators are NOT advanced here. accControllerIntegrate does that once the
%   applied (saturated) acceleration is known, so anti-windup can back-calculate
%   against it.

dt = p.control.dt;

% --- Filtered derivative of ego speed ---------------------------------------
% Used as the ego acceleration estimate. Derivative acts on the measurement
% rather than the error, so a set-point change produces no derivative kick.
if ~ctrl.isInitialised
    ctrl.vEgoPrev = meas.vEgo;
    ctrl.isInitialised = true;
end
rawDerivative = (meas.vEgo - ctrl.vEgoPrev) / dt;
alpha = dt / (p.control.speed.Tf + dt);
ctrl.egoAccelEstimate = ctrl.egoAccelEstimate + alpha * (rawDerivative - ctrl.egoAccelEstimate);
ctrl.vEgoPrev = meas.vEgo;

% --- Speed loop (cruise control) --------------------------------------------
speedError = meas.setSpeed - meas.vEgo;
aSpeedRaw  = p.control.speed.Kp * speedError ...
           + ctrl.speedIntegral ...
           - p.control.speed.Kd * ctrl.egoAccelEstimate;

% --- Spacing loop (gap regulation) ------------------------------------------
desiredGap   = accSpacingPolicy(meas.vEgo, p);
spacingError = meas.gap - desiredGap;

% Derivative action on closing velocity (section 4.1). This is deliberately NOT
% the exact derivative of the spacing error: that would be
%     vRel - timeGap * aEgo
% and the second term feeds the controller's own braking back positively. With
% Kd*timeGap of order one it cancels a large fraction of the braking demand at
% precisely the moment hard braking is called for, which turns the lead-braking
% scenario into a collision. Closing velocity alone is both the physically
% meaningful signal and the well-behaved one.
spacingErrorRate = meas.vRel;

aSpacingRaw = p.control.spacing.Kp * spacingError ...
            + ctrl.spacingIntegral ...
            + p.control.spacing.Kd * spacingErrorRate;

% --- Min-selector arbitration ------------------------------------------------
if aSpacingRaw < aSpeedRaw
    aDes = aSpacingRaw;
    mode = "spacing";
else
    aDes = aSpeedRaw;
    mode = "speed";
end

ctrl.last = struct( ...
    'speedError',        speedError, ...
    'spacingError',      spacingError, ...
    'aSpeedRaw',         aSpeedRaw, ...
    'aSpacingRaw',       aSpacingRaw, ...
    'desiredGap',        desiredGap, ...
    'isSpeedSelected',   mode == "speed", ...
    'isSpacingSelected', mode == "spacing");
end
