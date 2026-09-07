function ctrl = accControllerInit(p, vEgo0)
%ACCCONTROLLERINIT Create the ACC controller state struct.
%
%   ctrl = accControllerInit(p, vEgo0)
%
%   The controller is stateful across samples (two integrators and two
%   derivative filters) but is written in a functional style: every step returns
%   a new state struct rather than mutating the one it was given.

if nargin < 2
    vEgo0 = 0;
end

ctrl = struct();
ctrl.speedIntegral   = 0;    % m/s^2 contributed by the speed loop integrator
ctrl.spacingIntegral = 0;    % m/s^2 contributed by the spacing loop integrator
ctrl.egoAccelEstimate = 0;   % m/s^2, filtered derivative of measured ego speed
ctrl.vEgoPrev        = vEgo0;
ctrl.isInitialised   = false;

% Populated by accControllerStep, consumed by accControllerIntegrate.
ctrl.last = struct( ...
    'speedError',        0, ...
    'spacingError',      0, ...
    'aSpeedRaw',         0, ...
    'aSpacingRaw',       0, ...
    'desiredGap',        accSpacingPolicy(vEgo0, p), ...
    'isSpeedSelected',   true, ...
    'isSpacingSelected', false);
end
