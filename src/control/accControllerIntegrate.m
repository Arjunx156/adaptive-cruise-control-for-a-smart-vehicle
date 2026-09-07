function ctrl = accControllerIntegrate(ctrl, aApplied, isSaturated, p)
%ACCCONTROLLERINTEGRATE Advance both integrators with anti-windup (section 4.2).
%
%   ctrl = accControllerIntegrate(ctrl, aApplied, isSaturated, p)
%
%   aApplied is the acceleration actually sent to the plant after saturation and
%   jerk limiting; isSaturated says whether either limit bound the command.
%
%   A two-loop ACC has two distinct windup sources, and both must be handled:
%
%     1. Actuator saturation. The commanded acceleration exceeds what the vehicle
%        can deliver, so the integrator accumulates error the system cannot act
%        on. This is the classic case of section 4.2.
%
%     2. Losing the min-selection. The loop that did not win still sees its own
%        error every sample. Left alone it charges up while inactive and dumps
%        that charge the instant it takes over.
%
%   Both reduce to the same rule: a loop may integrate only when it is actually
%   in command and actually free to act.
%
%   p.control.antiWindupMethod selects the strategy:
%     'conditional'     freeze the integrator otherwise (default)
%     'backCalculation' drive it toward the applied acceleration with time
%                       constant Tt, sized as sqrt(Ti*Td) per loop
%     'none'            integrate unconditionally -- the failure case, kept so
%                       the comparison in scripts/runWindupComparison can be made

dt = p.control.dt;
last = ctrl.last;

isSpeedFree   = last.isSpeedSelected   && ~isSaturated;
isSpacingFree = last.isSpacingSelected && ~isSaturated;

switch p.control.antiWindupMethod
    case 'none'
        speedGrowth   = p.control.speed.Ki   * last.speedError;
        spacingGrowth = p.control.spacing.Ki * last.spacingError;

    case 'conditional'
        speedGrowth   = p.control.speed.Ki   * last.speedError   * isSpeedFree;
        spacingGrowth = p.control.spacing.Ki * last.spacingError * isSpacingFree;

    case 'backCalculation'
        speedGrowth   = p.control.speed.Ki   * last.speedError ...
                      + (aApplied - last.aSpeedRaw)   / p.control.speed.Tt;
        spacingGrowth = p.control.spacing.Ki * last.spacingError ...
                      + (aApplied - last.aSpacingRaw) / p.control.spacing.Tt;

    otherwise
        error('accControllerIntegrate:unknownMethod', ...
              'Unknown anti-windup method "%s".', p.control.antiWindupMethod);
end

ctrl.speedIntegral   = ctrl.speedIntegral   + dt * speedGrowth;
ctrl.spacingIntegral = ctrl.spacingIntegral + dt * spacingGrowth;

% Hard bound regardless of method: an integrator asking for more authority than
% the vehicle physically has is asking for something it can never be given.
ctrl.speedIntegral   = clampToAuthority(ctrl.speedIntegral, p);
ctrl.spacingIntegral = clampToAuthority(ctrl.spacingIntegral, p);
end

function value = clampToAuthority(value, p)
value = min(max(value, p.actuator.aMinCapability), p.actuator.aMaxCapability);
end
