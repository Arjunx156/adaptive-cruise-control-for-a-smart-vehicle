function models = accLinearModels(p, v0)
%ACCLINEARMODELS Linearised plant and loop transfer functions (section 3.2).
%
%   models = accLinearModels(p, v0)
%
%   Linearising about a nominal cruising speed v0 collapses the quadratic drag
%   term into linear damping, b = rho*Cd*A*v0, and the vehicle becomes a mass
%   against velocity-proportional friction. Rolling resistance contributes
%   nothing to the perturbation model: tanh(v/vEps) is already saturated at any
%   sensible cruising speed, so its slope is effectively zero.
%
%   This approximation is honest only near v0. runLinearAnalysis quantifies how
%   far from v0 it stays useful.
%
%   Returned fields:
%       b           N/(m/s)  linearised damping coefficient
%       Pv          tf       ego speed        <- acceleration command
%       Pgap        tf       separation       <- acceleration command
%       Cspeed      tf       speed loop controller
%       Lspeed      tf       speed loop gain
%       Lspacing    tf       spacing loop gain
%       Tspeed      tf       closed-loop speed response
%       Tspacing    tf       closed-loop gap response

arguments
    p  struct
    v0 (1,1) double = p.control.setSpeed
end

s = tf('s');
vh = p.vehicle;

b = vh.airDensity * vh.dragCoeff * vh.frontalArea * v0;   % d(dragForce)/dv
lambda = b / vh.mass;                                     % 1/s, drag pole

% --- Plant -------------------------------------------------------------------
actuator = 1 / (p.actuator.tau * s + 1);      % section 3.3
Pv   = actuator / (s + lambda);               % speed from acceleration command
Pgap = -Pv / s;                               % the gap is the integral of vRel (2.3)

% --- Speed loop --------------------------------------------------------------
cs = p.control.speed;
Cspeed = cs.Kp + cs.Ki / s + cs.Kd * s / (cs.Tf * s + 1);
Lspeed = minreal(Cspeed * Pv);

% --- Spacing loop ------------------------------------------------------------
% The spacing controller is not a single-input PID: the error itself depends on
% ego speed through the time-gap policy, and the derivative acts on relative
% velocity. Written against the measured pair (gap, speed) the controller is a
% row vector, and the loop gain is its product with the corresponding plant
% column.
cd = p.control.spacing;
tg = p.spacing.timeGap;
Kgap   = cd.Kp + cd.Ki / s;
Kspeed = -(tg * (cd.Kp + cd.Ki / s) + cd.Kd);

% The negation matters. Pgap already carries the sign of the feedback path (the
% gap shrinks when the ego vehicle accelerates), so Kgap*Pgap + Kspeed*Pv is the
% loop with the feedback sign folded in. margin and feedback both assume the
% convention 1 + L, so the quantity they need is its negative. Getting this wrong
% reports a stable design as unstable.
Lspacing = minreal(-(Kgap * Pgap + Kspeed * Pv));

% An independent construction of the same closed loop, assembled directly from
% the differential equations rather than from transfer function algebra. Its
% poles must match those of Tspacing; the test that compares them is what catches
% a sign or convention error in the block-diagram route above.
lambda_ = lambda;
tau = p.actuator.tau;
A = [      0,                    -1,        0,      0;
           0,              -lambda_,        1,      0;
     cd.Kp/tau, (-cd.Kp*tg - cd.Kd)/tau, -1/tau, 1/tau;
       cd.Ki,             -cd.Ki*tg,        0,      0];
spacingClosedLoopSS = ss(A, [0; 0; 0; 0], [1 0 0 0], 0, ...
    'StateName', {'gap', 'speed', 'accel', 'integral'});

models = struct( ...
    'v0', v0, ...
    'b', b, ...
    'dragPole', lambda, ...
    'Pv', Pv, ...
    'Pgap', Pgap, ...
    'Cspeed', Cspeed, ...
    'Lspeed', Lspeed, ...
    'Lspacing', Lspacing, ...
    'Tspeed', minreal(feedback(Lspeed, 1)), ...
    'Tspacing', minreal(feedback(Lspacing, 1)), ...
    'spacingClosedLoopSS', spacingClosedLoopSS);
end
