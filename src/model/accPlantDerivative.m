function dx = accPlantDerivative(x, aLead, aDesSat, gradeRad, p)
%ACCPLANTDERIVATIVE Nonlinear longitudinal derivative for the two-vehicle system.
%
%   dx = accPlantDerivative(x, aLead, aDesSat, gradeRad, p)
%
%   State vector:
%       x(1) sLead   m     lead vehicle position
%       x(2) vLead   m/s   lead vehicle speed
%       x(3) sEgo    m     ego vehicle position
%       x(4) vEgo    m/s   ego vehicle speed
%       x(5) aTrac   m/s^2 achieved tractive acceleration (actuator output)
%
%   Inputs:
%       aLead     m/s^2  lead vehicle acceleration demanded by the scenario
%       aDesSat   m/s^2  ego acceleration command, already saturated
%       gradeRad  rad    road inclination (positive = uphill)
%
%   Aerodynamic drag, rolling resistance and grade are deliberately left OUT of
%   the controller's model, so they act as genuine disturbances it has to reject
%   (sections 3.1 and 4.1). The controller commands an acceleration; the actuator
%   lag (section 3.3) sits between that command and the force actually produced.

vLead = x(2);
vEgo  = x(4);
aTrac = x(5);

v = p.vehicle;

% Resistances expressed as decelerations (force divided by mass).
aDrag  = 0.5 * v.airDensity * v.dragCoeff * v.frontalArea * vEgo * abs(vEgo) / v.mass;
aRoll  = v.rollingCoeff * v.gravity * tanh(vEgo / v.stictionSpeed);
aGrade = v.gravity * sin(gradeRad);

aEgo = aTrac - aDrag - aRoll - aGrade;

% Neither vehicle may roll backwards: below zero speed, decelerations are held off.
aLead = clampAtStandstill(vLead, aLead);
aEgo  = clampAtStandstill(vEgo,  aEgo);

dx = [vLead; aLead; vEgo; aEgo; (aDesSat - aTrac) / p.actuator.tau];
end

function a = clampAtStandstill(v, a)
if v <= 0 && a < 0
    a = 0;
end
end
