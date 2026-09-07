function aTrim = accTrimAcceleration(v, gradeRad, p)
%ACCTRIMACCELERATION Tractive acceleration needed to hold speed v on a given grade.
%
%   This is the value the integrator must reach to eliminate steady-state error
%   (section 4.1): exactly enough tractive effort to balance drag, rolling
%   resistance and gravity.

vh = p.vehicle;
aDrag  = 0.5 * vh.airDensity * vh.dragCoeff * vh.frontalArea * v * abs(v) / vh.mass;
aRoll  = vh.rollingCoeff * vh.gravity * tanh(v / vh.stictionSpeed);
aGrade = vh.gravity * sin(gradeRad);
aTrim  = aDrag + aRoll + aGrade;
end
