function names = accScenarioNames()
%ACCSCENARIONAMES The scenario suite of section 6.2, in reporting order.
%
%   The first six are the scenarios named in section 6.2. The last two are
%   additions: gradeDisturbance isolates the integral action's disturbance
%   rejection, and coldStart is the sustained-saturation case that makes
%   integrator windup visible.
names = ["steadyFollow", "approach", "leadBraking", "leadAccelerating", ...
         "cutIn", "stopAndGo", "gradeDisturbance", "coldStart"];
end
