function report = runLinearAnalysis(paramSet, opts)
%RUNLINEARANALYSIS Frequency-domain analysis of both control loops (sections 3.2, 4.3).
%
%   report = runLinearAnalysis()
%   report = runLinearAnalysis("robot")
%
%   Prints stability margins, saves Bode and root-locus figures, and quantifies
%   how far from the linearisation point the linear model stays trustworthy.

arguments
    paramSet (1,1) string = "automotive"
    opts.SaveFigures (1,1) logical = true
end

accSetupPath();
p = accLoadParams(paramSet);
v0 = p.control.setSpeed;

models = accLinearModels(p, v0);
report = accMarginReport(models);
report.models = models;

if ~opts.SaveFigures
    return
end

% --- Bode plots with margins -------------------------------------------------
fig = figure('Visible', 'off', 'Position', [100 100 1000 720]);
subplot(1, 2, 1);
margin(models.Lspeed);
grid on;
title('Speed loop');

subplot(1, 2, 2);
margin(models.Lspacing);
grid on;
title('Spacing loop');

sgtitle(sprintf('Loop gain and stability margins (%s, v0 = %.1f m/s)', paramSet, v0));
exportgraphics(fig, fullfile(accResultsDir(), sprintf('%s_bode.png', paramSet)), ...
               'Resolution', 150);
close(fig);

% --- Root locus of the spacing loop ------------------------------------------
% The spacing plant carries the integrator of section 2.3 on top of the vehicle
% and actuator poles, which is why derivative action is what makes the approach
% converge rather than oscillate.
fig = figure('Visible', 'off', 'Position', [100 100 700 620]);
rlocus(models.Lspacing);
grid on;
title(sprintf('Spacing loop root locus (%s)', paramSet));
exportgraphics(fig, fullfile(accResultsDir(), sprintf('%s_rlocus.png', paramSet)), ...
               'Resolution', 150);
close(fig);

% --- Validity of the linearisation -------------------------------------------
fig = plotLinearisationValidity(p, models);
exportgraphics(fig, fullfile(accResultsDir(), sprintf('%s_linearisation.png', paramSet)), ...
               'Resolution', 150);
close(fig);

fprintf('\nWrote Bode, root locus and linearisation figures to %s\n', accResultsDir());
end

% -------------------------------------------------------------------------
function fig = plotLinearisationValidity(p, models)
%PLOTLINEARISATIONVALIDITY Linear against nonlinear open-loop step response.
%
%   Section 3.2: the approximation is honest only near the operating point at
%   which it was taken. Running the same acceleration step from the nominal speed
%   and from well below it shows where that honesty runs out.

v0 = models.v0;
startSpeeds = [v0, 0.6 * v0, 0.25 * v0];
aStep = 0.4;
duration = 30;
t = (0:p.sim.dt:duration)';

fig = figure('Visible', 'off', 'Position', [100 100 860 560]);
hold on;
colours = lines(numel(startSpeeds));

for idx = 1:numel(startSpeeds)
    vStart = startSpeeds(idx);
    vLinear = vStart + lsim(models.Pv, aStep * ones(size(t)), t);
    vNonlinear = simulateOpenLoop(p, vStart, aStep, t);

    plot(t, vNonlinear, '-',  'Color', colours(idx, :), 'LineWidth', 1.6, ...
         'DisplayName', sprintf('nonlinear, from %.0f m/s', vStart));
    plot(t, vLinear,    '--', 'Color', colours(idx, :), 'LineWidth', 1.2, ...
         'DisplayName', sprintf('linear, from %.0f m/s', vStart));
end

xlabel('time (s)');
ylabel('speed (m/s)');
title(sprintf('Linearisation validity: identical %.1f m/s^2 step from three speeds', aStep));
subtitle(sprintf('Model linearised at %.1f m/s; agreement degrades away from it', v0));
legend('Location', 'best');
grid on;
end

% -------------------------------------------------------------------------
function v = simulateOpenLoop(p, vStart, aStep, t)
x = [1e6; 0; 0; vStart; accTrimAcceleration(vStart, 0, p)];
aCmd = x(5) + aStep;
v = zeros(size(t));
for k = 1:numel(t)
    v(k) = x(4);
    x = x + p.sim.dt * accPlantDerivative(x, 0, aCmd, 0, p);
end
end
