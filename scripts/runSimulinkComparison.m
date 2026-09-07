function comparison = runSimulinkComparison(scenarioName, paramSet, opts)
%RUNSIMULINKCOMPARISON Simulate the Simulink model and compare it to the .m run.
%
%   comparison = runSimulinkComparison()
%   comparison = runSimulinkComparison("leadBraking", "automotive")
%
%   Both routes integrate the same equations with the same fixed-step RK4 solver
%   and share one control law, so they should agree closely. Where they do not,
%   the difference is a modelling error in one of the two -- which is precisely
%   what a parity check is for.

arguments
    scenarioName (1,1) string = "leadBraking"
    paramSet     (1,1) string = "automotive"
    opts.SaveFigures (1,1) logical = true
end

accSetupPath();
p = accLoadParams(paramSet);
p.sensor.mode = 'ideal';   % the Simulink model measures the plant directly
scenario = accScenario(scenarioName, p);

reference = accRunSimulation(scenario, p);
simulink = simulateModel(scenario, p);

% Resample the Simulink output onto the reference time grid before comparing.
gapSim  = interp1(simulink.t, simulink.gap,  reference.t, 'linear', 'extrap');
vEgoSim = interp1(simulink.t, simulink.vEgo, reference.t, 'linear', 'extrap');

comparison = struct( ...
    'scenario',     scenarioName, ...
    'paramSet',     paramSet, ...
    'gapRmsError',  sqrt(mean((gapSim  - reference.gap ).^2)), ...
    'gapMaxError',  max(abs(gapSim  - reference.gap)), ...
    'speedRmsError', sqrt(mean((vEgoSim - reference.vEgo).^2)), ...
    'speedMaxError', max(abs(vEgoSim - reference.vEgo)), ...
    'reference',    reference, ...
    'simulink',     simulink);

fprintf('\nSimulink vs MATLAB parity, %s (%s):\n', scenarioName, paramSet);
fprintf('  gap    RMS %.4f m     max %.4f m\n', comparison.gapRmsError, comparison.gapMaxError);
fprintf('  speed  RMS %.4f m/s   max %.4f m/s\n', comparison.speedRmsError, comparison.speedMaxError);

if opts.SaveFigures
    fig = plotComparison(reference, gapSim, vEgoSim, scenarioName, paramSet);
    outFile = fullfile(accResultsDir(), sprintf('%s_simulink_parity.png', paramSet));
    exportgraphics(fig, outFile, 'Resolution', 150);
    close(fig);
    fprintf('  wrote %s\n', outFile);
end
end

% =========================================================================
function out = simulateModel(scenario, p)
%SIMULATEMODEL Build the model if needed, stage its inputs, and run it.

modelName = 'accModel';
if ~bdIsLoaded(modelName)
    buildAccSimulinkModel('ModelName', modelName);
end

% The lead acceleration and road grade are time series rather than functions
% inside the model, so the scenario definitions stay in one place.
t = (0:p.sim.dt:scenario.tEnd)';
aLeadValues = zeros(size(t));
gradeValues = zeros(size(t));
x = scenario.x0(:);
for k = 1:numel(t)
    aLeadValues(k) = scenario.leadAccel(t(k), x);
    gradeValues(k) = scenario.grade(t(k));
end

assignin('base', 'accParams', p);
assignin('base', 'accStopTime', scenario.tEnd);
assignin('base', 'accInitial', struct( ...
    'sLead', scenario.x0(1), 'vLead', scenario.x0(2), ...
    'sEgo',  scenario.x0(3), 'vEgo',  scenario.x0(4), ...
    'aTrac', scenario.x0(5)));
assignin('base', 'aLeadInput', timeseries(aLeadValues, t));
assignin('base', 'gradeInput', timeseries(gradeValues, t));

AccControllerRuntime.reset(p, scenario.x0(4), scenario.x0(1) - scenario.x0(3));

simOut = sim(modelName, 'ReturnWorkspaceOutputs', 'on');

out = struct( ...
    't',       simOut.simGap.time, ...
    'gap',     squeeze(simOut.simGap.signals.values), ...
    'vEgo',    squeeze(simOut.simVEgo.signals.values), ...
    'vLead',   squeeze(simOut.simVLead.signals.values), ...
    'command', squeeze(simOut.simCommand.signals.values));
end

% =========================================================================
function fig = plotComparison(reference, gapSim, vEgoSim, scenarioName, paramSet)
fig = figure('Visible', 'off', 'Position', [100 100 900 700]);
tl = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('Simulink and MATLAB agreement: %s (%s)', scenarioName, paramSet), ...
      'Interpreter', 'none', 'FontWeight', 'bold');

ax = nexttile(tl);
plot(ax, reference.t, reference.gap, 'LineWidth', 1.8, 'DisplayName', 'MATLAB');
hold(ax, 'on');
plot(ax, reference.t, gapSim, '--', 'LineWidth', 1.3, 'DisplayName', 'Simulink');
ylabel(ax, 'separation (m)');
legend(ax, 'Location', 'best');
grid(ax, 'on');

ax = nexttile(tl);
plot(ax, reference.t, reference.vEgo, 'LineWidth', 1.8, 'DisplayName', 'MATLAB');
hold(ax, 'on');
plot(ax, reference.t, vEgoSim, '--', 'LineWidth', 1.3, 'DisplayName', 'Simulink');
ylabel(ax, 'ego speed (m/s)');
grid(ax, 'on');

ax = nexttile(tl);
plot(ax, reference.t, gapSim - reference.gap, 'LineWidth', 1.4);
ylabel(ax, 'gap difference (m)');
xlabel(ax, 'time (s)');
grid(ax, 'on');
end
