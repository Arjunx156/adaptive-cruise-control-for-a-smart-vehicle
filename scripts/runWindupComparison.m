function summary = runWindupComparison(paramSet, opts)
%RUNWINDUPCOMPARISON Show what integrator windup costs (section 4.2).
%
%   summary = runWindupComparison()
%
%   Runs a scenario that holds the actuator in saturation for a long time, three
%   times with different anti-windup strategies, and reports the difference
%   numerically as well as graphically.
%
%   The default scenario is coldStart rather than leadBraking. Braking hard is
%   the case section 4.2 points at, but in this design the spacing loop's
%   authority during braking comes almost entirely from the derivative term, and
%   its integral gain is small enough that a few seconds of saturation barely
%   charges it. Windup needs sustained saturation with a large persistent error,
%   which is what accelerating to the set speed from rest provides.

arguments
    paramSet (1,1) string = "automotive"
    opts.Scenario (1,1) string = "coldStart"
    opts.SaveFigures (1,1) logical = true
end

accSetupPath();
p = accLoadParams(paramSet);
methods = {'none', 'backCalculation', 'conditional'};
labels  = ["no anti-windup", "back-calculation", "conditional integration"];

runs = cell(1, numel(methods));
rows = cell(1, numel(methods));

for idx = 1:numel(methods)
    pRun = p;
    pRun.control.antiWindupMethod = methods{idx};
    scenario = accScenario(opts.Scenario, pRun);
    out = accRunSimulation(scenario, pRun);
    runs{idx} = out;

    m = accMetrics(out);
    rows{idx} = struct( ...
        'method',          string(methods{idx}), ...
        'minGap',          m.minGap, ...
        'isCollision',     m.isCollision, ...
        'peakSpeedIntegral',   max(abs(out.speedIntegral)), ...
        'peakSpacingIntegral', max(abs(out.spacingIntegral)), ...
        'speedOvershoot',      max(0, max(out.vEgo) - out.setSpeed), ...
        'settleTime',          settlingTime(out));
end

summary = struct2table([rows{:}]);
disp(summary);

if opts.SaveFigures
    fig = plotComparison(runs, labels, opts.Scenario, paramSet);
    outFile = fullfile(accResultsDir(), sprintf('%s_windup_comparison.png', paramSet));
    exportgraphics(fig, outFile, 'Resolution', 150);
    close(fig);
    fprintf('\nWrote %s\n', outFile);
end
end

% -------------------------------------------------------------------------
function seconds = settlingTime(out)
%SETTLINGTIME Last moment the speed is still outside a 1%% band of the set point.
%
%   This is the quantity windup damages. The accumulated charge has to be unwound
%   before the controller responds correctly again, so windup shows up as a long
%   tail after the event rather than as a larger peak during it.
tolerance = 0.01 * out.setSpeed;
isOutside = abs(out.vEgo - out.setSpeed) > tolerance;
lastOutside = find(isOutside, 1, 'last');
if isempty(lastOutside)
    seconds = 0;
else
    seconds = out.t(lastOutside);
end
end

% -------------------------------------------------------------------------
function fig = plotComparison(runs, labels, scenarioName, paramSet)
fig = figure('Visible', 'off', 'Position', [100 100 900 780]);
tl = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, sprintf('Integrator windup during %s (%s parameters)', scenarioName, paramSet), ...
      'Interpreter', 'none', 'FontWeight', 'bold');

ax = nexttile(tl);
hold(ax, 'on');
for idx = 1:numel(runs)
    plot(ax, runs{idx}.t, runs{idx}.vEgo, 'LineWidth', 1.5, ...
         'DisplayName', labels(idx));
end
yline(ax, runs{1}.setSpeed, '--', 'set speed', 'Color', [0.4 0.4 0.4], ...
      'HandleVisibility', 'off');
ylabel(ax, 'ego speed (m/s)');
legend(ax, 'Location', 'best');
grid(ax, 'on');

ax = nexttile(tl);
hold(ax, 'on');
for idx = 1:numel(runs)
    plot(ax, runs{idx}.t, runs{idx}.speedIntegral, 'LineWidth', 1.5, ...
         'DisplayName', labels(idx));
end
ylabel(ax, 'speed integrator (m/s^2)');
grid(ax, 'on');

ax = nexttile(tl);
hold(ax, 'on');
for idx = 1:numel(runs)
    plot(ax, runs{idx}.t, runs{idx}.aCommand, 'LineWidth', 1.5, ...
         'DisplayName', labels(idx));
end
ylabel(ax, 'applied command (m/s^2)');
xlabel(ax, 'time (s)');
grid(ax, 'on');
end
