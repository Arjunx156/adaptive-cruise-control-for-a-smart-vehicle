function metricsTable = runAllScenarios(paramSet, sensorMode, opts)
%RUNALLSCENARIOS Run the full section 6.2 scenario suite and save figures + metrics.
%
%   metricsTable = runAllScenarios()                       % automotive, ideal sensing
%   metricsTable = runAllScenarios("robot", "realistic")
%
%   Writes one PNG per scenario and one CSV of metrics into results/.

arguments
    paramSet   (1,1) string = "automotive"
    sensorMode (1,1) string = "ideal"
    opts.SaveFigures (1,1) logical = true
    opts.Verbose     (1,1) logical = true
end

accSetupPath();
p = accLoadParams(paramSet);
p.sensor.mode = char(sensorMode);

names = accScenarioNames();
metrics = cell(1, numel(names));

for idx = 1:numel(names)
    scenario = accScenario(names(idx), p);
    out = accRunSimulation(scenario, p);
    metrics{idx} = accMetrics(out);

    if opts.SaveFigures
        fig = accPlotResults(out);
        outFile = fullfile(accResultsDir(), ...
            sprintf('%s_%s_%s.png', paramSet, sensorMode, names(idx)));
        exportgraphics(fig, outFile, 'Resolution', 150);
        close(fig);
    end

    if opts.Verbose
        fprintf('  %-18s min gap %7.2f m   min TTC %6.2f s   gap RMS %6.3f m   at limit %4.1f%%\n', ...
            names(idx), metrics{idx}.minGap, metrics{idx}.minTtc, ...
            metrics{idx}.gapErrorRms, 100 * metrics{idx}.magLimitedFrac);
    end
end

metricsTable = struct2table([metrics{:}]);
csvFile = fullfile(accResultsDir(), sprintf('metrics_%s_%s.csv', paramSet, sensorMode));
writetable(metricsTable, csvFile);

if opts.Verbose
    fprintf('\nWrote %s\n', csvFile);
end
end
