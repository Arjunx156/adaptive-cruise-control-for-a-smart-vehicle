function runAll()
%RUNALL Regenerate every result in results/ from scratch.
%
%   matlab -batch "accSetupPath; runAll"
%
%   Runs the scenario suite at both scales with both sensing modes, the
%   frequency-domain analysis, the anti-windup comparison and the Simulink
%   parity check. Takes several minutes.

accSetupPath();
started = tic;

banner('Scenario suite');
for paramSet = ["automotive", "robot"]
    for sensorMode = ["ideal", "realistic"]
        fprintf('\n%s, %s sensing:\n', paramSet, sensorMode);
        runAllScenarios(paramSet, sensorMode);
    end
end

banner('Frequency-domain analysis');
runLinearAnalysis("automotive");
runLinearAnalysis("robot");

banner('Anti-windup comparison');
runWindupComparison("automotive");

banner('Simulink parity');
buildAccSimulinkModel();
runSimulinkComparison("leadBraking", "automotive");

banner(sprintf('Done in %.1f minutes', toc(started) / 60));
fprintf('All results are in %s\n', accResultsDir());
end

function banner(text)
fprintf('\n%s\n%s\n%s\n', repmat('=', 1, 68), text, repmat('=', 1, 68));
end
