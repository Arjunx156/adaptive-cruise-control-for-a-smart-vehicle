classdef AccReportingTest < matlab.unittest.TestCase
    %ACCREPORTINGTEST The figures and metrics that carry the results (section 6.3).
    %
    %   The plotting and metric code is a deliverable, not scaffolding: it is what
    %   anyone reading this project actually looks at. A figure that silently
    %   mislabels a mode, or a metric that reports a number from an empty road as
    %   though it meant something, is a defect like any other.

    properties
        P
    end

    methods (TestClassSetup)
        function setUpPath(~)
            accSetupPath();
        end
    end

    methods (TestMethodSetup)
        function loadParams(testCase)
            testCase.P = accParamsAutomotive();
        end
    end

    methods (Test)
        % --- Figures --------------------------------------------------------
        function resultFigureHasTheThreePanels(testCase)
            out = shortRun(testCase.P, "steadyFollow");
            fig = accPlotResults(out);
            testCase.addTeardown(@() close(fig));
            axesHandles = findobj(fig, 'Type', 'axes');
            testCase.verifyNumElements(axesHandles, 3, ...
                'Section 6.3 calls for velocities, separation and control effort.');
        end

        function spacingModeIntervalsAreShaded(testCase)
            out = shortRun(testCase.P, "steadyFollow");
            testCase.assumeTrue(any(out.isSpacingMode));
            fig = accPlotResults(out);
            testCase.addTeardown(@() close(fig));
            % findall, not findobj: the patches are deliberately handle-invisible
            % so they stay out of the legend.
            testCase.verifyNotEmpty(findall(fig, 'Type', 'patch'));
        end

        function shadingDoesNotDictateTheAxisLimits(testCase)
            % The shading is drawn after the data so it adopts the limits the
            % data established. Drawn before, it flattens every trace to a line.
            out = shortRun(testCase.P, "steadyFollow");
            fig = accPlotResults(out);
            testCase.addTeardown(@() close(fig));
            axesHandles = findobj(fig, 'Type', 'axes');
            for ax = axesHandles'
                % A y-range of order 1e6 would mean the patches set the limits.
                testCase.verifyLessThan(diff(ylim(ax)), 1e5);
            end
        end

        function figureWithoutSpacingModeDrawsNoShading(testCase)
            out = shortRun(testCase.P, "gradeDisturbance");
            testCase.assumeFalse(any(out.isSpacingMode));
            fig = accPlotResults(out);
            testCase.addTeardown(@() close(fig));
            testCase.verifyEmpty(findall(fig, 'Type', 'patch'));
        end

        function realisticSensingAddsTheMeasuredTrace(testCase)
            % The measured gap only belongs on the figure when it differs from
            % the true one.
            p0 = testCase.P;
            p0.sensor.mode = 'realistic';
            out = shortRun(p0, "steadyFollow");
            fig = accPlotResults(out);
            testCase.addTeardown(@() close(fig));
            names = string(get(findobj(fig, 'Type', 'line'), 'DisplayName'));
            testCase.verifyTrue(any(names == "measured gap"));
        end

        % --- Metrics --------------------------------------------------------
        function gapMetricsAreNotReportedFromAnEmptyRoad(testCase)
            out = shortRun(testCase.P, "gradeDisturbance");
            m = accMetrics(out);
            testCase.verifyTrue(isnan(m.gapErrorRms));
            testCase.verifyTrue(isnan(m.gapErrorPeak));
            testCase.verifyTrue(isnan(m.gapErrorFinal));
        end

        function collisionIsFlagged(testCase)
            out = shortRun(testCase.P, "steadyFollow");
            out.gap(round(end / 2)) = -1;
            m = accMetrics(out);
            testCase.verifyTrue(m.isCollision);
            testCase.verifyEqual(m.minGap, -1);
        end

        function jerkIsMeasuredOnTheControlRateNotTheSolverGrid(testCase)
            % The command is held between control updates. Differentiating it on
            % the 1 ms solver grid would report the height of the hold divided by
            % the solver step -- a number about twenty times too large.
            out = shortRun(testCase.P, "leadBraking");
            m = accMetrics(out);
            testCase.verifyLessThanOrEqual(m.maxAbsJerk, ...
                testCase.P.actuator.jerkMaxEmergency + 1e-6);
        end

        function saturationIsSplitByCause(testCase)
            out = shortRun(testCase.P, "leadBraking");
            m = accMetrics(out);
            testCase.verifyGreaterThanOrEqual(m.magLimitedFrac, 0);
            testCase.verifyLessThanOrEqual(m.magLimitedFrac, 1);
            testCase.verifyGreaterThanOrEqual(m.rateLimitedFrac, 0);
            testCase.verifyLessThanOrEqual(m.rateLimitedFrac, 1);
        end

        function overshootIsNeverNegative(testCase)
            % Never reaching the set speed is not negative overshoot.
            out = shortRun(testCase.P, "steadyFollow");
            m = accMetrics(out);
            testCase.verifyGreaterThanOrEqual(m.overshootAboveSet, 0);
        end

        % --- Parameter and path helpers -------------------------------------
        function bothParameterSetsLoadByName(testCase)
            testCase.verifyEqual(accLoadParams("automotive").name, 'automotive');
            testCase.verifyEqual(accLoadParams("robot").name, 'robot');
        end

        function unknownParameterSetErrors(testCase)
            testCase.verifyError(@() accLoadParams("hovercraft"), ...
                'accLoadParams:unknownSet');
        end

        function bothParameterSetsCarryTheSameFields(testCase)
            % The simulation core reads only from the parameter struct, so a
            % field present in one set and missing from the other is a latent
            % failure at the other scale.
            testCase.verifyEqual(sort(fieldnames(accParamsRobot())), ...
                                 sort(fieldnames(accParamsAutomotive())));
            for section = ["vehicle", "actuator", "spacing", "control", "safety", "sensor", "sim"]
                testCase.verifyEqual( ...
                    sort(fieldnames(accParamsRobot().(section))), ...
                    sort(fieldnames(accParamsAutomotive().(section))), ...
                    sprintf('Parameter sets disagree in the %s section.', section));
            end
        end

        function resultsDirectoryExists(testCase)
            testCase.verifyTrue(isfolder(accResultsDir()));
        end

        function scenarioNamesAreUniqueAndComplete(testCase)
            names = accScenarioNames();
            testCase.verifyEqual(numel(names), numel(unique(names)));
            testCase.verifyEqual(numel(names), 8);
        end
    end
end

% -------------------------------------------------------------------------
function out = shortRun(p, scenarioName)
%SHORTRUN A truncated scenario, long enough to exercise the reporting code.
scenario = accScenario(scenarioName, p);
scenario.tEnd = min(scenario.tEnd, 25);
out = accRunSimulation(scenario, p);
end
