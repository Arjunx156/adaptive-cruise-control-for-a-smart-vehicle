classdef AccScenarioTest < matlab.unittest.TestCase
    %ACCSCENARIOTEST End-to-end behaviour of the section 6.2 scenario suite.
    %
    %   These are the tests that decide whether the design is acceptable. The
    %   safety assertion -- that no scenario ever closes the gap to zero -- is the
    %   one that matters most, so it runs against every scenario in the suite.

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
        function everyScenarioBuilds(testCase)
            for name = accScenarioNames()
                scenario = accScenario(name, testCase.P);
                testCase.verifyEqual(scenario.name, name);
                testCase.verifySize(scenario.x0, [5 1]);
                testCase.verifyGreaterThan(scenario.tEnd, 0);
            end
        end

        function unknownScenarioNameErrors(testCase)
            testCase.verifyError(@() accScenario("teleportation", testCase.P), ...
                'accScenario:unknownName');
        end

        function noScenarioEverCollides(testCase)
            % The safety requirement, checked across the whole suite.
            for name = accScenarioNames()
                out = accRunSimulation(accScenario(name, testCase.P), testCase.P);
                testCase.verifyGreaterThan(min(out.gap), 0, ...
                    sprintf('Scenario %s closed the gap to zero.', name));
            end
        end

        function scenariosAlsoRunAtRobotScale(testCase)
            % The same definitions must work against the other parameter set.
            pRobot = accParamsRobot();
            for name = accScenarioNames()
                out = accRunSimulation(accScenario(name, pRobot), pRobot);
                testCase.verifyGreaterThan(min(out.gap), 0, ...
                    sprintf('Robot scenario %s closed the gap to zero.', name));
            end
        end

        function steadyFollowingHoldsTheDesiredGapWithoutOffset(testCase)
            % Section 6.2: confirms the integral action works in spacing mode.
            out = accRunSimulation(accScenario("steadyFollow", testCase.P), testCase.P);
            settled = out.t >= 0.5 * out.t(end);
            gapError = out.gap(settled) - out.gapDesired(settled);
            testCase.verifyLessThan(max(abs(gapError)), 0.05);
        end

        function steadyFollowingDoesNotHunt(testCase)
            % No oscillating acceleration command in the steady case.
            out = accRunSimulation(accScenario("steadyFollow", testCase.P), testCase.P);
            settled = out.t >= 0.5 * out.t(end);
            testCase.verifyLessThan(std(out.aCommand(settled)), 1e-3);
        end

        function approachEntersSpacingModeAndSettles(testCase)
            out = accRunSimulation(accScenario("approach", testCase.P), testCase.P);
            testCase.verifyTrue(any(out.isSpacingMode), 'Should enter spacing mode.');
            testCase.verifyTrue(out.isSpacingMode(end), 'Should still be following at the end.');
            testCase.verifyLessThan(abs(out.gap(end) - out.gapDesired(end)), 1.0);
        end

        function approachDoesNotOvershootIntoTheLeadVehicle(testCase)
            % Section 6.2: the question the approach scenario exists to answer.
            out = accRunSimulation(accScenario("approach", testCase.P), testCase.P);
            testCase.verifyGreaterThan(min(out.gap), 0.5 * min(out.gapDesired));
        end

        function leadBrakingStaysWithinCapabilityLimits(testCase)
            % Good tracking bought with impossible control effort is not a design.
            out = accRunSimulation(accScenario("leadBraking", testCase.P), testCase.P);
            testCase.verifyGreaterThanOrEqual(min(out.aCommand), ...
                testCase.P.actuator.aMinCapability - 1e-9);
            testCase.verifyLessThanOrEqual(max(out.aCommand), ...
                testCase.P.actuator.aMaxCapability + 1e-9);
        end

        function leadBrakingUnlocksEmergencyAuthority(testCase)
            % Comfort limits alone are not enough for this case.
            out = accRunSimulation(accScenario("leadBraking", testCase.P), testCase.P);
            testCase.verifyTrue(any(out.isEmergency));
            testCase.verifyLessThan(min(out.aCommand), testCase.P.actuator.aMinComfort);
        end

        function leadAcceleratingReturnsToSpeedModeWithoutChasing(testCase)
            % Section 6.2: the controller must not follow the lead vehicle past
            % the driver's set speed.
            out = accRunSimulation(accScenario("leadAccelerating", testCase.P), testCase.P);
            testCase.verifyFalse(out.isSpacingMode(end), 'Should end in speed mode.');
            testCase.verifyLessThan(max(out.vEgo), out.setSpeed + 0.1);
            testCase.verifyLessThan(abs(out.vEgo(end) - out.setSpeed), 0.05);
        end

        function cutInProducesAStepChangeInGap(testCase)
            % The scenario is only a disturbance test if the gap really jumps.
            out = accRunSimulation(accScenario("cutIn", testCase.P), testCase.P);
            gapJump = max(abs(diff(out.gap)));
            testCase.verifyGreaterThan(gapJump, 10);
        end

        function cutInRecoversToTheDesiredGap(testCase)
            out = accRunSimulation(accScenario("cutIn", testCase.P), testCase.P);
            testCase.verifyLessThan(abs(out.gap(end) - out.gapDesired(end)), 1.0);
        end

        function stopAndGoNeverFallsBelowTheStandstillGap(testCase)
            % Section 6.2: the low-speed case, where the time-gap policy produces
            % very small targets.
            out = accRunSimulation(accScenario("stopAndGo", testCase.P), testCase.P);
            testCase.verifyGreaterThanOrEqual(min(out.gap), ...
                0.9 * testCase.P.spacing.standstillGap);
        end

        function gradeDisturbanceStaysInSpeedMode(testCase)
            out = accRunSimulation(accScenario("gradeDisturbance", testCase.P), testCase.P);
            testCase.verifyFalse(any(out.isSpacingMode), ...
                'An empty road should never engage the spacing loop.');
        end

        function realisticSensingDoesNotCauseACollision(testCase)
            % Section 5.2: filtering costs phase margin, but the design must still
            % survive the sensor it will actually be given.
            p0 = testCase.P;
            p0.sensor.mode = 'realistic';
            for name = ["leadBraking", "cutIn", "stopAndGo"]
                out = accRunSimulation(accScenario(name, p0), p0);
                testCase.verifyGreaterThan(min(out.gap), 0, ...
                    sprintf('Scenario %s collided with realistic sensing.', name));
            end
        end

        function metricsCoverEveryReportedField(testCase)
            out = accRunSimulation(accScenario("steadyFollow", testCase.P), testCase.P);
            m = accMetrics(out);
            expected = {'scenario', 'paramSet', 'sensorMode', 'minGap', 'isCollision', ...
                        'minTtc', 'gapErrorRms', 'gapErrorPeak', 'maxAccel', 'maxDecel', ...
                        'maxAbsJerk', 'magLimitedFrac', 'rateLimitedFrac'};
            for idx = 1:numel(expected)
                testCase.verifyTrue(isfield(m, expected{idx}), ...
                    sprintf('Metrics missing field %s.', expected{idx}));
            end
        end
    end
end
