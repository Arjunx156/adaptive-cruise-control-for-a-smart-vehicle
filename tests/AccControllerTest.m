classdef AccControllerTest < matlab.unittest.TestCase
    %ACCCONTROLLERTEST The PID control law, arbitration and anti-windup (section 4).

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
        function spacingPolicyScalesWithSpeed(testCase)
            % Section 2.2: constant time gap, plus a standstill offset so the
            % target does not collapse to zero when stopped.
            p0 = testCase.P;
            testCase.verifyEqual(accSpacingPolicy(0, p0), p0.spacing.standstillGap);
            testCase.verifyEqual(accSpacingPolicy(20, p0), ...
                p0.spacing.standstillGap + 20 * p0.spacing.timeGap, 'AbsTol', 1e-12);
        end

        function spacingPolicyNeverGoesNegative(testCase)
            testCase.verifyEqual(accSpacingPolicy(-5, testCase.P), ...
                testCase.P.spacing.standstillGap);
        end

        function clearRoadSelectsSpeedMode(testCase)
            % An enormous gap makes the spacing demand large and positive, so the
            % min-selector must fall through to the speed loop.
            [~, mode] = stepOnce(testCase.P, 20, 5000, 0, 25);
            testCase.verifyEqual(mode, "speed");
        end

        function closingOnSlowerVehicleSelectsSpacingMode(testCase)
            [~, mode] = stepOnce(testCase.P, 25, 10, -8, 25);
            testCase.verifyEqual(mode, "spacing");
        end

        function selectorTakesTheSmallerDemand(testCase)
            [aDes, ~, ctrl] = stepOnce(testCase.P, 25, 10, -8, 25);
            testCase.verifyEqual(aDes, min(ctrl.last.aSpeedRaw, ctrl.last.aSpacingRaw));
        end

        function proportionalActionAloneLeavesSteadyStateOffset(testCase)
            % Section 4.1: a nonzero error is required to generate the force that
            % balances drag and rolling resistance.
            p0 = testCase.P;
            p0.control.speed.Ki = 0;
            p0.control.spacing.Ki = 0;
            out = runCruise(p0);
            finalError = out.setSpeed - out.vEgo(end);
            testCase.verifyGreaterThan(abs(finalError), 0.1, ...
                'P-only cruise control should retain a visible speed offset.');
        end

        function integralActionRemovesSteadyStateOffset(testCase)
            out = runCruise(testCase.P);
            finalError = out.setSpeed - out.vEgo(end);
            testCase.verifyLessThan(abs(finalError), 0.02, ...
                'PI cruise control should hold the set speed.');
        end

        function integralActionRejectsSustainedGrade(testCase)
            % Section 3.1: the reason a cruise control recovers on a hill.
            p0 = testCase.P;
            scenario = cruiseScenario(p0, 200);
            scenario.grade = @(t) atan(0.04) * double(t > 20);
            out = accRunSimulation(scenario, p0);
            testCase.verifyLessThan(abs(out.setSpeed - out.vEgo(end)), 0.05);
        end

        function conditionalIntegrationFreezesWhenMagnitudeLimited(testCase)
            p0 = testCase.P;
            p0.control.antiWindupMethod = 'conditional';
            ctrl = accControllerInit(p0, 0);
            ctrl.last.speedError = 20;
            ctrl.last.isSpeedSelected = true;
            ctrl.last.isSpacingSelected = false;
            before = ctrl.speedIntegral;
            frozen = accControllerIntegrate(ctrl, -8, true, p0);
            free   = accControllerIntegrate(ctrl, -8, false, p0);
            testCase.verifyEqual(frozen.speedIntegral, before);
            testCase.verifyGreaterThan(free.speedIntegral, before);
        end

        function unselectedLoopDoesNotIntegrate(testCase)
            % The second windup source: a loop that lost the min-selection must
            % not charge up while inactive.
            p0 = testCase.P;
            ctrl = accControllerInit(p0, 0);
            ctrl.last.speedError = 20;
            ctrl.last.isSpeedSelected = false;
            ctrl.last.isSpacingSelected = true;
            after = accControllerIntegrate(ctrl, 0, false, p0);
            testCase.verifyEqual(after.speedIntegral, ctrl.speedIntegral);
        end

        function integratorIsClampedToPhysicalAuthority(testCase)
            p0 = testCase.P;
            p0.control.antiWindupMethod = 'none';
            ctrl = accControllerInit(p0, 0);
            ctrl.last.speedError = 1e4;
            ctrl.last.isSpeedSelected = true;
            for k = 1:5000
                ctrl = accControllerIntegrate(ctrl, 0, true, p0);
            end
            testCase.verifyLessThanOrEqual(ctrl.speedIntegral, p0.actuator.aMaxCapability);
        end

        function disablingAntiWindupWindsTheIntegratorUp(testCase)
            % Section 4.2: without mitigation the integrator accumulates error
            % the actuator never acted on.
            p0 = testCase.P;
            p0.control.antiWindupMethod = 'none';
            withoutAw = integralAfterSaturatedSamples(p0);
            p0.control.antiWindupMethod = 'conditional';
            withAw = integralAfterSaturatedSamples(p0);
            testCase.verifyGreaterThan(withoutAw, 10 * withAw + 0.1);
        end

        function unknownAntiWindupMethodErrors(testCase)
            p0 = testCase.P;
            p0.control.antiWindupMethod = 'wishfulThinking';
            ctrl = accControllerInit(p0, 0);
            testCase.verifyError(@() accControllerIntegrate(ctrl, 0, false, p0), ...
                'accControllerIntegrate:unknownMethod');
        end

        function timeToCollisionIsInfiniteWhenOpening(testCase)
            testCase.verifyEqual(accTimeToCollision(50, 5), Inf);
            testCase.verifyEqual(accTimeToCollision(50, 0), Inf);
            testCase.verifyEqual(accTimeToCollision(50, -10), 5, 'AbsTol', 1e-12);
        end
    end
end

% -------------------------------------------------------------------------
function [aDes, mode, ctrl] = stepOnce(p, vEgo, gap, vRel, setSpeed)
ctrl = accControllerInit(p, vEgo);
meas = struct('gap', gap, 'vEgo', vEgo, 'vRel', vRel, 'setSpeed', setSpeed);
[aDes, mode, ctrl] = accControllerStep(ctrl, meas, p);
end

function scenario = cruiseScenario(p, tEnd)
%CRUISESCENARIO Speed mode only: the lead vehicle is far away and faster.
vStart = 0.8 * p.control.setSpeed;
scenario = accScenario("steadyFollow", p);
scenario.tEnd = tEnd;
scenario.x0 = [1e5; 2 * p.control.setSpeed; 0; vStart; accTrimAcceleration(vStart, 0, p)];
scenario.leadAccel = @(t, x) 0;
end

function out = runCruise(p)
out = accRunSimulation(cruiseScenario(p, 150), p);
end

function value = integralAfterSaturatedSamples(p)
ctrl = accControllerInit(p, 0);
ctrl.last.speedError = 20;
ctrl.last.isSpeedSelected = true;
ctrl.last.isSpacingSelected = false;
for k = 1:500
    ctrl = accControllerIntegrate(ctrl, p.actuator.aMaxComfort, true, p);
end
value = ctrl.speedIntegral;
end
