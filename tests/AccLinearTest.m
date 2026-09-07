classdef AccLinearTest < matlab.unittest.TestCase
    %ACCLINEARTEST Linearised models and stability margins (sections 3.2, 4.3).

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
        function dampingMatchesDragSlopeAtTheOperatingPoint(testCase)
            v0 = 25;
            models = accLinearModels(testCase.P, v0);
            vh = testCase.P.vehicle;
            expected = vh.airDensity * vh.dragCoeff * vh.frontalArea * v0;
            testCase.verifyEqual(models.b, expected, 'RelTol', 1e-12);
        end

        function speedPlantDcGainIsSetByDrag(testCase)
            models = accLinearModels(testCase.P, 25);
            testCase.verifyEqual(dcgain(models.Pv), 1 / models.dragPole, 'RelTol', 1e-9);
        end

        function spacingPlantContainsAnIntegrator(testCase)
            % Section 2.3: the gap is the integral of relative velocity, so the
            % spacing plant must have a pole at the origin.
            models = accLinearModels(testCase.P, 25);
            testCase.verifyTrue(any(abs(pole(models.Pgap)) < 1e-9));
        end

        function bothLoopsMeetThePhaseMarginSpecification(testCase)
            models = accLinearModels(testCase.P, 25);
            report = accMarginReport(models, 'Verbose', false);
            testCase.verifyGreaterThanOrEqual(report.speed.phaseMargin, 45);
            testCase.verifyGreaterThanOrEqual(report.spacing.phaseMargin, 45);
            testCase.verifyTrue(report.isPassing);
        end

        function spacingLoopMatchesItsStateSpaceConstruction(testCase)
            % Two independent derivations of the same closed loop must agree.
            % This is the check that catches a feedback-sign error in the
            % transfer function algebra.
            models = accLinearModels(testCase.P, 25);
            fromTf = sort(pole(models.Tspacing));
            fromSs = sort(eig(models.spacingClosedLoopSS));
            testCase.verifyEqual(fromTf, fromSs, 'AbsTol', 1e-8, ...
                'Transfer function and state-space closed loops disagree.');
        end

        function bothClosedLoopsAreStable(testCase)
            models = accLinearModels(testCase.P, 25);
            testCase.verifyLessThan(max(real(pole(models.Tspeed))), 0);
            testCase.verifyLessThan(max(real(pole(models.Tspacing))), 0);
        end

        function linearModelPredictsNonlinearResponseNearOperatingPoint(testCase)
            % Section 3.2: the approximation is honest only near v0.
            errorNear = openLoopStepError(testCase.P, 25, 25, 0.5);
            errorFar  = openLoopStepError(testCase.P, 25,  8, 0.5);
            testCase.verifyLessThan(errorNear, 0.05, ...
                'Linear model should track the plant near its operating point.');
            testCase.verifyGreaterThan(errorFar, errorNear, ...
                'Accuracy should degrade away from the linearisation point.');
        end
    end
end

% -------------------------------------------------------------------------
function err = openLoopStepError(p, v0, vStart, aStep)
%OPENLOOPSTEPERROR RMS speed difference between the linear and nonlinear plants.
duration = 20;
models = accLinearModels(p, v0);

t = (0:p.sim.dt:duration)';
vLinear = vStart + lsim(models.Pv, aStep * ones(size(t)), t);

x = [1e6; 0; 0; vStart; accTrimAcceleration(vStart, 0, p)];
vNonlinear = zeros(size(t));
aCmd = x(5) + aStep;
for k = 1:numel(t)
    vNonlinear(k) = x(4);
    dx = accPlantDerivative(x, 0, aCmd, 0, p);
    x = x + p.sim.dt * dx;
end

err = sqrt(mean((vLinear - vNonlinear).^2)) / max(abs(vNonlinear - vStart));
end
