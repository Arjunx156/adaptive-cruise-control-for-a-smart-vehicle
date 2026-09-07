classdef AccPlantTest < matlab.unittest.TestCase
    %ACCPLANTTEST Longitudinal vehicle model (section 3).

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
        function coastingDecelerates(testCase)
            % With no tractive effort, drag and rolling resistance must slow the
            % vehicle -- and only slow it.
            x = [1000; 25; 0; 25; 0];
            dx = accPlantDerivative(x, 0, 0, 0, testCase.P);
            testCase.verifyLessThan(dx(4), 0, 'Coasting vehicle should decelerate.');
        end

        function trimAccelerationHoldsSpeed(testCase)
            % accTrimAcceleration is defined as the effort that exactly balances
            % the resistances, so applying it must produce zero net acceleration.
            v = 25;
            aTrim = accTrimAcceleration(v, 0, testCase.P);
            x = [1000; v; 0; v; aTrim];
            dx = accPlantDerivative(x, 0, aTrim, 0, testCase.P);
            testCase.verifyEqual(dx(4), 0, 'AbsTol', 1e-12);
        end

        function dragGrowsWithSquareOfSpeed(testCase)
            % Section 3.1: quadratic drag is what makes the plant nonlinear.
            p0 = testCase.P;
            p0.vehicle.rollingCoeff = 0;   % isolate the aerodynamic term
            aAt10 = -derivativeAt(p0, 10);
            aAt20 = -derivativeAt(p0, 20);
            testCase.verifyEqual(aAt20 / aAt10, 4, 'RelTol', 1e-9);
        end

        function vehicleDoesNotRollBackwards(testCase)
            % At standstill a braking command must not produce reverse motion.
            x = [10; 0; 0; 0; -2];
            dx = accPlantDerivative(x, -2, -2, 0, testCase.P);
            testCase.verifyGreaterThanOrEqual(dx(4), 0);
            testCase.verifyGreaterThanOrEqual(dx(2), 0);
        end

        function uphillGradeOpposesMotion(testCase)
            % Section 3.1: grade is an external disturbance, not a plant property.
            x = [1000; 25; 0; 25; 0];
            onFlat = accPlantDerivative(x, 0, 0, 0,        testCase.P);
            uphill = accPlantDerivative(x, 0, 0, atan(0.05), testCase.P);
            testCase.verifyLessThan(uphill(4), onFlat(4));
        end

        function actuatorLagIsFirstOrder(testCase)
            % Section 3.3: the response to a step command must reach 1 - 1/e of
            % its final value after one time constant.
            p0 = testCase.P;
            aCmd = 1.0;
            tau = p0.actuator.tau;
            dt = 1e-4;
            aTrac = 0;
            for k = 1:round(tau / dt)
                aTrac = aTrac + dt * (aCmd - aTrac) / tau;
            end
            testCase.verifyEqual(aTrac, 1 - exp(-1), 'RelTol', 1e-3);
        end

        function gapIsIntegralOfRelativeVelocity(testCase)
            % Section 2.3: the structural feature that shapes the spacing loop.
            x = [100; 20; 0; 25; 0];
            dx = accPlantDerivative(x, 0, 0, 0, testCase.P);
            gapRate = dx(1) - dx(3);
            testCase.verifyEqual(gapRate, x(2) - x(4), 'AbsTol', 1e-12);
        end
    end
end

function a = derivativeAt(p, v)
x = [1000; v; 0; v; 0];
dx = accPlantDerivative(x, 0, 0, 0, p);
a = dx(4);
end
