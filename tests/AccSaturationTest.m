classdef AccSaturationTest < matlab.unittest.TestCase
    %ACCSATURATIONTEST Comfort, capability and jerk limits (section 3.4).

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
            testCase.P.actuator.jerkMax = 1e6;            % isolate magnitude limits
            testCase.P.actuator.jerkMaxEmergency = 1e6;
        end
    end

    methods (Test)
        function comfortLimitsApplyRoutinely(testCase)
            [aSat, info] = accSaturation(-5, 0, Inf, 1.0, 0.02, testCase.P);
            testCase.verifyEqual(aSat, testCase.P.actuator.aMinComfort);
            testCase.verifyFalse(info.isEmergency);
            testCase.verifyTrue(info.isMagnitudeLimited);
        end

        function unsaturatedCommandPassesThrough(testCase)
            [aSat, info] = accSaturation(-1, 0, Inf, 1.0, 0.02, testCase.P);
            testCase.verifyEqual(aSat, -1);
            testCase.verifyFalse(info.isMagnitudeLimited);
            testCase.verifyFalse(info.isSaturated);
        end

        function lowTimeToCollisionUnlocksCapability(testCase)
            ttc = 0.5 * testCase.P.safety.ttcEmergency;
            [aSat, info] = accSaturation(-20, 0, ttc, 1.0, 0.02, testCase.P);
            testCase.verifyTrue(info.isEmergency);
            testCase.verifyEqual(aSat, testCase.P.actuator.aMinCapability);
        end

        function severeGapViolationUnlocksCapability(testCase)
            % A slow but severe encroachment never produces a small TTC, so the
            % gap ratio has to be an independent trigger.
            gapRatio = 0.5 * testCase.P.safety.gapRatioEmergency;
            [aSat, info] = accSaturation(-20, 0, Inf, gapRatio, 0.02, testCase.P);
            testCase.verifyTrue(info.isEmergency);
            testCase.verifyEqual(aSat, testCase.P.actuator.aMinCapability);
        end

        function jerkLimitBindsBeforeMagnitudeLimit(testCase)
            p0 = accParamsAutomotive();
            dt = 0.02;
            [aSat, info] = accSaturation(-2, 0, Inf, 1.0, dt, p0);
            testCase.verifyEqual(aSat, -p0.actuator.jerkMax * dt, 'AbsTol', 1e-12);
            testCase.verifyTrue(info.isRateLimited);
            testCase.verifyFalse(info.isMagnitudeLimited);
        end

        function emergencyAllowsFasterJerk(testCase)
            p0 = accParamsAutomotive();
            dt = 0.02;
            routine   = accSaturation(-8, 0, Inf, 1.0, dt, p0);
            emergency = accSaturation(-8, 0, 1.0, 1.0, dt, p0);
            testCase.verifyLessThan(emergency, routine, ...
                'Emergency braking must be allowed to build faster than comfort jerk.');
        end
    end
end
