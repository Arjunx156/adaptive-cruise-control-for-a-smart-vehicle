classdef AccSensorTest < matlab.unittest.TestCase
    %ACCSENSORTEST Ultrasonic and encoder measurement model (section 5).

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
            testCase.P = accParamsRobot();
        end
    end

    methods (Test)
        function idealModePassesTruthThrough(testCase)
            p0 = testCase.P;
            p0.sensor.mode = 'ideal';
            sensor = accSensorInit(p0, 0.5, 0.3);
            truth = struct('gap', 0.5, 'vEgo', 0.3, 'vRel', -0.1);
            meas = accSensorStep(sensor, truth, 0, p0);
            testCase.verifyEqual(meas.gap, truth.gap);
            testCase.verifyEqual(meas.vEgo, truth.vEgo);
            testCase.verifyEqual(meas.vRel, truth.vRel);
        end

        function realisticModeRespectsRangeBounds(testCase)
            % Section 5.1: targets beyond the maximum range are simply not seen.
            p0 = testCase.P;
            p0.sensor.mode = 'realistic';
            [gaps, ~] = runSensor(p0, 10 * p0.sensor.rangeMax, 0.3, 6.0);
            testCase.verifyLessThanOrEqual(max(gaps), p0.sensor.rangeMax + 1e-9);
        end

        function realisticModeConvergesOnAStaticTarget(testCase)
            % Noise and quantisation should average out; the estimate must not
            % drift away from a stationary target.
            p0 = testCase.P;
            p0.sensor.mode = 'realistic';
            trueGap = 0.60;
            [gaps, ~] = runSensor(p0, trueGap, 0.3, 8.0);
            settled = gaps(round(0.5 * numel(gaps)):end);
            testCase.verifyEqual(mean(settled), trueGap, 'AbsTol', 0.05);
        end

        function realisticModeIsReproducible(testCase)
            p0 = testCase.P;
            p0.sensor.mode = 'realistic';
            first  = runSensor(p0, 0.6, 0.3, 3.0);
            second = runSensor(p0, 0.6, 0.3, 3.0);
            testCase.verifyEqual(first, second, ...
                'A fixed seed must make sensor noise reproducible.');
        end

        function encoderSpeedIsNoisierNearStandstill(testCase)
            % Section 5.3: fewer pulses accumulate per sampling interval at low
            % speed, so the speed estimate is worse exactly where the spacing
            % targets are smallest.
            p0 = testCase.P;
            p0.sensor.mode = 'realistic';
            [~, slow] = runSensor(p0, 0.6, 0.005, 8.0);
            [~, fast] = runSensor(p0, 0.6, p0.control.setSpeed, 8.0);
            testCase.verifyGreaterThan(std(slow - 0.005), std(fast - p0.control.setSpeed));
        end

        function measurementIsHeldBetweenSensorSamples(testCase)
            % The rangefinder cannot be sampled as fast as the controller runs.
            p0 = testCase.P;
            p0.sensor.mode = 'realistic';
            gaps = runSensor(p0, 0.6, 0.3, 1.0);
            stepsPerSample = round((1 / p0.sensor.rangeRate) / p0.control.dt);
            testCase.verifyGreaterThan(stepsPerSample, 1);
            changes = sum(abs(diff(gaps)) > 0);
            testCase.verifyLessThan(changes, numel(gaps) / 2, ...
                'The range estimate should update slower than the control loop.');
        end
    end
end

% -------------------------------------------------------------------------
function [gaps, speeds] = runSensor(p, trueGap, trueVEgo, duration)
sensor = accSensorInit(p, min(trueGap, p.sensor.rangeMax), trueVEgo);
truth = struct('gap', trueGap, 'vEgo', trueVEgo, 'vRel', 0);
n = round(duration / p.control.dt);
gaps = zeros(n, 1);
speeds = zeros(n, 1);
for k = 1:n
    [meas, sensor] = accSensorStep(sensor, truth, (k - 1) * p.control.dt, p);
    gaps(k) = meas.gap;
    speeds(k) = meas.vEgo;
end
end
