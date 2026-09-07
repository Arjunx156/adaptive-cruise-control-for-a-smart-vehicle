function [meas, sensor] = accSensorStep(sensor, truth, t, p)
%ACCSENSORSTEP Produce the measurements the controller actually sees.
%
%   [meas, sensor] = accSensorStep(sensor, truth, t, p)
%
%   truth.gap, truth.vEgo, truth.vRel are the exact values from the plant.
%
%   In 'ideal' mode they are passed straight through, which isolates control
%   design from sensing (section 6.1). In 'realistic' mode the ultrasonic
%   limitations of section 5.1 are applied -- latency, a slower update rate than
%   the control loop, quantisation, noise, dropouts, spurious short readings and
%   a bounded range -- and relative velocity is no longer measured at all. It has
%   to be estimated from a noisy range signal, which is exactly the constraint
%   that shapes the derivative term.

if strcmp(sensor.mode, 'ideal')
    meas = struct('gap', truth.gap, 'vEgo', truth.vEgo, 'vRel', truth.vRel, ...
                  'isValid', true);
    sensor.gapMeas  = truth.gap;
    sensor.vEgoMeas = truth.vEgo;
    sensor.gapHat   = truth.gap;
    sensor.vRelHat  = truth.vRel;
    return
end

% --- Transport delay ---------------------------------------------------------
sensor.delayGap  = [sensor.delayGap(2:end),  truth.gap];
sensor.delayVEgo = [sensor.delayVEgo(2:end), truth.vEgo];
delayedGap  = sensor.delayGap(1);
delayedVEgo = sensor.delayVEgo(1);

% --- Encoder speed: quantised, and noisier at low speed (section 5.3) --------
lowSpeedScale = exp(-abs(delayedVEgo) / max(0.1 * p.control.setSpeed, eps));
speedStd = p.sensor.speedNoiseStd + ...
           (p.sensor.speedNoiseLow - p.sensor.speedNoiseStd) * lowSpeedScale;
vEgoRaw  = delayedVEgo + speedStd * randn(sensor.rng);
sensor.vEgoMeas = max(0, quantise(vEgoRaw, p.sensor.speedQuant));

% --- Ultrasonic range: sampled slower than the control loop, held between ----
isNewSample = t >= sensor.nextSampleTime - 1e-12;
if isNewSample
    sensor.nextSampleTime = sensor.nextSampleTime + sensor.samplePeriod;

    isDropout = rand(sensor.rng) < p.sensor.dropoutProb;
    isOutOfRange = delayedGap > p.sensor.rangeMax;

    if ~isDropout
        rawGap = delayedGap + p.sensor.rangeNoiseStd * randn(sensor.rng);

        % A spurious echo from a nearer reflector reads short, never long.
        if rand(sensor.rng) < p.sensor.spikeProb
            rawGap = rawGap - p.sensor.spikeMag * rand(sensor.rng);
        end

        rawGap = min(max(rawGap, p.sensor.rangeMin), p.sensor.rangeMax);
        rawGap = quantise(rawGap, p.sensor.rangeQuant);

        if isOutOfRange
            rawGap = p.sensor.rangeMax;   % no target in view
        end

        sensor.medianBuffer = [sensor.medianBuffer(2:end), rawGap];
    end

    % Median filter rejects the spikes that survived; the low-pass smooths what
    % is left. Both cost phase margin, which is the trade of section 5.2.
    gapMedian = median(sensor.medianBuffer);
    alphaGap = sensor.samplePeriod / (p.sensor.gapFilterTf + sensor.samplePeriod);
    sensor.gapFiltered = sensor.gapFiltered + alphaGap * (gapMedian - sensor.gapFiltered);

    % Alpha-beta tracker: the only source of relative velocity in this mode.
    dtS = sensor.samplePeriod;
    gapPredicted = sensor.gapHat + sensor.vRelHat * dtS;
    residual = sensor.gapFiltered - gapPredicted;
    sensor.gapHat  = gapPredicted + p.sensor.alpha * residual;
    sensor.vRelHat = sensor.vRelHat + p.sensor.beta * residual / dtS;
end

sensor.gapMeas = sensor.gapHat;

meas = struct( ...
    'gap',     sensor.gapHat, ...
    'vEgo',    sensor.vEgoMeas, ...
    'vRel',    sensor.vRelHat, ...
    'isValid', sensor.gapHat < p.sensor.rangeMax - 1e-9);
end

function y = quantise(x, step)
if step <= 0
    y = x;
else
    y = round(x / step) * step;
end
end
