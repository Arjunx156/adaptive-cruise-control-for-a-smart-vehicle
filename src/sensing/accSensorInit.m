function sensor = accSensorInit(p, gap0, vEgo0)
%ACCSENSORINIT Create the sensor and estimator state (section 5).

if nargin < 2, gap0  = 0; end
if nargin < 3, vEgo0 = 0; end

sensor = struct();
sensor.mode = p.sensor.mode;
sensor.rng  = RandStream('threefry', 'Seed', p.sensor.seed);

sensor.samplePeriod   = 1 / p.sensor.rangeRate;
sensor.nextSampleTime = 0;

% Transport delay buffer, indexed at the control rate.
nDelay = max(1, round(p.sensor.rangeLatency / p.control.dt) + 1);
sensor.delayGap  = repmat(gap0,  1, nDelay);
sensor.delayVEgo = repmat(vEgo0, 1, nDelay);

% Median window for outlier rejection.
sensor.medianBuffer = repmat(gap0, 1, p.sensor.medianWindow);

% Estimator states.
sensor.gapFiltered = gap0;
sensor.gapHat      = gap0;
sensor.vRelHat     = 0;
sensor.vEgoMeas    = vEgo0;
sensor.gapMeas     = gap0;
end
