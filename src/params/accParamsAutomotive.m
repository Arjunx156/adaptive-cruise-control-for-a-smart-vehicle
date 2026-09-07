function p = accParamsAutomotive()
%ACCPARAMSAUTOMOTIVE Highway-scale parameter set for the ACC simulation.
%
%   Values are standard passenger-car figures (mid-size sedan). Every quantity
%   the simulation needs lives in this struct; nothing is hard-coded downstream,
%   so accParamsRobot returns the same fields at a different scale.
%
%   See docs/adaptive_cruise_control_project.md sections 3 and 4.

p = struct();
p.name = 'automotive';

% --- Vehicle (section 3.1) ---------------------------------------------------
p.vehicle.mass          = 1500;    % kg
p.vehicle.dragCoeff     = 0.32;    % Cd, dimensionless
p.vehicle.frontalArea   = 2.2;     % m^2
p.vehicle.airDensity    = 1.225;   % kg/m^3
p.vehicle.rollingCoeff  = 0.015;   % Cr, dimensionless
p.vehicle.gravity       = 9.81;    % m/s^2
p.vehicle.stictionSpeed = 0.10;    % m/s, smooths the rolling-resistance sign change

% --- Actuator (sections 3.3, 3.4) --------------------------------------------
p.actuator.tau            = 0.50;  % s, first-order lag from command to effect
p.actuator.aMaxComfort    =  1.50; % m/s^2, routine acceleration ceiling
p.actuator.aMinComfort    = -2.00; % m/s^2, routine deceleration floor
p.actuator.aMaxCapability =  3.00; % m/s^2, powertrain limit
p.actuator.aMinCapability = -8.00; % m/s^2, tyre-friction braking limit
p.actuator.jerkMax        =  2.50; % m/s^3, comfort rate limit on the command
p.actuator.jerkMaxEmergency = 12.0; % m/s^3, rate limit once emergency braking is unlocked

% --- Spacing policy (section 2.2) --------------------------------------------
p.spacing.timeGap       = 1.5;     % s, constant-time-gap headway
p.spacing.standstillGap = 5.0;     % m, offset so the target does not collapse to zero

% --- Controller (section 4) --------------------------------------------------
p.control.dt            = 0.02;    % s, 50 Hz control loop
p.control.setSpeed      = 25.0;    % m/s (90 km/h) driver set point
p.control.antiWindupMethod = 'conditional';  % 'conditional' | 'backCalculation' | 'none'

p.control.speed.Kp = 0.50;
p.control.speed.Ki = 0.05;
p.control.speed.Kd = 0.10;
p.control.speed.Tf = 0.20;         % s, derivative filter time constant
p.control.speed.Tt = 1.41;         % s, back-calculation time constant, sqrt(Ti*Td)

p.control.spacing.Kp = 0.15;
p.control.spacing.Ki = 0.01;
p.control.spacing.Kd = 0.80;
p.control.spacing.Tf = 0.40;       % s, heavier filtering: this is the noisy path
p.control.spacing.Tt = 8.94;       % s, back-calculation time constant, sqrt(Ti*Td)

% --- Safety (section 3.4) ----------------------------------------------------
p.safety.ttcEmergency = 4.0;       % s, below this the capability limits are unlocked
p.safety.ttcCritical  = 2.0;       % s, reporting threshold for the metrics table
p.safety.gapRatioEmergency = 0.60; % unlock capability limits below this fraction of the desired gap

% --- Sensing (section 5) -----------------------------------------------------
p.sensor.mode          = 'ideal';  % 'ideal' | 'realistic'
p.sensor.rangeRate     = 20;       % Hz, ultrasonic update rate
p.sensor.rangeMin      = 0.30;     % m, blind zone
p.sensor.rangeMax      = 120.0;    % m (radar stand-in at automotive scale)
p.sensor.rangeNoiseStd = 0.15;     % m, 1-sigma measurement noise
p.sensor.rangeQuant    = 0.02;     % m, quantisation step
p.sensor.rangeLatency  = 0.06;     % s, transport delay
p.sensor.dropoutProb   = 0.01;     % probability of a lost echo per sample
p.sensor.spikeProb     = 0.01;     % probability of a spurious short reading
p.sensor.spikeMag      = 5.0;      % m, magnitude of a spurious reading
p.sensor.speedQuant    = 0.05;     % m/s, encoder speed quantisation
p.sensor.speedNoiseStd = 0.02;     % m/s, 1-sigma at nominal speed
p.sensor.speedNoiseLow = 0.10;     % m/s, inflated 1-sigma near standstill
p.sensor.medianWindow  = 5;        % samples, outlier-rejection window
p.sensor.gapFilterTf   = 0.10;     % s, low-pass on the accepted range
p.sensor.alpha         = 0.50;     % alpha-beta tracker position gain
p.sensor.beta          = 0.10;     % alpha-beta tracker velocity gain
p.sensor.seed          = 20260907; % fixed seed keeps runs reproducible

% --- Integration -------------------------------------------------------------
p.sim.dt = 0.001;                  % s, RK4 fixed step
end
