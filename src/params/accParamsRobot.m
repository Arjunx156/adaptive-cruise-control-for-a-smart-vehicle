function p = accParamsRobot()
%ACCPARAMSROBOT Small-robot parameter set for the ACC simulation.
%
%   Same fields as accParamsAutomotive, scaled to a two-wheeled robot with an
%   HC-SR04 ultrasonic rangefinder and wheel encoders (section 7).
%
%   NOTE: these values are engineering estimates for a typical hobby chassis.
%   They must be replaced with measured values before any claim is made about
%   how well the simulation predicts the physical vehicle.

p = accParamsAutomotive();
p.name = 'robot';

% --- Vehicle -----------------------------------------------------------------
p.vehicle.mass          = 0.80;    % kg
p.vehicle.dragCoeff     = 1.05;    % blunt body, but aerodynamically irrelevant here
p.vehicle.frontalArea   = 0.02;    % m^2
p.vehicle.rollingCoeff  = 0.015;   % dominated by gearbox and bearing friction
p.vehicle.stictionSpeed = 0.01;    % m/s

% --- Actuator ----------------------------------------------------------------
p.actuator.tau            = 0.08;  % s, small DC motor + driveline
p.actuator.aMaxComfort    =  0.30;
p.actuator.aMinComfort    = -0.40;
p.actuator.aMaxCapability =  0.90;
p.actuator.aMinCapability = -1.40;
p.actuator.jerkMax        =  1.20;
p.actuator.jerkMaxEmergency = 6.00;

% --- Spacing policy ----------------------------------------------------------
p.spacing.timeGap       = 1.0;     % s
p.spacing.standstillGap = 0.15;    % m, must stay outside the sensor blind zone

% --- Controller --------------------------------------------------------------
p.control.setSpeed      = 0.35;    % m/s

p.control.speed.Kp = 1.20;
p.control.speed.Ki = 0.60;
p.control.speed.Kd = 0.02;
p.control.speed.Tf = 0.08;
p.control.speed.Tt = 0.36;

p.control.spacing.Kp = 1.00;
p.control.spacing.Ki = 0.10;
p.control.spacing.Kd = 0.90;
p.control.spacing.Tf = 0.15;
p.control.spacing.Tt = 3.00;

% --- Safety ------------------------------------------------------------------
p.safety.ttcEmergency = 2.0;
p.safety.ttcCritical  = 1.0;

% --- Sensing: HC-SR04 characteristics (section 5.1) --------------------------
p.sensor.rangeRate     = 20;       % Hz, limited by echo decay
p.sensor.rangeMin      = 0.03;     % m, blind zone
p.sensor.rangeMax      = 2.00;     % m, usable range indoors
p.sensor.rangeNoiseStd = 0.006;    % m
p.sensor.rangeQuant    = 0.003;    % m
p.sensor.rangeLatency  = 0.05;     % s
p.sensor.dropoutProb   = 0.03;     % soft/angled surfaces absorb or deflect the pulse
p.sensor.spikeProb     = 0.02;
p.sensor.spikeMag      = 0.30;     % m
p.sensor.speedQuant    = 0.010;    % m/s, encoder tick quantisation
p.sensor.speedNoiseStd = 0.005;
p.sensor.speedNoiseLow = 0.030;    % few pulses per interval near standstill
p.sensor.gapFilterTf   = 0.08;
end
