classdef AccControllerRuntime < handle
    %ACCCONTROLLERRUNTIME Controller state held across Simulink solver steps.
    %
    %   The Simulink model drives the same control law as accRunSimulation rather
    %   than a second copy of it, so the block diagram and the .m simulation
    %   cannot drift apart. That requires somewhere to keep the controller state
    %   between calls from a MATLAB Function block, which is what this singleton
    %   provides.
    %
    %   Usage:
    %       AccControllerRuntime.reset(p, vEgo0, gap0);   % before each simulation
    %       aApplied = AccControllerRuntime.step(gap, vEgo, vRel, setSpeed);

    properties (Access = private)
        Controller
        Params
        PreviousCommand
    end

    methods (Static)
        function obj = instance(newInstance)
            %INSTANCE The single runtime object, created or replaced on demand.
            persistent stored
            if nargin > 0
                stored = newInstance;
            end
            if isempty(stored)
                error('AccControllerRuntime:notInitialised', ...
                      'Call AccControllerRuntime.reset(p, vEgo0, gap0) first.');
            end
            obj = stored;
        end

        function reset(p, vEgo0, gap0)
            %RESET Install a parameter set and start from the trim operating point.
            obj = AccControllerRuntime();
            obj.Params = p;
            obj.Controller = accControllerInit(p, vEgo0);

            aTrim = accTrimAcceleration(vEgo0, 0, p);
            obj.Controller.speedIntegral   = aTrim;
            obj.Controller.spacingIntegral = aTrim;
            obj.Controller.last.desiredGap = accSpacingPolicy(vEgo0, p);
            obj.PreviousCommand = aTrim;

            AccControllerRuntime.instance(obj);
        end

        function aApplied = step(gap, vEgo, vRel, setSpeed)
            %STEP One control sample: sense, decide, saturate, integrate.
            obj = AccControllerRuntime.instance();
            aApplied = obj.stepImpl(gap, vEgo, vRel, setSpeed);
        end
    end

    methods (Access = private)
        function aApplied = stepImpl(obj, gap, vEgo, vRel, setSpeed)
            p = obj.Params;
            meas = struct('gap', gap, 'vEgo', vEgo, 'vRel', vRel, 'setSpeed', setSpeed);

            [aDes, ~, obj.Controller] = accControllerStep(obj.Controller, meas, p);

            ttc = accTimeToCollision(gap, vRel);
            gapRatio = gap / max(obj.Controller.last.desiredGap, eps);
            [aApplied, satInfo] = accSaturation(aDes, obj.PreviousCommand, ttc, ...
                                                gapRatio, p.control.dt, p);

            obj.Controller = accControllerIntegrate(obj.Controller, aApplied, ...
                                                    satInfo.isMagnitudeLimited, p);
            obj.PreviousCommand = aApplied;
        end
    end
end
