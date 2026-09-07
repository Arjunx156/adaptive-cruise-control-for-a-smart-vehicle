function aCommand = accControllerRuntimeStep(gap, vEgo, vRel, setSpeed)
%ACCCONTROLLERRUNTIMESTEP Plain-function entry point for the Simulink model.
%
%   The MATLAB Function block declares this extrinsic. A static method call on a
%   class cannot be declared extrinsic, so the block calls this wrapper instead.

aCommand = AccControllerRuntime.step(gap, vEgo, vRel, setSpeed);
end
