function modelPath = buildAccSimulinkModel(opts)
%BUILDACCSIMULINKMODEL Generate the ACC Simulink model programmatically.
%
%   modelPath = buildAccSimulinkModel()
%
%   The model is built from a script rather than saved as a binary .slx that
%   nobody can diff. The block diagram is therefore reviewable in version
%   control, and it is regenerated from the same parameter struct the .m
%   simulation uses.
%
%   The plant is native Simulink blocks: two integrator chains for the vehicles,
%   a first-order transfer function for the actuator lag (section 3.3), and
%   explicit drag, rolling resistance and grade terms (section 3.1). The
%   controller is a MATLAB Function block that calls the same control law
%   accRunSimulation uses, so the two never diverge.
%
%   Run scripts/runSimulinkComparison to simulate it and compare.

arguments
    opts.ModelName (1,1) string = "accModel"
    opts.Open (1,1) logical = false
end

accSetupPath();
name = char(opts.ModelName);

if bdIsLoaded(name)
    close_system(name, 0);
end

new_system(name);
addBlocks(name);
addLines(name);
Simulink.BlockDiagram.arrangeSystem(name);

modelPath = fullfile(accResultsDir(), [name '.slx']);
save_system(name, modelPath);

if opts.Open
    open_system(name);
end

fprintf('Built %s\n', modelPath);
end

% =========================================================================
function addBlocks(name)
%ADDBLOCKS Every block in the diagram, with its parameters.

% --- Scenario inputs, supplied from the base workspace ----------------------
add(name, 'simulink/Sources/From Workspace', 'aLead', ...
    'VariableName', 'aLeadInput', 'Interpolate', 'on', 'OutputAfterFinalValue', 'Holding final value');
add(name, 'simulink/Sources/From Workspace', 'grade', ...
    'VariableName', 'gradeInput', 'Interpolate', 'on', 'OutputAfterFinalValue', 'Holding final value');
add(name, 'simulink/Sources/Constant', 'setSpeed', 'Value', 'accParams.control.setSpeed');

% --- Lead vehicle: acceleration integrated twice, never reversing -----------
add(name, 'simulink/Continuous/Integrator', 'vLead', ...
    'InitialCondition', 'accInitial.vLead', 'LimitOutput', 'on', ...
    'UpperSaturationLimit', 'inf', 'LowerSaturationLimit', '0');
add(name, 'simulink/Continuous/Integrator', 'sLead', 'InitialCondition', 'accInitial.sLead');

% --- Controller: the same control law the .m simulation runs ----------------
add(name, 'simulink/User-Defined Functions/MATLAB Function', 'controller');
setControllerScript(name);

% --- Actuator lag (section 3.3) ---------------------------------------------
add(name, 'simulink/Continuous/Transfer Fcn', 'actuator', ...
    'Numerator', '[1]', 'Denominator', '[accParams.actuator.tau 1]');

% --- Resistances, which the controller is not told about (section 3.1) ------
add(name, 'simulink/Math Operations/Math Function', 'vSquared', 'Operator', 'square');
add(name, 'simulink/Math Operations/Gain', 'dragGain', ...
    'Gain', ['0.5 * accParams.vehicle.airDensity * accParams.vehicle.dragCoeff' ...
             ' * accParams.vehicle.frontalArea / accParams.vehicle.mass']);
add(name, 'simulink/Math Operations/Gain', 'invStiction', ...
    'Gain', '1 / accParams.vehicle.stictionSpeed');
add(name, 'simulink/Math Operations/Trigonometric Function', 'rollTanh', 'Operator', 'tanh');
add(name, 'simulink/Math Operations/Gain', 'rollGain', ...
    'Gain', 'accParams.vehicle.rollingCoeff * accParams.vehicle.gravity');
add(name, 'simulink/Math Operations/Trigonometric Function', 'gradeSin', 'Operator', 'sin');
add(name, 'simulink/Math Operations/Gain', 'gradeGain', 'Gain', 'accParams.vehicle.gravity');

add(name, 'simulink/Math Operations/Sum', 'egoAccel', 'Inputs', '+---');

% --- Ego vehicle ------------------------------------------------------------
add(name, 'simulink/Continuous/Integrator', 'vEgo', ...
    'InitialCondition', 'accInitial.vEgo', 'LimitOutput', 'on', ...
    'UpperSaturationLimit', 'inf', 'LowerSaturationLimit', '0');
add(name, 'simulink/Continuous/Integrator', 'sEgo', 'InitialCondition', 'accInitial.sEgo');

% --- Derived signals --------------------------------------------------------
add(name, 'simulink/Math Operations/Sum', 'gap', 'Inputs', '+-');
add(name, 'simulink/Math Operations/Sum', 'vRel', 'Inputs', '+-');

% --- Logging ----------------------------------------------------------------
add(name, 'simulink/Sinks/To Workspace', 'logGap', 'VariableName', 'simGap', ...
    'SaveFormat', 'Structure With Time');
add(name, 'simulink/Sinks/To Workspace', 'logVEgo', 'VariableName', 'simVEgo', ...
    'SaveFormat', 'Structure With Time');
add(name, 'simulink/Sinks/To Workspace', 'logVLead', 'VariableName', 'simVLead', ...
    'SaveFormat', 'Structure With Time');
add(name, 'simulink/Sinks/To Workspace', 'logCommand', 'VariableName', 'simCommand', ...
    'SaveFormat', 'Structure With Time');

% --- Solver: fixed-step RK4, matching accRunSimulation ----------------------
set_param(name, 'Solver', 'ode4', 'FixedStep', 'accParams.sim.dt', ...
                'StopTime', 'accStopTime', 'SolverType', 'Fixed-step');
end

% =========================================================================
function addLines(name)
%ADDLINES The signal flow, written out so the diagram can be read as text.

connections = {
    'aLead/1',      'vLead/1'
    'vLead/1',      'sLead/1'
    'vLead/1',      'vRel/1'
    'vEgo/1',       'vRel/2'
    'sLead/1',      'gap/1'
    'sEgo/1',       'gap/2'

    'gap/1',        'controller/1'
    'vEgo/1',       'controller/2'
    'vRel/1',       'controller/3'
    'setSpeed/1',   'controller/4'

    'controller/1', 'actuator/1'
    'actuator/1',   'egoAccel/1'

    'vEgo/1',       'vSquared/1'
    'vSquared/1',   'dragGain/1'
    'dragGain/1',   'egoAccel/2'

    'vEgo/1',       'invStiction/1'
    'invStiction/1','rollTanh/1'
    'rollTanh/1',   'rollGain/1'
    'rollGain/1',   'egoAccel/3'

    'grade/1',      'gradeSin/1'
    'gradeSin/1',   'gradeGain/1'
    'gradeGain/1',  'egoAccel/4'

    'egoAccel/1',   'vEgo/1'
    'vEgo/1',       'sEgo/1'

    'gap/1',        'logGap/1'
    'vEgo/1',       'logVEgo/1'
    'vLead/1',      'logVLead/1'
    'controller/1', 'logCommand/1'
    };

for idx = 1:size(connections, 1)
    add_line(name, connections{idx, 1}, connections{idx, 2}, 'autorouting', 'on');
end
end

% =========================================================================
function setControllerScript(name)
%SETCONTROLLERSCRIPT Point the MATLAB Function block at the shared control law.
%
%   The call is extrinsic because the controller is written for clarity rather
%   than for code generation. That is fine here: this model exists to be
%   simulated, not deployed. The embedded implementation in section 7 is a
%   separate exercise with different constraints.

script = [ ...
    "function aCommand = accControllerBlock(gap, vEgo, vRel, setSpeed)"
    "%#codegen"
    "coder.extrinsic('accControllerRuntimeStep');"
    "aCommand = 0;"
    "aCommand = accControllerRuntimeStep(gap, vEgo, vRel, setSpeed);"
    "end"];

chart = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', [name '/controller']);
chart.Script = char(strjoin(script, newline));

% The controller is discrete: it runs at p.control.dt, not at every solver step.
chart.ChartUpdate = 'DISCRETE';
chart.SampleTime = 'accParams.control.dt';
end

% =========================================================================
function add(model, source, blockName, varargin)
add_block(source, [model '/' blockName], varargin{:});
end
