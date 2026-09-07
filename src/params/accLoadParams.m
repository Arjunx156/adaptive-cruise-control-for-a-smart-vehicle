function p = accLoadParams(name)
%ACCLOADPARAMS Select a parameter set by name.
%
%   p = accLoadParams("automotive") | accLoadParams("robot")

switch string(name)
    case "automotive"
        p = accParamsAutomotive();
    case "robot"
        p = accParamsRobot();
    otherwise
        error('accLoadParams:unknownSet', ...
              'Unknown parameter set "%s". Use "automotive" or "robot".', name);
end
end
