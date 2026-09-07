function accSetupPath()
%ACCSETUPPATH Add the project source, script and test folders to the MATLAB path.
%
%   Call this once at the start of a session (every entry-point script in
%   scripts/ calls it automatically).

root = fileparts(mfilename('fullpath'));

% The project root goes on the path explicitly rather than relying on it being
% the working directory: matlab.unittest runs suites with the tests folder as
% cwd, which would otherwise hide this function from the tests themselves.
addpath(root);
addpath(genpath(fullfile(root, 'src')));
addpath(fullfile(root, 'scripts'));
addpath(fullfile(root, 'tests'));
end
