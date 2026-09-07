function results = runTests(opts)
%RUNTESTS Run the ACC test suite, optionally with line coverage.
%
%   runTests                      % run everything (about 8 minutes)
%   runTests('Coverage', true)    % add statement coverage of src/
%   runTests('Filter', "Plant")   % run only tests whose name matches a substring
%   runTests('Filter', ["Plant" "Sensor"])   % or any of several
%
%   Coverage is off by default. The instrumentation costs roughly a factor of
%   nine, which is unremarkable on the unit tests but turns the end-to-end
%   scenario tests -- already nineteen minutes of simulation -- into three hours.
%   Run it deliberately, not on every change.
%
%   Exits with a nonzero status when run under -batch and anything fails, so CI
%   and the command line agree on what "passing" means.

arguments
    opts.Coverage (1,1) logical = false
    opts.Filter (1,:) string = string.empty
end

accSetupPath();
root = fileparts(mfilename('fullpath'));

import matlab.unittest.TestSuite
import matlab.unittest.TestRunner
import matlab.unittest.plugins.CodeCoveragePlugin
import matlab.unittest.plugins.codecoverage.CoverageResult

suite = TestSuite.fromFolder(fullfile(root, 'tests'));
if ~isempty(opts.Filter)
    % contains accepts a list of patterns and matches any of them, so a filter
    % can select several test classes at once.
    isMatch = contains(string({suite.Name}), opts.Filter, 'IgnoreCase', true);
    suite = suite(isMatch);
end

runner = TestRunner.withTextOutput('OutputDetail', 'Terse');

if opts.Coverage
    format = CoverageResult;
    runner.addPlugin(CodeCoveragePlugin.forFolder(fullfile(root, 'src'), ...
        'IncludingSubfolders', true, 'Producing', format));
end

results = runner.run(suite);

fprintf('\n%s\n', repmat('-', 1, 68));
fprintf('%d passed, %d failed, %d incomplete in %.1f s\n', ...
    nnz([results.Passed]), nnz([results.Failed]), nnz([results.Incomplete]), ...
    sum([results.Duration]));

if opts.Coverage
    % coverageSummary returns one [executed, executable] row per file, so the
    % totals have to be summed across files rather than read off one row.
    perFile = coverageSummary(format.Result, "statement");
    executed = sum(perFile(:, 1));
    executable = sum(perFile(:, 2));
    linePercent = 100 * executed / executable;
    fprintf('Statement coverage of src/: %.1f%% (%d of %d statements)\n', ...
        linePercent, executed, executable);
    if linePercent < 80
        fprintf('WARNING: below the 80%% coverage target.\n');
    end
end
fprintf('%s\n', repmat('-', 1, 68));

if any([results.Failed]) && batchStartupOptionUsed
    exit(1);
end

if nargout == 0
    clear results
end
end
