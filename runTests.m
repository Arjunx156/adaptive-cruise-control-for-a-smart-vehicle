function results = runTests(opts)
%RUNTESTS Run the ACC test suite, optionally with line coverage.
%
%   runTests                      % run everything, report coverage
%   runTests('Coverage', false)   % skip the coverage instrumentation
%   runTests('Filter', "Plant")   % run only test classes matching a substring
%
%   Exits with a nonzero status when run under -batch and anything fails, so CI
%   and the command line agree on what "passing" means.

arguments
    opts.Coverage (1,1) logical = true
    opts.Filter (1,1) string = ""
end

accSetupPath();
root = fileparts(mfilename('fullpath'));

import matlab.unittest.TestSuite
import matlab.unittest.TestRunner
import matlab.unittest.plugins.CodeCoveragePlugin
import matlab.unittest.plugins.codecoverage.CoverageResult

suite = TestSuite.fromFolder(fullfile(root, 'tests'));
if strlength(opts.Filter) > 0
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
    summary = format.Result.coverageSummary;
    linePercent = 100 * summary(1) / summary(2);
    fprintf('Statement coverage of src/: %.1f%% (%d of %d)\n', ...
        linePercent, summary(1), summary(2));
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
