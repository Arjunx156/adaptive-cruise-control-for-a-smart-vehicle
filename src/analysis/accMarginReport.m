function report = accMarginReport(models, opts)
%ACCMARGINREPORT Stability margins and bandwidth for both loops (section 4.3).
%
%   report = accMarginReport(models)
%
%   Gain selection balances objectives that cannot be maximised together, so the
%   margins are reported next to the bandwidth they bought.

arguments
    models struct
    opts.MinPhaseMargin (1,1) double = 45
    opts.Verbose (1,1) logical = true
end

report = struct();
report.speed   = loopSummary(models.Lspeed,   models.Tspeed);
report.spacing = loopSummary(models.Lspacing, models.Tspacing);
report.minPhaseMarginSpec = opts.MinPhaseMargin;
report.isPassing = report.speed.phaseMargin   >= opts.MinPhaseMargin && ...
                   report.spacing.phaseMargin >= opts.MinPhaseMargin;

if opts.Verbose
    fprintf('\nLinearised about v0 = %.1f m/s (drag pole %.4f rad/s)\n', ...
            models.v0, models.dragPole);
    fprintf('%-10s %12s %12s %14s %14s\n', ...
            'loop', 'gain margin', 'phase margin', 'crossover', 'cl bandwidth');
    printLoop('speed',   report.speed);
    printLoop('spacing', report.spacing);
    fprintf('\nSpecification: phase margin >= %.0f deg   ->   %s\n', ...
            opts.MinPhaseMargin, verdict(report.isPassing));
end
end

% -------------------------------------------------------------------------
function s = loopSummary(L, T)
[Gm, Pm, Wcg, Wcp] = margin(L);
s = struct( ...
    'gainMargin',    Gm, ...
    'gainMargin_dB', 20 * log10(Gm), ...
    'phaseMargin',   Pm, ...
    'phaseCrossover', Wcg, ...
    'gainCrossover',  Wcp, ...
    'bandwidth',      bandwidth(T), ...
    'isConditionallyStable', Gm < 1);
end

function printLoop(name, s)
fprintf('%-10s %9.1f dB %9.1f deg %10.3f rad/s %10.3f rad/s\n', ...
        name, s.gainMargin_dB, s.phaseMargin, s.gainCrossover, s.bandwidth);
end

function txt = verdict(isPassing)
if isPassing
    txt = 'PASS';
else
    txt = 'FAIL';
end
end
