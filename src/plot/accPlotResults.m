function fig = accPlotResults(out, opts)
%ACCPLOTRESULTS The three signals that characterise ACC behaviour (section 6.3).
%
%   Panel 1  velocity trajectories -- tracking quality and mode transitions
%   Panel 2  actual against desired separation -- regulation accuracy, and
%            whether the gap ever approaches zero
%   Panel 3  control effort against its limits -- whether the performance above
%            was achieved within realistic bounds

arguments
    out struct
    opts.Visible (1,1) string = "off"
end

t = out.t;
p = out.params;
fig = figure('Visible', opts.Visible, 'Position', [100 100 900 780]);
tl = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

title(tl, sprintf('%s  —  %s', out.scenario.name, out.scenario.description), ...
      'Interpreter', 'none', 'FontWeight', 'bold');
subtitle(tl, sprintf('%s parameters, %s sensing', p.name, p.sensor.mode), ...
         'Interpreter', 'none');

% --- Panel 1: velocities -----------------------------------------------------
ax1 = nexttile(tl);
hold(ax1, 'on');
plot(ax1, t, out.vLead, 'LineWidth', 1.4, 'DisplayName', 'lead');
plot(ax1, t, out.vEgo,  'LineWidth', 1.6, 'DisplayName', 'ego');
yline(ax1, out.setSpeed, '--', 'set speed', 'Color', [0.4 0.4 0.4], ...
      'HandleVisibility', 'off');
ylabel(ax1, 'speed (m/s)');
legend(ax1, 'Location', 'best');
grid(ax1, 'on');

% --- Panel 2: separation -----------------------------------------------------
ax2 = nexttile(tl);
hold(ax2, 'on');
plot(ax2, t, out.gap, 'LineWidth', 1.6, 'DisplayName', 'actual gap');
plot(ax2, t, out.gapDesired, '--', 'LineWidth', 1.3, 'DisplayName', 'desired gap');
if ~strcmp(p.sensor.mode, 'ideal')
    plot(ax2, t, out.gapMeas, ':', 'LineWidth', 1.0, 'DisplayName', 'measured gap');
end
yline(ax2, 0, '-', 'collision', 'Color', [0.8 0 0], 'LineWidth', 1.2, ...
      'HandleVisibility', 'off');
ylabel(ax2, 'separation (m)');
legend(ax2, 'Location', 'best');
grid(ax2, 'on');

% --- Panel 3: control effort -------------------------------------------------
ax3 = nexttile(tl);
hold(ax3, 'on');
plot(ax3, t, out.aCommand, 'LineWidth', 1.6, 'DisplayName', 'applied command');
plot(ax3, t, out.aTrac, ':', 'LineWidth', 1.1, 'DisplayName', 'achieved (after lag)');
yline(ax3, p.actuator.aMaxComfort, '--', 'Color', [0.4 0.4 0.4], 'HandleVisibility', 'off');
yline(ax3, p.actuator.aMinComfort, '--', 'Color', [0.4 0.4 0.4], 'HandleVisibility', 'off');
yline(ax3, p.actuator.aMinCapability, '-.', 'Color', [0.8 0 0], 'HandleVisibility', 'off');
ylabel(ax3, 'acceleration (m/s^2)');
xlabel(ax3, 'time (s)');
legend(ax3, 'Location', 'best');
grid(ax3, 'on');

linkaxes([ax1 ax2 ax3], 'x');
xlim(ax1, [t(1) t(end)]);

% Shade the spacing-mode intervals last, so the patches adopt the axis limits
% the data established rather than forcing limits of their own.
for ax = [ax1 ax2 ax3]
    shadeSpacingMode(ax, t, out.isSpacingMode);
end
end

% -------------------------------------------------------------------------
function shadeSpacingMode(ax, t, isSpacing)
%SHADESPACINGMODE Grey background wherever the spacing loop won the min-selector.
edges = diff([false; isSpacing(:); false]);
starts = find(edges == 1);
stops  = find(edges == -1) - 1;
if isempty(starts)
    return
end

limits = ylim(ax);
hold(ax, 'on');
patches = gobjects(1, numel(starts));
for idx = 1:numel(starts)
    x = [t(starts(idx)) t(stops(idx)) t(stops(idx)) t(starts(idx))];
    patches(idx) = patch(ax, 'XData', x, ...
        'YData', [limits(1) limits(1) limits(2) limits(2)], ...
        'FaceColor', [0.85 0.88 0.92], 'EdgeColor', 'none', ...
        'HandleVisibility', 'off');
end
uistack(patches, 'bottom');
ylim(ax, limits);
end
