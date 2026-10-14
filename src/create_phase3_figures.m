function create_phase3_figures(cycles, severitySummary, roadNames, speedsKmh, ...
    commonEdges, cumulativeThresholds, cfg, figureDir)
%CREATE_PHASE3_FIGURES Export Phase 3 load-spectrum and severity figures.

oldVisibility = get(groot, 'DefaultFigureVisible');
set(groot, 'DefaultFigureVisible', 'off');
cleanup = onCleanup(@() set(groot, 'DefaultFigureVisible', oldVisibility));

colors.speed30 = [0.000, 0.447, 0.741];
colors.speed60 = [0.850, 0.325, 0.098];
colors.excluded = [0.635, 0.078, 0.184];
labels = {'Smooth', 'Speed bump', 'Pothole', 'Rough road'};

plot_histograms(cycles, severitySummary, roadNames, speedsKmh, ...
    commonEdges, labels, colors, figureDir);
plot_exceedance(cycles, roadNames, speedsKmh, cumulativeThresholds, ...
    severitySummary, labels, colors, figureDir);
plot_severity(severitySummary, cfg, labels, colors, figureDir);
clear cleanup
end

function plot_histograms(cycles, summary, roadNames, speedsKmh, edges, ...
    labels, colors, figureDir)
fig = figure('Color', 'white', 'Position', [60, 60, 1400, 900]);
layout = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, {'Rainflow force-range spectra over equal 500 m exposure', ...
    'Weighted by cycle count; residual half cycles retained; common range bins'});
for iRoad = 1:numel(roadNames)
    ax = nexttile(layout); hold(ax, 'on');
    handles = gobjects(0); names = {};
    for iSpeed = 1:numel(speedsKmh)
        c = cycles{iRoad, iSpeed};
        if isempty(c)
            continue
        end
        y = weighted_cycle_histogram(c, edges);
        color = colors.speed30;
        if speedsKmh(iSpeed) == 60, color = colors.speed60; end
        handles(end+1) = stairs(ax, edges, [y; y(end)], 'LineWidth', 1.5, ...
            'Color', color); %#ok<AGROW>
        names{end+1} = sprintf('%d km/h', speedsKmh(iSpeed)); %#ok<AGROW>
    end
    xlim(ax, [edges(1), edges(end)]);
    xlabel(ax, 'Dynamic suspension-force range (N)');
    ylabel(ax, 'Cycle count per bin');
    title(ax, labels{iRoad}); grid(ax, 'on');
    if ~isempty(handles), legend(ax, handles, names, 'Location', 'northeast'); end
    if iRoad == 1
        text(ax, 0.5, 0.55, 'Zero force: zero rainflow cycles', ...
            'Units', 'normalized', 'HorizontalAlignment', 'center');
    end
    annotate_exclusions(ax, summary, roadNames{iRoad}, colors.excluded);
end
export_pair(fig, figureDir, 'rainflow_range_histograms');
close(fig);
end

function plot_exceedance(cycles, roadNames, speedsKmh, thresholds, summary, ...
    labels, colors, figureDir)
fig = figure('Color', 'white', 'Position', [60, 60, 1400, 900]);
layout = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, {'Cumulative rainflow range exceedance over equal 500 m exposure', ...
    'Number of cycles with force range greater than or equal to threshold'});
for iRoad = 1:numel(roadNames)
    ax = nexttile(layout); hold(ax, 'on');
    handles = gobjects(0); names = {};
    for iSpeed = 1:numel(speedsKmh)
        c = cycles{iRoad, iSpeed};
        if isempty(c), continue; end
        exceedance = zeros(size(thresholds));
        for k = 1:numel(thresholds)
            exceedance(k) = sum(c.CycleCount(c.ForceRange_N >= thresholds(k)));
        end
        positive = exceedance > 0;
        color = colors.speed30;
        if speedsKmh(iSpeed) == 60, color = colors.speed60; end
        handles(end+1) = plot(ax, thresholds(positive), exceedance(positive), ...
            'LineWidth', 1.5, 'Color', color); %#ok<AGROW>
        names{end+1} = sprintf('%d km/h', speedsKmh(iSpeed)); %#ok<AGROW>
    end
    xlim(ax, [thresholds(1), thresholds(end)]);
    xlabel(ax, 'Force-range threshold (N)');
    ylabel(ax, 'Cycles with range >= threshold');
    title(ax, labels{iRoad}); grid(ax, 'on');
    if ~isempty(handles)
        set(ax, 'YScale', 'log');
        legend(ax, handles, names, 'Location', 'southwest');
    end
    if iRoad == 1
        text(ax, 0.5, 0.55, 'Zero force: no positive values for log axis', ...
            'Units', 'normalized', 'HorizontalAlignment', 'center');
    end
    annotate_exclusions(ax, summary, roadNames{iRoad}, colors.excluded);
end
export_pair(fig, figureDir, 'rainflow_cumulative_exceedance');
close(fig);
end

function plot_severity(summary, cfg, labels, colors, figureDir)
fig = figure('Color', 'white', 'Position', [100, 100, 1400, 650]);
layout = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, {sprintf(['Relative load-severity over 500 m, F_{ref} = %.0f N ' ...
    '(not fatigue damage or life)'], cfg.phase3.referenceForce_N), ...
    'Zero signals are marked at the axis floor; contact-invalid cases are excluded'});
columns = {'Severity_m3', 'Severity_m5'};
for iExponent = 1:2
    ax = nexttile(layout); hold(ax, 'on');
    values = summary.(columns{iExponent});
    positiveValues = values(isfinite(values) & values > 0);
    yFloor = min(positiveValues) / 20;
    yCeiling = max(positiveValues) * 3;
    for iRoad = 1:4
        row30 = 2*iRoad - 1;
        row60 = 2*iRoad;
        plot_point(ax, iRoad-0.10, values(row30), yFloor, 'o', colors.speed30, ...
            summary.Eligible(row30));
        plot_point(ax, iRoad+0.10, values(row60), yFloor, 's', colors.speed60, ...
            summary.Eligible(row60));
        annotate_severity_status(ax, iRoad-0.10, values(row30), yFloor, ...
            summary.Eligible(row30), colors.excluded);
        annotate_severity_status(ax, iRoad+0.10, values(row60), yFloor, ...
            summary.Eligible(row60), colors.excluded);
    end
    set(ax, 'YScale', 'log', 'XTick', 1:4, 'XTickLabel', labels);
    xlim(ax, [0.5, 4.5]); ylim(ax, [yFloor/1.5, yCeiling]);
    ylabel(ax, sprintf('S_%d (dimensionless)', cfg.phase3.severityExponents(iExponent)));
    title(ax, sprintf('Exponent m = %d (illustrative, uncalibrated)', ...
        cfg.phase3.severityExponents(iExponent)));
    grid(ax, 'on');
    h30 = plot(ax, nan, nan, 'o', 'Color', colors.speed30, ...
        'MarkerFaceColor', colors.speed30, 'DisplayName', '30 km/h');
    h60 = plot(ax, nan, nan, 's', 'Color', colors.speed60, ...
        'MarkerFaceColor', colors.speed60, 'DisplayName', '60 km/h');
    legend(ax, [h30 h60], 'Location', 'northwest');
end
export_pair(fig, figureDir, 'relative_load_severity');
close(fig);
end

function annotate_exclusions(ax, summary, roadName, excludedColor)
rows = summary.Road == string(roadName) & ~summary.Eligible;
if ~any(rows)
    return
end
speeds = summary.Speed_kmh(rows);
messages = arrayfun(@(v) sprintf('%g km/h excluded: contact invalid', v), ...
    speeds, 'UniformOutput', false);
text(ax, 0.98, 0.88, messages, 'Units', 'normalized', ...
    'HorizontalAlignment', 'right', 'Color', excludedColor, ...
    'FontWeight', 'bold');
end

function annotate_severity_status(ax, x, value, yFloor, eligible, excludedColor)
if ~eligible
    plot(ax, x, yFloor, 'x', 'Color', excludedColor, ...
        'MarkerSize', 9, 'LineWidth', 1.8);
    text(ax, x, yFloor*1.35, 'excluded', 'Color', excludedColor, ...
        'HorizontalAlignment', 'center');
elseif value == 0
    text(ax, x, yFloor*1.35, '0', 'HorizontalAlignment', 'center');
end
end

function plot_point(ax, x, value, yFloor, marker, color, eligible)
if ~eligible
    return
end
if value > 0
    plot(ax, x, value, marker, 'Color', color, 'MarkerFaceColor', color, ...
        'MarkerSize', 7, 'LineWidth', 1.2);
else
    plot(ax, x, yFloor, marker, 'Color', color, 'MarkerFaceColor', 'none', ...
        'MarkerSize', 7, 'LineWidth', 1.2);
end
end

function export_pair(fig, figureDir, baseName)
exportgraphics(fig, fullfile(figureDir, [baseName '.png']), 'Resolution', 300);
exportgraphics(fig, fullfile(figureDir, [baseName '.pdf']), 'ContentType', 'vector');
end
