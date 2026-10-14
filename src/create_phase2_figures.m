function create_phase2_figures(results, roadNames, speedsKmh, cfg, figureDir)
%CREATE_PHASE2_FIGURES Generate and export all Phase 2 engineering figures.

oldVisibility = get(groot, 'DefaultFigureVisible');
set(groot, 'DefaultFigureVisible', 'off');
visibilityCleanup = onCleanup(@() set(groot, 'DefaultFigureVisible', oldVisibility));

colors.speed30 = [0.000, 0.447, 0.741];
colors.speed60 = [0.850, 0.325, 0.098];
colors.invalid = [0.635, 0.078, 0.184];
colors.spring = [0.000, 0.447, 0.741];
colors.damper = [0.466, 0.674, 0.188];
colors.sum = [0.850, 0.325, 0.098];

plot_road_geometry(cfg, figureDir);
plot_event_response('speed_bump', 'Speed bump', results, roadNames, ...
    speedsKmh, cfg, colors, figureDir);
plot_event_response('pothole', 'Pothole', results, roadNames, ...
    speedsKmh, cfg, colors, figureDir);
plot_force_decomposition(results, roadNames, speedsKmh, cfg, colors, figureDir);
plot_rough_comparison(results, roadNames, speedsKmh, cfg, colors, figureDir);
plot_summary(results, roadNames, speedsKmh, colors, figureDir);

clear visibilityCleanup
end

function plot_road_geometry(cfg, figureDir)
x = linspace(0, cfg.track.length_m, 25001).';
names = {'smooth', 'speed_bump', 'pothole', 'rough'};
labels = {'Smooth', 'Speed bump', 'Pothole', 'Rough'};
lineColors = lines(4);

fig = new_figure([80, 80, 1300, 850]);
layout = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, 'Spatial road geometry — identical at both vehicle speeds');

ax = nexttile(layout);
hold(ax, 'on');
for k = 1:numel(names)
    plot(ax, x, 1000*generate_road(x, names{k}, cfg.road), ...
        'LineWidth', 1.1, 'Color', lineColors(k,:), 'DisplayName', labels{k});
end
xlabel(ax, 'Distance x (m)'); ylabel(ax, 'Road elevation r (mm)');
title(ax, 'Full 500 m track'); grid(ax, 'on'); legend(ax, 'Location', 'eastoutside');

ax = nexttile(layout);
xClose = linspace(cfg.road.obstacleStart_m - 1, ...
    cfg.road.obstacleStart_m + cfg.road.bumpLength_m + 1, 1200).';
plot(ax, xClose, 1000*generate_road(xClose, 'speed_bump', cfg.road), ...
    'LineWidth', 1.6, 'Color', lineColors(2,:));
xlabel(ax, 'Distance x (m)'); ylabel(ax, 'Road elevation r (mm)');
title(ax, sprintf('Speed bump close-up: %.0f mm high, %.2f m long', ...
    1000*cfg.road.bumpHeight_m, cfg.road.bumpLength_m)); grid(ax, 'on');

ax = nexttile(layout);
xClose = linspace(cfg.road.obstacleStart_m - 1, ...
    cfg.road.obstacleStart_m + cfg.road.potholeLength_m + 1, 1200).';
plot(ax, xClose, 1000*generate_road(xClose, 'pothole', cfg.road), ...
    'LineWidth', 1.6, 'Color', lineColors(3,:));
xlabel(ax, 'Distance x (m)'); ylabel(ax, 'Road elevation r (mm)');
title(ax, sprintf('Pothole close-up: %.0f mm deep, %.2f m long', ...
    1000*cfg.road.potholeDepth_m, cfg.road.potholeLength_m)); grid(ax, 'on');
export_pair(fig, figureDir, 'road_geometry');
close(fig);
end

function plot_event_response(roadName, displayName, results, roadNames, ...
    speedsKmh, cfg, colors, figureDir)
iRoad = find(strcmp(roadNames, roadName), 1);
fig = new_figure([80, 40, 1300, 1050]);
layout = tiledlayout(fig, 5, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, sprintf('%s response aligned to obstacle entry', displayName));
fields = {'RoadElevation_m', 'DynamicSuspensionForce_Fs_N', ...
    'SuspensionDeflection_zs_minus_zu_m', 'BodyAcceleration_mps2', ...
    'EstimatedTotalTireNormalForce_N'};
scales = [1000, 1, 1000, 1, 1];
ylabels = {'Road r (mm)', 'Fs (N)', 'Deflection (mm)', ...
    'Body accel. (m/s^2)', 'Total normal force (N)'};

contactTau = nan;
contactValue = nan;
pothole60 = results{iRoad, find(speedsKmh == 60, 1)};
firstLoss = find(pothole60.data.PredictedContactLoss, 1, 'first');
if ~isempty(firstLoss)
    entryTime60 = cfg.road.obstacleStart_m / pothole60.speed_mps;
    contactTau = pothole60.data.Time_s(firstLoss) - entryTime60;
    contactValue = pothole60.data.EstimatedTotalTireNormalForce_N(firstLoss);
end

for iField = 1:numel(fields)
    ax = nexttile(layout);
    hold(ax, 'on');
    for iSpeed = 1:numel(speedsKmh)
        result = results{iRoad, iSpeed};
        entryTime = cfg.road.obstacleStart_m / result.speed_mps;
        tau = result.data.Time_s - entryTime;
        y = result.data.(fields{iField}) * scales(iField);
        inRange = tau >= cfg.phase2.eventPlotStart_s & ...
            tau <= cfg.phase2.eventPlotEnd_s;
        tau = tau(inRange);
        y = y(inRange);
        if speedsKmh(iSpeed) == 30
            plot(ax, tau, y, 'Color', colors.speed30, 'LineWidth', 1.35, ...
                'DisplayName', '30 km/h');
        elseif isnan(contactTau)
            plot(ax, tau, y, 'Color', colors.speed60, 'LineWidth', 1.35, ...
                'DisplayName', '60 km/h');
        else
            valid = tau <= contactTau;
            invalid = tau >= contactTau;
            plot(ax, tau(valid), y(valid), 'Color', colors.speed60, ...
                'LineWidth', 1.35, 'DisplayName', '60 km/h: before first loss');
            plot(ax, tau(invalid), y(invalid), '--', 'Color', colors.invalid, ...
                'LineWidth', 1.25, 'DisplayName', ...
                '60 km/h: bilateral-model continuation (unreliable)');
        end
    end
    xlim(ax, [cfg.phase2.eventPlotStart_s, cfg.phase2.eventPlotEnd_s]);
    ylabel(ax, ylabels{iField}); grid(ax, 'on');
    if ~isnan(contactTau)
        add_invalid_region(ax, contactTau, cfg.phase2.eventPlotEnd_s, colors.invalid);
        if iField == 1
            xline(ax, contactTau, ':', 'First nonpositive normal force', ...
                'Color', colors.invalid, 'LabelOrientation', 'horizontal', ...
                'HandleVisibility', 'off');
        else
            xline(ax, contactTau, ':', 'Color', colors.invalid, ...
                'HandleVisibility', 'off');
        end
    end
    if iField == 1
        legend(ax, 'Location', 'eastoutside');
    end
    if iField == numel(fields)
        yline(ax, 0, 'k--', 'Zero normal force', 'HandleVisibility', 'off');
        if ~isnan(contactTau)
            plot(ax, contactTau, contactValue, 'x', 'Color', colors.invalid, ...
                'MarkerSize', 9, 'LineWidth', 1.8, 'HandleVisibility', 'off');
        end
        xlabel(ax, 'Time relative to obstacle entry, tau (s)');
    else
        ax.XTickLabel = [];
    end
end
export_pair(fig, figureDir, [roadName '_event_response']);
close(fig);
end

function add_invalid_region(ax, startTau, endTau, invalidColor)
limits = ylim(ax);
region = patch(ax, [startTau endTau endTau startTau], ...
    [limits(1) limits(1) limits(2) limits(2)], invalidColor, ...
    'FaceAlpha', 0.07, 'EdgeColor', 'none', 'HandleVisibility', 'off');
uistack(region, 'bottom');
end

function plot_force_decomposition(results, roadNames, speedsKmh, cfg, colors, figureDir)
iRoad = find(strcmp(roadNames, 'speed_bump'), 1);
fig = new_figure([80, 100, 1300, 750]);
layout = tiledlayout(fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, ['Speed-bump suspension-force decomposition: ' ...
    'Fs = spring force + damper force']);
for iSpeed = 1:numel(speedsKmh)
    result = results{iRoad, iSpeed};
    d = result.data;
    tau = d.Time_s - cfg.road.obstacleStart_m / result.speed_mps;
    inRange = tau >= cfg.phase2.eventPlotStart_s & tau <= cfg.phase2.eventPlotEnd_s;
    ax = nexttile(layout); hold(ax, 'on');
    plot(ax, tau(inRange), d.SpringForce_ks_zs_minus_zu_N(inRange), ...
        'Color', colors.spring, 'LineWidth', 1.25, 'DisplayName', 'Spring force');
    plot(ax, tau(inRange), d.DamperForce_cs_vs_minus_vu_N(inRange), ...
        'Color', colors.damper, 'LineWidth', 1.25, 'DisplayName', 'Damper force');
    plot(ax, tau(inRange), d.DynamicSuspensionForce_Fs_N(inRange), ...
        'Color', colors.sum, 'LineWidth', 1.6, 'DisplayName', 'Sum Fs');
    yline(ax, 0, 'k:', 'HandleVisibility', 'off');
    ylabel(ax, 'Force (N)'); grid(ax, 'on');
    title(ax, sprintf('%d km/h', speedsKmh(iSpeed)));
    if iSpeed == 1
        legend(ax, 'Location', 'eastoutside');
    end
    if iSpeed == numel(speedsKmh)
        xlabel(ax, 'Time relative to obstacle entry, tau (s)');
    else
        ax.XTickLabel = [];
    end
end
export_pair(fig, figureDir, 'bump_force_decomposition');
close(fig);
end

function plot_rough_comparison(results, roadNames, speedsKmh, cfg, colors, figureDir)
iRoad = find(strcmp(roadNames, 'rough'), 1);
fig = new_figure([80, 70, 1300, 950]);
layout = tiledlayout(fig, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, sprintf('Rough-road response over the same %.0f–%.0f m interval', ...
    cfg.phase2.roughPlotStart_m, cfg.phase2.roughPlotEnd_m));
fields = {'RoadElevation_m', 'DynamicSuspensionForce_Fs_N', ...
    'BodyAcceleration_mps2', 'EstimatedTotalTireNormalForce_N'};
scales = [1000, 1, 1, 1];
ylabels = {'Road r (mm)', 'Fs (N)', 'Body accel. (m/s^2)', ...
    'Total normal force (N)'};
speedColors = {colors.speed30, colors.speed60};
for iField = 1:numel(fields)
    ax = nexttile(layout); hold(ax, 'on');
    for iSpeed = 1:numel(speedsKmh)
        d = results{iRoad, iSpeed}.data;
        inRange = d.Distance_m >= cfg.phase2.roughPlotStart_m & ...
            d.Distance_m <= cfg.phase2.roughPlotEnd_m;
        plot(ax, d.Distance_m(inRange), d.(fields{iField})(inRange)*scales(iField), ...
            'Color', speedColors{iSpeed}, 'LineWidth', 1.1, ...
            'DisplayName', sprintf('%d km/h', speedsKmh(iSpeed)));
    end
    xlim(ax, [cfg.phase2.roughPlotStart_m, cfg.phase2.roughPlotEnd_m]);
    ylabel(ax, ylabels{iField}); grid(ax, 'on');
    if iField == 1
        legend(ax, 'Location', 'eastoutside');
    end
    if iField == numel(fields)
        yline(ax, 0, 'k--', 'Zero normal force', 'HandleVisibility', 'off');
        xlabel(ax, 'Distance x (m)');
    else
        ax.XTickLabel = [];
    end
end
export_pair(fig, figureDir, 'rough_road_comparison');
close(fig);
end

function plot_summary(results, roadNames, speedsKmh, colors, figureDir)
summaryRows = cell(numel(results), 1);
row = 0;
caseLabels = strings(numel(results), 1);
invalidIndex = false(numel(results), 1);
displayRoadCodes = {'S', 'B', 'P', 'R'};
for iRoad = 1:numel(roadNames)
    for iSpeed = 1:numel(speedsKmh)
        row = row + 1;
        summaryRows{row} = analyze_loads(results{iRoad, iSpeed});
        caseLabels(row) = sprintf('%s%d', displayRoadCodes{iRoad}, speedsKmh(iSpeed));
        invalidIndex(row) = summaryRows{row}.PredictedContactLoss;
        if invalidIndex(row)
            caseLabels(row) = caseLabels(row) + "*";
        end
    end
end
summary = vertcat(summaryRows{:});
values = {summary.PeakAbsSuspensionForce_N, ...
    summary.FullTrackRMSSuspensionForce_N, ...
    1000*summary.PeakAbsSuspensionDeflection_m, ...
    summary.FullTrackRMSBodyAcceleration_mps2};
ylabels = {'Peak abs. Fs (N)', 'Full-track RMS Fs (N)', ...
    'Peak abs. travel (mm)', 'Full-track RMS body accel. (m/s^2)'};

fig = new_figure([40, 50, 1450, 900]);
layout = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, {['Eight-case summary — * P60 is outside contact validity ' ...
    'from first predicted loss onward'], ...
    'Case codes: S = smooth, B = bump, P = pothole, R = rough; number = speed in km/h'});
barColors = zeros(numel(results), 3);
for k = 1:numel(results)
    if invalidIndex(k)
        barColors(k,:) = colors.invalid;
    elseif mod(k, 2) == 1
        barColors(k,:) = colors.speed30;
    else
        barColors(k,:) = colors.speed60;
    end
end
for iMetric = 1:numel(values)
    ax = nexttile(layout);
    bars = bar(ax, values{iMetric}, 'FaceColor', 'flat');
    bars.CData = barColors;
    ylabel(ax, ylabels{iMetric}); grid(ax, 'on');
    ax.XTick = 1:numel(caseLabels);
    ax.XTickLabel = caseLabels;
    ax.XTickLabelRotation = 0;
    ax.TickLabelInterpreter = 'none';
    xlim(ax, [0.25, numel(caseLabels) + 0.9]);
end
export_pair(fig, figureDir, 'summary_comparison');
close(fig);
end

function fig = new_figure(position)
fig = figure('Color', 'white', 'Position', position);
end

function export_pair(fig, figureDir, baseName)
exportgraphics(fig, fullfile(figureDir, [baseName '.png']), 'Resolution', 300);
exportgraphics(fig, fullfile(figureDir, [baseName '.pdf']), 'ContentType', 'vector');
end
