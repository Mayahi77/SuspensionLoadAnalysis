function phase4 = create_animation(projectRoot, cfg, results, roadNames, speedsKmh)
%CREATE_ANIMATION Animate existing speed-bump histories at 30 and 60 km/h.
% This is a schematic visualization. It never reruns the ODE per frame.

animationDir = fullfile(projectRoot, 'outputs', 'animation');
frameDir = fullfile(animationDir, 'inspection_frames');
if ~isfolder(animationDir), mkdir(animationDir); end
if ~isfolder(frameDir), mkdir(frameDir); end

iRoad = find(strcmp(roadNames, 'speed_bump'), 1);
speeds = [30, 60];
indices = [find(speedsKmh == 30, 1), find(speedsKmh == 60, 1)];
if isempty(iRoad) || any(cellfun(@isempty, num2cell(indices)))
    error('Phase4:MissingCases', 'Speed-bump 30/60 km/h histories are required.');
end

event = cell(1, 2);
contactValid = false(1, 2);
for k = 1:2
    result = results{iRoad, indices(k)};
    event{k} = build_event_history(result, cfg);
    contactValid(k) = all(result.data.EstimatedTotalTireNormalForce_N > 0);
end
contactTable = table(speeds(:), contactValid(:), ...
    [event{1}.minimumNormalForce_N; event{2}.minimumNormalForce_N], ...
    'VariableNames', {'Speed_kmh', 'ContactValid', ...
    'MinimumEstimatedTotalTireNormalForce_N'});
writetable(contactTable, fullfile(animationDir, 'phase4_contact_validity.csv'));

phase4.rendered = false;
phase4.format = "none";
phase4.summary = table();
if ~all(contactValid)
    phase4.status = "blocked: selected speed-bump case is contact-invalid";
    write_phase4_status(fullfile(animationDir, 'phase4_status.txt'), phase4, cfg, contactTable);
    return
end

nFrames = round(cfg.phase4.playbackDuration_s * cfg.phase4.frameRate_Hz);
frameTau = linspace(cfg.phase4.tauStart_s, cfg.phase4.tauEnd_s, nFrames).';
sampled = cell(1, 2);
for k = 1:2
    sampled{k} = sample_event_history(event{k}, frameTau);
end
motionScale = cfg.phase4.motionScale;
forceLimit = 1.10 * max(cellfun(@(s) max(abs(s.Fs_N)), sampled));
if forceLimit == 0, forceLimit = 1; end
slowFactor = (cfg.phase4.tauEnd_s-cfg.phase4.tauStart_s) / cfg.phase4.playbackDuration_s;

fig = figure('Color', 'white', 'Visible', 'off', 'Position', [80, 60, 1500, 900]);
ax1 = axes(fig, 'Position', [0.055, 0.53, 0.405, 0.35]);
ax2 = axes(fig, 'Position', [0.54, 0.53, 0.405, 0.35]);
axHistory = axes(fig, 'Position', [0.085, 0.10, 0.84, 0.30]);
sgtitle(fig, sprintf(['Speed-bump quarter-car schematic | slow motion %.3gx | ' ...
    'vertical perturbation scale %.1fx'], slowFactor, motionScale));
draw = cell(1, 2);
draw{1} = initialize_schematic(ax1, sampled{1}, event{1}, speeds(1), motionScale);
draw{2} = initialize_schematic(ax2, sampled{2}, event{2}, speeds(2), motionScale);

hold(axHistory, 'on');
colors = {[0.000, 0.447, 0.741], [0.850, 0.325, 0.098]};
historyLines = gobjects(1, 2);
for k = 1:2
    historyLines(k) = plot(axHistory, frameTau, sampled{k}.Fs_N, ...
        'Color', colors{k}, 'LineWidth', 1.4, ...
        'DisplayName', sprintf('%d km/h', speeds(k)));
end
cursor = xline(axHistory, frameTau(1), 'k--', 'Event cursor');
xlim(axHistory, [cfg.phase4.tauStart_s, cfg.phase4.tauEnd_s]);
ylim(axHistory, [-forceLimit, forceLimit]);
xlabel(axHistory, 'Time relative to obstacle entry, tau (s)');
ylabel(axHistory, 'Dynamic suspension force Fs (N)');
title(axHistory, 'Existing simulated suspension-force histories');
legend(axHistory, historyLines, 'Location', 'northeast'); grid(axHistory, 'on');

mp4Path = fullfile(animationDir, 'speed_bump_quarter_car_phase4.mp4');
gifPath = fullfile(animationDir, 'speed_bump_quarter_car_phase4.gif');
writer = []; mp4Written = false; gifWritten = false;
try
    writer = VideoWriter(mp4Path, 'MPEG-4');
    writer.FrameRate = cfg.phase4.frameRate_Hz;
    open(writer); mp4Written = true;
catch
end

[~, maxFrame] = max(max(abs([sampled{1}.Fs_N, sampled{2}.Fs_N]), [], 2));
zeroFrame = find(frameTau >= 0, 1);
inspection = unique([1, zeroFrame, maxFrame, nFrames]);
for iFrame = 1:nFrames
    tau = frameTau(iFrame);
    for k = 1:2
        update_schematic(draw{k}, sampled{k}, event{k}, iFrame, tau, speeds(k), cfg, motionScale);
    end
    cursor.Value = tau;
    drawnow;
    frame = getframe(fig);
    if mp4Written
        writeVideo(writer, frame);
    else
        [indexed, map] = rgb2ind(frame2im(frame), 256);
        if ~gifWritten
            imwrite(indexed, map, gifPath, 'gif', 'LoopCount', inf, ...
                'DelayTime', 1/cfg.phase4.frameRate_Hz); gifWritten = true;
        else
            imwrite(indexed, map, gifPath, 'gif', 'WriteMode', 'append', ...
                'DelayTime', 1/cfg.phase4.frameRate_Hz);
        end
    end
    if any(inspection == iFrame)
        if iFrame == 1
            label = 'before_bump';
        elseif iFrame == zeroFrame
            label = 'during_bump';
        elseif iFrame == maxFrame
            label = 'maximum_response';
        else
            label = 'settling';
        end
        exportgraphics(fig, fullfile(frameDir, ['phase4_' label '.png']), 'Resolution', 150);
    end
end
if mp4Written, close(writer); end
close(fig);

if mp4Written
    [exportTable, exportOK] = verify_mp4(mp4Path, cfg.phase4.frameRate_Hz, nFrames);
    if ~exportOK
        delete(mp4Path); mp4Written = false;
    end
else
    exportTable = empty_export_table(); exportOK = false;
end
if ~mp4Written && gifWritten
    [exportTable, exportOK] = verify_gif(gifPath, cfg.phase4.frameRate_Hz, nFrames);
    phase4.format = "GIF"; phase4.file = gifPath;
else
    phase4.format = "MP4"; phase4.file = mp4Path;
end
phase4.rendered = exportOK;
phase4.summary = table(phase4.format, string(phase4.file), exportOK, ...
    exportTable.FrameCount, exportTable.FrameRate_Hz, exportTable.Duration_s, ...
    exportTable.Width_px, exportTable.Height_px, slowFactor, motionScale, ...
    'VariableNames', {'Format', 'File', 'ExportVerified', 'FrameCount', ...
    'FrameRate_Hz', 'Duration_s', 'Width_px', 'Height_px', ...
    'PlaybackSpeed_x', 'MotionScale'});
writetable(phase4.summary, fullfile(animationDir, 'phase4_export_summary.csv'));

phase4.frameValidation = verify_frame_values(event, sampled, frameTau, cfg, speeds);
writetable(phase4.frameValidation, fullfile(animationDir, 'phase4_frame_validation.csv'));
phase4.contactValidity = contactTable;
phase4.status = "completed";
write_phase4_status(fullfile(animationDir, 'phase4_status.txt'), phase4, cfg, contactTable);
end

function event = build_event_history(result, cfg)
d = result.data;
event.tau_s = d.Time_s - cfg.road.obstacleStart_m / result.speed_mps;
event.distance_m = d.Distance_m;
event.road_m = d.RoadElevation_m;
event.zs_m = d.SprungDisplacement_m;
event.zu_m = d.UnsprungDisplacement_m;
event.Fs_N = d.DynamicSuspensionForce_Fs_N;
event.deflection_m = d.SuspensionDeflection_zs_minus_zu_m;
event.minimumNormalForce_N = min(d.EstimatedTotalTireNormalForce_N);
end

function sampled = sample_event_history(event, tau)
sampled.tau_s = tau;
sampled.distance_m = interp1(event.tau_s, event.distance_m, tau, 'linear');
sampled.road_m = interp1(event.tau_s, event.road_m, tau, 'linear');
sampled.zs_m = interp1(event.tau_s, event.zs_m, tau, 'linear');
sampled.zu_m = interp1(event.tau_s, event.zu_m, tau, 'linear');
sampled.Fs_N = interp1(event.tau_s, event.Fs_N, tau, 'linear');
sampled.deflection_m = interp1(event.tau_s, event.deflection_m, tau, 'linear');
end

function draw = initialize_schematic(ax, ~, event, speedKmh, scale)
hold(ax, 'on'); axis(ax, 'equal');
xlim(ax, [-1.2, 1.2]); ylim(ax, [-0.25, 1.55]);
set(ax, 'XTick', [], 'YTick', []); box(ax, 'on');
title(ax, sprintf('%d km/h', speedKmh));
text(ax, -1.1, 1.42, 'SCHEMATIC - not vehicle geometry', 'FontWeight', 'bold');
text(ax, -1.1, 1.31, sprintf('Vertical perturbation scale: %.1fx', scale));
roadX = linspace(event.distance_m(1), event.distance_m(end), 200).';
roadY = scale * interp1(event.distance_m, event.road_m, roadX, 'linear', 'extrap');
draw.road = plot(ax, roadX-roadX(101), roadY, 'k-', 'LineWidth', 1.5);
draw.body = patch(ax, [-0.31, 0.31, 0.31, -0.31], ...
    [0.85, 0.85, 1.10, 1.10], [0.40, 0.60, 0.90], 'EdgeColor', 'k');
draw.wheel = rectangle(ax, 'Position', [-0.15, 0.30, 0.30, 0.16], ...
    'Curvature', [0.2, 0.2], 'FaceColor', [0.25, 0.25, 0.25], 'EdgeColor', 'k');
draw.spring = plot(ax, nan, nan, '-', 'Color', [0.00, 0.45, 0.74], 'LineWidth', 2);
draw.damper = plot(ax, nan, nan, '-', 'Color', [0.85, 0.33, 0.10], 'LineWidth', 3);
draw.tire = plot(ax, nan, nan, '-', 'Color', [0.20, 0.20, 0.20], 'LineWidth', 2);
draw.info = text(ax, -1.1, -0.17, '', 'FontName', 'Consolas');
end

function update_schematic(draw, sampled, event, iFrame, tau, speedKmh, cfg, scale)
zs = sampled.zs_m(iFrame); zu = sampled.zu_m(iFrame);
currentDistance = sampled.distance_m(iFrame);
roadX = linspace(currentDistance-cfg.phase4.roadViewHalfWidth_m, ...
    currentDistance+cfg.phase4.roadViewHalfWidth_m, 200).';
roadY = scale * interp1(event.distance_m, event.road_m, roadX, 'linear', 'extrap');
set(draw.road, 'XData', roadX-currentDistance, 'YData', roadY);
bodyCenterY = 0.98 + scale*zs; wheelCenterY = 0.38 + scale*zu;
bodyWidth = 0.62; bodyHeight = 0.25; wheelWidth = 0.30; wheelHeight = 0.16;
set(draw.body, 'XData', [-bodyWidth/2, bodyWidth/2, bodyWidth/2, -bodyWidth/2], ...
    'YData', bodyCenterY + [-bodyHeight/2, -bodyHeight/2, bodyHeight/2, bodyHeight/2]);
set(draw.wheel, 'Position', [-wheelWidth/2, wheelCenterY-wheelHeight/2, wheelWidth, wheelHeight]);
bodyBottom = bodyCenterY - bodyHeight/2; wheelTop = wheelCenterY + wheelHeight/2;
wheelBottom = wheelCenterY - wheelHeight/2;
[springX, springY] = spring_path(0, bodyBottom, 0, wheelTop, 10, 0.08);
set(draw.spring, 'XData', springX, 'YData', springY);
set(draw.damper, 'XData', [0.14, 0.14], 'YData', [bodyBottom, wheelTop]);
set(draw.tire, 'XData', [0, 0], 'YData', [wheelBottom, scale*sampled.road_m(iFrame)]);
set(draw.info, 'String', sprintf(['tau = %+6.3f s\nFs = %+8.2f N\n' ...
    'deflection = %+7.3f mm'], tau, sampled.Fs_N(iFrame), ...
    1000*sampled.deflection_m(iFrame)));
title(draw.body.Parent, sprintf('%d km/h', speedKmh));
end

function [x, y] = spring_path(x1, y1, x2, y2, turns, amplitude)
t = linspace(0, 1, 2*turns+1);
x = x1 + (x2-x1)*t + amplitude*sin(2*pi*turns*t);
y = y1 + (y2-y1)*t;
end

function validation = verify_frame_values(event, sampled, frameTau, cfg, speeds)
targetTau = [cfg.phase4.tauStart_s; 0; cfg.phase4.tauEnd_s];
rows = numel(targetTau) * numel(speeds);
Speed_kmh = zeros(rows, 1); RequestedTau_s = zeros(rows, 1); FrameTau_s = zeros(rows, 1);
Fs_N = zeros(rows, 1); Deflection_mm = zeros(rows, 1); Zs_m = zeros(rows, 1); Zu_m = zeros(rows, 1);
MaxValueError = zeros(rows, 1); row = 0;
for k = 1:numel(speeds)
    for j = 1:numel(targetTau)
        row = row + 1; iFrame = nearest_index(frameTau, targetTau(j));
        expected = sample_event_history(event{k}, frameTau(iFrame));
        Speed_kmh(row) = speeds(k); RequestedTau_s(row) = targetTau(j); FrameTau_s(row) = frameTau(iFrame);
        Fs_N(row) = sampled{k}.Fs_N(iFrame); Deflection_mm(row) = 1000*sampled{k}.deflection_m(iFrame);
        Zs_m(row) = sampled{k}.zs_m(iFrame); Zu_m(row) = sampled{k}.zu_m(iFrame);
        MaxValueError(row) = max(abs([sampled{k}.Fs_N(iFrame)-expected.Fs_N, ...
            sampled{k}.deflection_m(iFrame)-expected.deflection_m, sampled{k}.zs_m(iFrame)-expected.zs_m, ...
            sampled{k}.zu_m(iFrame)-expected.zu_m]));
    end
end
validation = table(Speed_kmh, RequestedTau_s, FrameTau_s, Fs_N, Deflection_mm, Zs_m, Zu_m, MaxValueError, ...
    'VariableNames', {'Speed_kmh', 'RequestedTau_s', 'FrameTau_s', 'DisplayedFs_N', ...
    'DisplayedDeflection_mm', 'DisplayedZs_m', 'DisplayedZu_m', 'MaxDisplayedValueError'});
end

function idx = nearest_index(x, value)
[~, idx] = min(abs(x-value));
end

function [exportTable, ok] = verify_mp4(path, frameRate, expectedFrames)
ok = false; exportTable = empty_export_table();
try
    reader = VideoReader(path); count = 0;
    while hasFrame(reader), readFrame(reader); count = count + 1; end
    exportTable = table(count, frameRate, reader.Duration, reader.Width, reader.Height, ...
        'VariableNames', {'FrameCount', 'FrameRate_Hz', 'Duration_s', 'Width_px', 'Height_px'});
    ok = count == expectedFrames && reader.Width > 0 && reader.Height > 0 && reader.Duration > 0;
catch
end
end

function [exportTable, ok] = verify_gif(path, frameRate, expectedFrames)
ok = false; exportTable = empty_export_table();
try
    info = imfinfo(path); count = numel(info);
    exportTable = table(count, frameRate, count/frameRate, info(1).Width, info(1).Height, ...
        'VariableNames', {'FrameCount', 'FrameRate_Hz', 'Duration_s', 'Width_px', 'Height_px'});
    ok = count == expectedFrames && info(1).Width > 0 && info(1).Height > 0;
catch
end
end

function t = empty_export_table()
t = table(0, 0, 0, 0, 0, 'VariableNames', {'FrameCount', 'FrameRate_Hz', ...
    'Duration_s', 'Width_px', 'Height_px'});
end

function write_phase4_status(path, phase4, cfg, contactTable)
fid = fopen(path, 'w');
if fid < 0, error('Phase4:FileOpen', 'Could not open %s.', path); end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, 'Phase 4 speed-bump animation status\n\n');
fprintf(fid, 'Status: %s\n', phase4.status);
fprintf(fid, 'Selected cases: speed bump at 30 and 60 km/h.\n');
fprintf(fid, 'Input: existing simulated histories; no ODE rerun per frame.\n');
fprintf(fid, 'Tau window: %.3f to %.3f s relative to obstacle entry.\n', cfg.phase4.tauStart_s, cfg.phase4.tauEnd_s);
fprintf(fid, 'Playback duration: %.3f s at %.1f fps.\n', cfg.phase4.playbackDuration_s, cfg.phase4.frameRate_Hz);
fprintf(fid, 'Slow-motion factor: %.6g x real time.\n', ...
    (cfg.phase4.tauEnd_s-cfg.phase4.tauStart_s)/cfg.phase4.playbackDuration_s);
fprintf(fid, 'Vertical drawing scale: %.3g x for road, body, and wheel perturbations.\n', cfg.phase4.motionScale);
fprintf(fid, 'Drawing offsets are static schematic offsets about static equilibrium.\n');
fprintf(fid, 'Sprung and unsprung positions are driven separately by zs and zu.\n');
fprintf(fid, 'Contact checks before rendering:\n');
for k = 1:height(contactTable)
    fprintf(fid, '  %g km/h: valid=%s, minimum normal force=%.9g N\n', contactTable.Speed_kmh(k), ...
        string(contactTable.ContactValid(k)), contactTable.MinimumEstimatedTotalTireNormalForce_N(k));
end
if phase4.rendered, fprintf(fid, 'Export verified by decoding the saved video/GIF.\n');
else, fprintf(fid, 'Export was not verified; inspect export summary for limitation.\n'); end
clear cleanup
end
