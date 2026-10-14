function eventMetrics = calculate_event_metrics(results, roadNames, speedsKmh, cfg)
%CALCULATE_EVENT_METRICS Time-integrated RMS over a common obstacle window.
% The window begins at obstacle entry and ends exactly 3 s later for both
% speeds. These complement, and do not replace, the full-track metrics.

eventRoads = {'speed_bump', 'pothole'};
nRows = numel(eventRoads) * numel(speedsKmh);
Road = strings(nRows, 1);
Speed_kmh = zeros(nRows, 1);
WindowStartRelativeToEntry_s = zeros(nRows, 1);
WindowEndRelativeToEntry_s = zeros(nRows, 1);
WindowDuration_s = zeros(nRows, 1);
EventWindowRMSSuspensionForce_N = zeros(nRows, 1);
EventWindowRMSSuspensionDeflection_m = zeros(nRows, 1);
EventWindowRMSBodyAcceleration_mps2 = zeros(nRows, 1);
ContainsPredictedContactLoss = false(nRows, 1);

row = 0;
for iRoadName = 1:numel(eventRoads)
    iRoad = find(strcmp(roadNames, eventRoads{iRoadName}), 1);
    for iSpeed = 1:numel(speedsKmh)
        row = row + 1;
        result = results{iRoad, iSpeed};
        d = result.data;
        entryTime = cfg.road.obstacleStart_m / result.speed_mps;
        windowStart = entryTime + cfg.phase2.eventWindowStart_s;
        windowEnd = entryTime + cfg.phase2.eventWindowEnd_s;
        inWindow = d.Time_s >= windowStart & d.Time_s <= windowEnd;
        t = d.Time_s(inWindow);
        duration = t(end) - t(1);

        Road(row) = string(eventRoads{iRoadName});
        Speed_kmh(row) = speedsKmh(iSpeed);
        WindowStartRelativeToEntry_s(row) = cfg.phase2.eventWindowStart_s;
        WindowEndRelativeToEntry_s(row) = cfg.phase2.eventWindowEnd_s;
        WindowDuration_s(row) = duration;
        EventWindowRMSSuspensionForce_N(row) = rms_time(t, ...
            d.DynamicSuspensionForce_Fs_N(inWindow));
        EventWindowRMSSuspensionDeflection_m(row) = rms_time(t, ...
            d.SuspensionDeflection_zs_minus_zu_m(inWindow));
        EventWindowRMSBodyAcceleration_mps2(row) = rms_time(t, ...
            d.BodyAcceleration_mps2(inWindow));
        ContainsPredictedContactLoss(row) = any(d.PredictedContactLoss(inWindow));
    end
end

eventMetrics = table(Road, Speed_kmh, WindowStartRelativeToEntry_s, ...
    WindowEndRelativeToEntry_s, WindowDuration_s, ...
    EventWindowRMSSuspensionForce_N, EventWindowRMSSuspensionDeflection_m, ...
    EventWindowRMSBodyAcceleration_mps2, ContainsPredictedContactLoss);
end

function value = rms_time(t, signal)
value = sqrt(trapz(t, signal.^2) / (t(end) - t(1)));
end
