function contactTable = analyze_contact_validity(results, roadNames, speedsKmh, cfg)
%ANALYZE_CONTACT_VALIDITY Locate first sampled nonpositive normal force.

nRows = numel(results);
Road = strings(nRows, 1);
Speed_kmh = zeros(nRows, 1);
HasPredictedContactLoss = false(nRows, 1);
FirstNonpositiveTime_s = nan(nRows, 1);
FirstNonpositiveDistance_m = nan(nRows, 1);
FirstNonpositiveTauFromObstacleEntry_s = nan(nRows, 1);
FirstNonpositiveNormalForce_N = nan(nRows, 1);

row = 0;
for iRoad = 1:numel(roadNames)
    for iSpeed = 1:numel(speedsKmh)
        row = row + 1;
        result = results{iRoad, iSpeed};
        d = result.data;
        firstIndex = find(d.EstimatedTotalTireNormalForce_N <= 0, 1, 'first');
        Road(row) = string(roadNames{iRoad});
        Speed_kmh(row) = speedsKmh(iSpeed);
        if ~isempty(firstIndex)
            HasPredictedContactLoss(row) = true;
            FirstNonpositiveTime_s(row) = d.Time_s(firstIndex);
            FirstNonpositiveDistance_m(row) = d.Distance_m(firstIndex);
            entryTime = cfg.road.obstacleStart_m / result.speed_mps;
            FirstNonpositiveTauFromObstacleEntry_s(row) = ...
                d.Time_s(firstIndex) - entryTime;
            FirstNonpositiveNormalForce_N(row) = ...
                d.EstimatedTotalTireNormalForce_N(firstIndex);
        end
    end
end

contactTable = table(Road, Speed_kmh, HasPredictedContactLoss, ...
    FirstNonpositiveTime_s, FirstNonpositiveDistance_m, ...
    FirstNonpositiveTauFromObstacleEntry_s, FirstNonpositiveNormalForce_N);
end
