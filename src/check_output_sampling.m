function samplingCheck = check_output_sampling(results, roadNames, speedsKmh, cfg)
%CHECK_OUTPUT_SAMPLING Compare 200 Hz exports with 1000 Hz exports.
% Solver RelTol, AbsTol, and MaxStep remain identical to each corresponding
% baseline case. This is an output-grid check, not a solver-convergence test.

checkRoads = {'speed_bump', 'pothole', 'rough'};
nRows = numel(checkRoads);
Road = strings(nRows, 1);
Speed_kmh = repmat(60, nRows, 1);
BaselineOutputRate_Hz = repmat(cfg.output.sampleRate_Hz, nRows, 1);
ComparisonOutputRate_Hz = repmat(cfg.phase2.samplingRate_Hz, nRows, 1);
SolverMaxStep_s = zeros(nRows, 1);
BaselinePeakAbsForce_N = zeros(nRows, 1);
ComparisonPeakAbsForce_N = zeros(nRows, 1);
PeakForceAbsDifference_N = zeros(nRows, 1);
PeakForceRelativeDifference_percent = zeros(nRows, 1);
BaselinePeakAbsTravel_m = zeros(nRows, 1);
ComparisonPeakAbsTravel_m = zeros(nRows, 1);
PeakTravelAbsDifference_m = zeros(nRows, 1);
PeakTravelRelativeDifference_percent = zeros(nRows, 1);
BaselineMinimumNormalForce_N = zeros(nRows, 1);
ComparisonMinimumNormalForce_N = zeros(nRows, 1);
MinimumNormalForceAbsDifference_N = zeros(nRows, 1);
MinimumNormalForceRelativeDifference_percent = zeros(nRows, 1);
PeakForceDifferenceAboveOnePercent = false(nRows, 1);
PeakTravelDifferenceAboveOnePercent = false(nRows, 1);
MinimumNormalForceDifferenceAboveOnePercent = false(nRows, 1);
AnyComparedMetricDifferenceAboveOnePercent = false(nRows, 1);

iSpeed = find(speedsKmh == 60, 1);
for row = 1:nRows
    roadName = checkRoads{row};
    iRoad = find(strcmp(roadNames, roadName), 1);
    baseline = results{iRoad, iSpeed};
    settings = cfg.solver;
    settings.outputStep_s = 1 / cfg.phase2.samplingRate_Hz;
    settings.MaxStep = baseline.maxStep_s;
    comparison = simulate_case(roadName, 60, cfg, settings);

    baseForce = max(abs(baseline.data.DynamicSuspensionForce_Fs_N));
    compForce = max(abs(comparison.data.DynamicSuspensionForce_Fs_N));
    baseTravel = max(abs(baseline.data.SuspensionDeflection_zs_minus_zu_m));
    compTravel = max(abs(comparison.data.SuspensionDeflection_zs_minus_zu_m));
    baseNormal = min(baseline.data.EstimatedTotalTireNormalForce_N);
    compNormal = min(comparison.data.EstimatedTotalTireNormalForce_N);

    forceAbs = abs(compForce - baseForce);
    travelAbs = abs(compTravel - baseTravel);
    normalAbs = abs(compNormal - baseNormal);
    forceRel = relative_percent(forceAbs, compForce);
    travelRel = relative_percent(travelAbs, compTravel);
    normalRel = relative_percent(normalAbs, compNormal);

    Road(row) = string(roadName);
    SolverMaxStep_s(row) = baseline.maxStep_s;
    BaselinePeakAbsForce_N(row) = baseForce;
    ComparisonPeakAbsForce_N(row) = compForce;
    PeakForceAbsDifference_N(row) = forceAbs;
    PeakForceRelativeDifference_percent(row) = forceRel;
    BaselinePeakAbsTravel_m(row) = baseTravel;
    ComparisonPeakAbsTravel_m(row) = compTravel;
    PeakTravelAbsDifference_m(row) = travelAbs;
    PeakTravelRelativeDifference_percent(row) = travelRel;
    BaselineMinimumNormalForce_N(row) = baseNormal;
    ComparisonMinimumNormalForce_N(row) = compNormal;
    MinimumNormalForceAbsDifference_N(row) = normalAbs;
    MinimumNormalForceRelativeDifference_percent(row) = normalRel;
    threshold = cfg.phase2.samplingDifferenceThresholdPercent;
    PeakForceDifferenceAboveOnePercent(row) = forceRel > threshold;
    PeakTravelDifferenceAboveOnePercent(row) = travelRel > threshold;
    MinimumNormalForceDifferenceAboveOnePercent(row) = normalRel > threshold;
    AnyComparedMetricDifferenceAboveOnePercent(row) = ...
        max([forceRel, travelRel, normalRel]) > threshold;
end

samplingCheck = table(Road, Speed_kmh, BaselineOutputRate_Hz, ...
    ComparisonOutputRate_Hz, SolverMaxStep_s, BaselinePeakAbsForce_N, ...
    ComparisonPeakAbsForce_N, PeakForceAbsDifference_N, ...
    PeakForceRelativeDifference_percent, BaselinePeakAbsTravel_m, ...
    ComparisonPeakAbsTravel_m, PeakTravelAbsDifference_m, ...
    PeakTravelRelativeDifference_percent, BaselineMinimumNormalForce_N, ...
    ComparisonMinimumNormalForce_N, MinimumNormalForceAbsDifference_N, ...
    MinimumNormalForceRelativeDifference_percent, ...
    PeakForceDifferenceAboveOnePercent, PeakTravelDifferenceAboveOnePercent, ...
    MinimumNormalForceDifferenceAboveOnePercent, ...
    AnyComparedMetricDifferenceAboveOnePercent);
end

function percent = relative_percent(absoluteDifference, comparisonValue)
percent = 100 * absoluteDifference / max(abs(comparisonValue), eps);
end
