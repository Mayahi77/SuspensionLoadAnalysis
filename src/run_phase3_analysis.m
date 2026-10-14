function phase3 = run_phase3_analysis(projectRoot, cfg, results, roadNames, speedsKmh)
%RUN_PHASE3_ANALYSIS Rainflow spectra and relative load-severity indicators.
% Uses MATLAB's documented rainflow implementation. The analysis is an
% equal-distance 500 m load comparison, not fatigue damage, life, or safety.

phase3Dir = fullfile(projectRoot, 'outputs', 'phase3');
cycleDir = fullfile(phase3Dir, 'cycles');
figureDir = fullfile(phase3Dir, 'figures');
if ~isfolder(cycleDir), mkdir(cycleDir); end
if ~isfolder(figureDir), mkdir(figureDir); end

phase3.availability = check_rainflow_availability();
writetable(phase3.availability, fullfile(phase3Dir, 'rainflow_availability.csv'));
if ~phase3.availability.ReadyForRainflowAnalysis
    error('Phase3:RainflowUnavailable', '%s', char(phase3.availability.Limitation));
end

fprintf('  Counting complete 500 m signed Fs histories with MATLAB rainflow...\n');
nCases = numel(results);
Road = strings(nCases, 1);
Speed_kmh = zeros(nCases, 1);
TrackLength_m = repmat(cfg.track.length_m, nCases, 1);
ContactValid = true(nCases, 1);
Eligible = true(nCases, 1);
MinimumEstimatedTotalTireNormalForce_N = nan(nCases, 1);
FirstNonpositiveTime_s = nan(nCases, 1);
FirstNonpositiveDistance_m = nan(nCases, 1);
ExclusionReason = strings(nCases, 1);
CycleTableFile = strings(nCases, 1);
RainflowRows = zeros(nCases, 1);
EquivalentCycleCount = zeros(nCases, 1);
ResidualHalfCycleRows = zeros(nCases, 1);
Severity_m3 = nan(nCases, 1);
Severity_m5 = nan(nCases, 1);
cycles = cell(size(results));
row = 0;

for iRoad = 1:numel(roadNames)
    for iSpeed = 1:numel(speedsKmh)
        row = row + 1;
        Road(row) = string(roadNames{iRoad});
        Speed_kmh(row) = speedsKmh(iSpeed);
        result = results{iRoad, iSpeed};
        normalForce = result.data.EstimatedTotalTireNormalForce_N;
        firstNonpositive = find(normalForce <= 0, 1, 'first');
        MinimumEstimatedTotalTireNormalForce_N(row) = min(normalForce);
        ContactValid(row) = isempty(firstNonpositive);
        Eligible(row) = ContactValid(row);
        if ~ContactValid(row)
            FirstNonpositiveTime_s(row) = result.data.Time_s(firstNonpositive);
            FirstNonpositiveDistance_m(row) = result.data.Distance_m(firstNonpositive);
            Eligible(row) = false;
            ExclusionReason(row) = string(sprintf(['Entire case excluded: estimated total tire ' ...
                'normal force first became nonpositive at t = %.6g s and x = %.6g m; ' ...
                'the bilateral-tire trajectory is invalid for comparison.'], ...
                FirstNonpositiveTime_s(row), FirstNonpositiveDistance_m(row)));
            cycles{iRoad, iSpeed} = empty_cycle_table();
            continue
        end

        c = compute_rainflow_cycles(result.data.DynamicSuspensionForce_Fs_N);
        cycles{iRoad, iSpeed} = c;
        fileName = sprintf('%s_%02dkmh_cycles.csv', roadNames{iRoad}, speedsKmh(iSpeed));
        writetable(c, fullfile(cycleDir, fileName));
        CycleTableFile(row) = string(fullfile('cycles', fileName));
        RainflowRows(row) = height(c);
        EquivalentCycleCount(row) = sum(c.CycleCount);
        ResidualHalfCycleRows(row) = sum(c.CycleCount == 0.5);
        s = calculate_relative_severity(c, cfg.phase3.referenceForce_N, ...
            cfg.phase3.severityExponents);
        Severity_m3(row) = s(1);
        Severity_m5(row) = s(2);
    end
end

Rank_m3 = dense_descending_rank(Severity_m3, Eligible);
Rank_m5 = dense_descending_rank(Severity_m5, Eligible);
phase3.severitySummary = table(Road, Speed_kmh, TrackLength_m, ContactValid, ...
    Eligible, MinimumEstimatedTotalTireNormalForce_N, FirstNonpositiveTime_s, ...
    FirstNonpositiveDistance_m, ExclusionReason, CycleTableFile, RainflowRows, ...
    EquivalentCycleCount, ResidualHalfCycleRows, Severity_m3, Rank_m3, ...
    Severity_m5, Rank_m5);
writetable(phase3.severitySummary, fullfile(phase3Dir, 'relative_severity_summary.csv'));

exclusions = phase3.severitySummary(~phase3.severitySummary.Eligible, ...
    {'Road', 'Speed_kmh', 'TrackLength_m', 'ContactValid', ...
    'MinimumEstimatedTotalTireNormalForce_N', 'FirstNonpositiveTime_s', ...
    'FirstNonpositiveDistance_m', 'ExclusionReason'});
writetable(exclusions, fullfile(phase3Dir, 'excluded_cases.csv'));

phase3.speedRatios = build_speed_ratios(phase3.severitySummary, roadNames);
writetable(phase3.speedRatios, fullfile(phase3Dir, 'severity_speed_ratios.csv'));

eligibleMask = reshape(Eligible, [numel(speedsKmh), numel(roadNames)]).';
eligibleRanges = cellfun(@(c) c.ForceRange_N, cycles(eligibleMask), ...
    'UniformOutput', false);
allRanges = vertcat(eligibleRanges{:});
maxRange = max(allRanges);
commonEdges = linspace(0, maxRange * (1 + 1e-12), ...
    cfg.phase3.histogramBinCount + 1).';
cumulativeThresholds = linspace(0, maxRange, ...
    cfg.phase3.cumulativeThresholdCount).';
[phase3.histograms, phase3.cumulativeExceedance] = build_spectrum_tables( ...
    cycles, Eligible, roadNames, speedsKmh, commonEdges, cumulativeThresholds);
writetable(phase3.histograms, fullfile(phase3Dir, 'load_spectrum_histograms.csv'));
writetable(phase3.cumulativeExceedance, ...
    fullfile(phase3Dir, 'cumulative_exceedance.csv'));

fprintf('  Comparing 200 Hz and 1000 Hz severity for valid 60 km/h cases...\n');
phase3.samplingSensitivity = severity_sampling_sensitivity(results, cycles, ...
    phase3.severitySummary, roadNames, speedsKmh, cfg);
writetable(phase3.samplingSensitivity, ...
    fullfile(phase3Dir, 'severity_sampling_sensitivity.csv'));

phase3.validation = validate_phase3(cycles, Eligible, commonEdges, ...
    results, roadNames, speedsKmh, cfg);
writetable(phase3.validation, fullfile(phase3Dir, 'phase3_validation.csv'));
phase3.validationPassed = all(phase3.validation.Passed);

fprintf('  Creating Phase 3 load-spectrum figures...\n');
create_phase3_figures(cycles, phase3.severitySummary, roadNames, speedsKmh, ...
    commonEdges, cumulativeThresholds, cfg, figureDir);
write_phase3_status(fullfile(phase3Dir, 'phase3_status.txt'), phase3, cfg);
phase3.cycles = cycles;
end

function c = empty_cycle_table()
c = array2table(zeros(0, 4), 'VariableNames', {'CycleCount', ...
    'ForceRange_N', 'ForceAmplitude_N', 'DynamicForceMean_N'});
end

function rankValues = dense_descending_rank(values, eligible)
rankValues = nan(size(values));
uniqueValues = unique(values(eligible & isfinite(values)), 'sorted');
uniqueValues = flipud(uniqueValues);
for k = 1:numel(uniqueValues)
    rankValues(eligible & values == uniqueValues(k)) = k;
end
end

function ratios = build_speed_ratios(summary, roadNames)
n = numel(roadNames);
Road = string(roadNames(:));
Ratio60over30_m3 = nan(n, 1);
Ratio60over30_m5 = nan(n, 1);
RatioAvailable = false(n, 1);
Reason = strings(n, 1);
for k = 1:n
    rows = find(summary.Road == Road(k));
    r30 = rows(summary.Speed_kmh(rows) == 30);
    r60 = rows(summary.Speed_kmh(rows) == 60);
    if ~summary.Eligible(r30) || ~summary.Eligible(r60)
        Reason(k) = "N/A: one speed is excluded for contact invalidity.";
    elseif summary.Severity_m3(r30) == 0 || summary.Severity_m5(r30) == 0
        Reason(k) = "N/A: 30 km/h denominator is zero.";
    else
        Ratio60over30_m3(k) = summary.Severity_m3(r60) / summary.Severity_m3(r30);
        Ratio60over30_m5(k) = summary.Severity_m5(r60) / summary.Severity_m5(r30);
        RatioAvailable(k) = true;
        Reason(k) = "";
    end
end
ratios = table(Road, Ratio60over30_m3, Ratio60over30_m5, RatioAvailable, Reason);
end

function [histTable, cumulativeTable] = build_spectrum_tables(cycles, eligible, ...
    roadNames, speedsKmh, edges, thresholds)
histParts = cell(nnz(eligible), 1);
cumParts = cell(nnz(eligible), 1);
part = 0; row = 0;
for iRoad = 1:numel(roadNames)
    for iSpeed = 1:numel(speedsKmh)
        row = row + 1;
        if ~eligible(row), continue; end
        part = part + 1;
        c = cycles{iRoad, iSpeed};
        weightedCounts = weighted_cycle_histogram(c, edges);
        nBins = numel(weightedCounts);
        histParts{part} = table(repmat(string(roadNames{iRoad}), nBins, 1), ...
            repmat(speedsKmh(iSpeed), nBins, 1), edges(1:end-1), edges(2:end), ...
            (edges(1:end-1)+edges(2:end))/2, weightedCounts, ...
            'VariableNames', {'Road', 'Speed_kmh', 'BinLower_N', 'BinUpper_N', ...
            'BinCenter_N', 'WeightedCycleCount'});
        exceedance = zeros(size(thresholds));
        for k = 1:numel(thresholds)
            exceedance(k) = sum(c.CycleCount(c.ForceRange_N >= thresholds(k)));
        end
        nThresholds = numel(thresholds);
        cumParts{part} = table(repmat(string(roadNames{iRoad}), nThresholds, 1), ...
            repmat(speedsKmh(iSpeed), nThresholds, 1), thresholds, exceedance, ...
            'VariableNames', {'Road', 'Speed_kmh', 'RangeThreshold_N', ...
            'CyclesAtOrAboveThreshold'});
    end
end
histTable = vertcat(histParts{:});
cumulativeTable = vertcat(cumParts{:});
end

function sampling = severity_sampling_sensitivity(results, cycles, summary, ...
    roadNames, speedsKmh, cfg)
checkRoads = {'speed_bump', 'rough'};
nRows = numel(checkRoads) * numel(cfg.phase3.severityExponents);
Road = strings(nRows, 1); Speed_kmh = repmat(60, nRows, 1);
Exponent_m = zeros(nRows, 1);
BaselineOutputRate_Hz = repmat(cfg.output.sampleRate_Hz, nRows, 1);
ComparisonOutputRate_Hz = repmat(cfg.phase3.samplingRate_Hz, nRows, 1);
BaselineContactValid = false(nRows, 1);
ComparisonContactValid = false(nRows, 1);
EligibleForComparison = false(nRows, 1);
ExclusionReason = strings(nRows, 1);
BaselineSeverity = nan(nRows, 1); ComparisonSeverity = nan(nRows, 1);
AbsoluteDifference = nan(nRows, 1); PercentageDifference = nan(nRows, 1);
i60 = find(speedsKmh == 60, 1); row = 0;
for k = 1:numel(checkRoads)
    iRoad = find(strcmp(roadNames, checkRoads{k}), 1);
    baseline = results{iRoad, i60};
    summaryRow = find(summary.Road == string(checkRoads{k}) & ...
        summary.Speed_kmh == 60, 1);
    baselineValid = summary.Eligible(summaryRow);
    settings = cfg.solver;
    settings.outputStep_s = 1 / cfg.phase3.samplingRate_Hz;
    settings.MaxStep = baseline.maxStep_s;
    comparison = simulate_case(checkRoads{k}, 60, cfg, settings);
    comparisonValid = all(comparison.data.EstimatedTotalTireNormalForce_N > 0);
    pairEligible = baselineValid && comparisonValid;
    if pairEligible
        baseS = calculate_relative_severity(cycles{iRoad, i60}, ...
            cfg.phase3.referenceForce_N, cfg.phase3.severityExponents);
        compCycles = compute_rainflow_cycles(comparison.data.DynamicSuspensionForce_Fs_N);
        compS = calculate_relative_severity(compCycles, cfg.phase3.referenceForce_N, ...
            cfg.phase3.severityExponents);
        reason = "";
    else
        baseS = nan(size(cfg.phase3.severityExponents));
        compS = nan(size(cfg.phase3.severityExponents));
        reason = "Excluded because the baseline or comparison output predicts nonpositive tire normal force.";
    end
    for j = 1:numel(cfg.phase3.severityExponents)
        row = row + 1;
        Road(row) = string(checkRoads{k});
        Exponent_m(row) = cfg.phase3.severityExponents(j);
        BaselineContactValid(row) = baselineValid;
        ComparisonContactValid(row) = comparisonValid;
        EligibleForComparison(row) = pairEligible;
        ExclusionReason(row) = reason;
        BaselineSeverity(row) = baseS(j);
        ComparisonSeverity(row) = compS(j);
        if pairEligible
            AbsoluteDifference(row) = abs(compS(j) - baseS(j));
            PercentageDifference(row) = 100 * AbsoluteDifference(row) / max(abs(compS(j)), eps);
        end
    end
end
sampling = table(Road, Speed_kmh, Exponent_m, BaselineOutputRate_Hz, ...
    ComparisonOutputRate_Hz, BaselineContactValid, ComparisonContactValid, ...
    EligibleForComparison, ExclusionReason, BaselineSeverity, ...
    ComparisonSeverity, AbsoluteDifference, PercentageDifference);
end

function validation = validate_phase3(cycles, eligible, edges, results, ...
    roadNames, speedsKmh, cfg)
% Expected before execution: [0,1000,0,-1000,0] has three residual half
% cycles: (range,mean) = (1000,500), (2000,0), (1000,-500).
expected = table([0.5;0.5;0.5], [1000;2000;1000], [500;0;-500], ...
    'VariableNames', {'CycleCount', 'ForceRange_N', 'DynamicForceMean_N'});
manual = compute_rainflow_cycles([0;1000;0;-1000;0]);
actual = manual(:, {'CycleCount', 'ForceRange_N', 'DynamicForceMean_N'});
expected = sortrows(expected, {'ForceRange_N', 'DynamicForceMean_N'});
actual = sortrows(actual, {'ForceRange_N', 'DynamicForceMean_N'});
manualError = max(abs(actual{:,:} - expected{:,:}), [], 'all');

amplitudeError = 0; minCount = inf; minRange = inf; histogramError = 0;
row = 0;
for iRoad = 1:size(cycles, 1)
    for iSpeed = 1:size(cycles, 2)
        row = row + 1;
        if ~eligible(row), continue; end
        c = cycles{iRoad, iSpeed};
        if ~isempty(c)
            amplitudeError = max(amplitudeError, ...
                max(abs(c.ForceAmplitude_N - c.ForceRange_N/2)));
            minCount = min(minCount, min(c.CycleCount));
            minRange = min(minRange, min(c.ForceRange_N));
        end
        histogramError = max(histogramError, ...
            abs(sum(weighted_cycle_histogram(c, edges)) - sum(c.CycleCount)));
    end
end
if isinf(minCount), minCount = 0; end
if isinf(minRange), minRange = 0; end

iBump = find(strcmp(roadNames, 'speed_bump'), 1);
i30 = find(speedsKmh == 30, 1);
baseCycles = cycles{iBump, i30};
scaledCycles = compute_rainflow_cycles(2 * ...
    results{iBump, i30}.data.DynamicSuspensionForce_Fs_N);
baseSeverity = calculate_relative_severity(baseCycles, cfg.phase3.referenceForce_N, ...
    cfg.phase3.severityExponents);
scaledSeverity = calculate_relative_severity(scaledCycles, cfg.phase3.referenceForce_N, ...
    cfg.phase3.severityExponents);
scalingError = abs(scaledSeverity ./ baseSeverity - ...
    2.^cfg.phase3.severityExponents);

iSmooth = find(strcmp(roadNames, 'smooth'), 1);
expectedEligible = false(size(eligible));
excludedTablesEmpty = true;
row = 0;
for iRoad = 1:numel(roadNames)
    for iSpeed = 1:numel(speedsKmh)
        row = row + 1;
        expectedEligible(row) = all( ...
            results{iRoad,iSpeed}.data.EstimatedTotalTireNormalForce_N > 0);
        if ~expectedEligible(row)
            excludedTablesEmpty = excludedTablesEmpty && isempty(cycles{iRoad,iSpeed});
        end
    end
end
invalidCasesStillEligible = nnz(~expectedEligible & eligible);
validCasesIncorrectlyExcluded = nnz(expectedEligible & ~eligible);
Test = ["Manual sequence expected residual half cycles"; ...
    "Amplitude equals range divided by two"; "Cycle counts are nonnegative"; ...
    "Force ranges are nonnegative"; "Histogram weighted counts reconcile"; ...
    "Scaling Fs by two scales S_3 by 2^3"; ...
    "Scaling Fs by two scales S_5 by 2^5"; ...
    "Smooth-road signals give zero cycles and severity"; ...
    "Every contact-invalid case is excluded"; ...
    "Every contact-valid case remains eligible"; ...
    "Excluded cases have no rainflow cycle table data"];
MeasuredValue = [manualError; amplitudeError; minCount; minRange; histogramError; ...
    scalingError(1); scalingError(2); ...
    height(cycles{iSmooth,1}) + height(cycles{iSmooth,2}); ...
    invalidCasesStillEligible; validCasesIncorrectlyExcluded; ...
    double(excludedTablesEmpty)];
AcceptanceCriterion = ["max absolute table difference <= 1e-12"; ...
    "max absolute identity error <= 1e-12 N"; "minimum count >= 0"; ...
    "minimum range >= 0 N"; "max absolute count difference <= 1e-12"; ...
    "relative multiplier error <= 1e-10"; ...
    "relative multiplier error <= 1e-10"; "cycle-table rows = 0"; ...
    "number of contact-invalid eligible cases = 0"; ...
    "number of contact-valid excluded cases = 0"; ...
    "all excluded cycle tables are empty"];
Passed = [manualError <= cfg.phase3.identityTolerance; ...
    amplitudeError <= cfg.phase3.identityTolerance; minCount >= 0; minRange >= 0; ...
    histogramError <= cfg.phase3.identityTolerance; ...
    scalingError(1) <= cfg.phase3.scalingRelativeTolerance; ...
    scalingError(2) <= cfg.phase3.scalingRelativeTolerance; ...
    isempty(cycles{iSmooth,1}) && isempty(cycles{iSmooth,2}); ...
    invalidCasesStillEligible == 0; validCasesIncorrectlyExcluded == 0; ...
    excludedTablesEmpty];
validation = table(Test, MeasuredValue, AcceptanceCriterion, Passed);
end

function write_phase3_status(filePath, phase3, cfg)
fid = fopen(filePath, 'w');
if fid < 0, error('Phase3:FileOpen', 'Could not open %s.', filePath); end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, 'Phase 3 rainflow load-spectrum analysis status\n\n');
fprintf(fid, 'Status: completed by an actual MATLAB run\n');
fprintf(fid, 'Rainflow implementation: %s\n', phase3.availability.FunctionPath);
fprintf(fid, 'Signal Processing Toolbox: %s %s; license checkout succeeded: %s\n', ...
    phase3.availability.ProductName, phase3.availability.ProductVersion, ...
    string(phase3.availability.LicenseCheckoutSucceeded));
fprintf(fid, 'Exposure: complete 500 m signed dynamic Fs history per eligible case.\n');
fprintf(fid, ['No filtering or cycle threshold was applied. MATLAB endpoint/residual ' ...
    'half cycles were retained; records were not forced periodic.\n']);
fprintf(fid, 'Cycle means are dynamic-force means, not total component-load means.\n');
excluded = phase3.severitySummary(~phase3.severitySummary.Eligible, :);
fprintf(fid, 'Contact-invalid cases excluded from the entire analysis: %d\n', height(excluded));
for k = 1:height(excluded)
    fprintf(fid, '  %s at %g km/h: %s\n', excluded.Road(k), ...
        excluded.Speed_kmh(k), excluded.ExclusionReason(k));
end
fprintf(fid, 'F_ref = %.0f N; exponents m = %g and %g are illustrative and uncalibrated.\n', ...
    cfg.phase3.referenceForce_N, cfg.phase3.severityExponents);
fprintf(fid, ['Severity is dimensionless relative load severity over 500 m, not Miner ' ...
    'damage, fatigue life, or a safety assessment. No mean-stress correction was used.\n']);
fprintf(fid, 'All required Phase 3 validations passed: %s\n', ...
    string(phase3.validationPassed));
clear cleanup
end
