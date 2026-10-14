function validation = validate_model(cfg, results, roadNames, speedsKmh)
%VALIDATE_MODEL Execute Phase 1 sanity, repeatability, and convergence checks.

checkName = strings(0, 1);
category = strings(0, 1);
passed = false(0, 1);
measuredValue = zeros(0, 1);
acceptanceThreshold = zeros(0, 1);
units = strings(0, 1);
details = strings(0, 1);

    function add_check(name, group, passValue, value, threshold, unitText, detailText)
        checkName(end+1, 1) = string(name);
        category(end+1, 1) = string(group);
        passed(end+1, 1) = logical(passValue);
        measuredValue(end+1, 1) = value;
        acceptanceThreshold(end+1, 1) = threshold;
        units(end+1, 1) = string(unitText);
        details(end+1, 1) = string(detailText);
    end

smoothMax = 0;
allFiniteAndEqual = true;
maxForceError = 0;
minDissipation = inf;
contactLossCount = 0;
for iRoad = 1:numel(roadNames)
    for iSpeed = 1:numel(speedsKmh)
        d = results{iRoad, iSpeed}.data;
        if strcmp(roadNames{iRoad}, 'smooth')
            % Columns 3:15 are dynamic signals. Column 16 is the estimated
            % total tire normal force and correctly contains the static load.
            responseColumns = d{:, 3:15};
            smoothMax = max(smoothMax, max(abs(responseColumns), [], 'all'));
        end
        lengths = varfun(@numel, d, 'OutputFormat', 'uniform');
        numericValues = d{:, 1:16};
        allFiniteAndEqual = allFiniteAndEqual && all(lengths == height(d)) && ...
            all(isfinite(numericValues), 'all');
        reconstructed = cfg.vehicle.ks_Npm * d.SuspensionDeflection_zs_minus_zu_m + ...
            cfg.vehicle.cs_NsPm * d.SuspensionRelativeVelocity_vs_minus_vu_mps;
        maxForceError = max(maxForceError, ...
            max(abs(d.DynamicSuspensionForce_Fs_N - reconstructed)));
        dissipation = d.SuspensionRelativeVelocity_vs_minus_vu_mps .* ...
            d.DamperForce_cs_vs_minus_vu_N;
        minDissipation = min(minDissipation, min(dissipation));

        hasLoss = any(d.PredictedContactLoss);
        contactLossCount = contactLossCount + double(hasLoss);
        add_check(sprintf('tire_contact_%s_%dkmh', roadNames{iRoad}, speedsKmh(iSpeed)), ...
            'contact', ~hasLoss, min(d.EstimatedTotalTireNormalForce_N), 0, 'N', ...
            'Pass means estimated total normal force remained positive; values are never clamped.');
    end
end

add_check('smooth_zero_response', 'required', smoothMax <= cfg.validation.zeroTolerance, ...
    smoothMax, cfg.validation.zeroTolerance, 'SI mixed', ...
    'Maximum absolute dynamic response across both smooth-road cases.');
add_check('finite_equal_length_outputs', 'required', allFiniteAndEqual, ...
    double(~allFiniteAndEqual), 0, 'boolean error', ...
    'Every output column is finite and has table height.');
add_check('suspension_force_reconstruction', 'required', ...
    maxForceError <= cfg.validation.forceTolerance_N, maxForceError, ...
    cfg.validation.forceTolerance_N, 'N', ...
    ['Algebraic consistency check only: exported Fs compared with ' ...
     'ks*(zs-zu)+cs*(vs-vu); this is not independent verification of the dynamics.']);
add_check('nonnegative_damper_dissipation', 'required', ...
    minDissipation >= -cfg.validation.dissipationTolerance_W, minDissipation, ...
    -cfg.validation.dissipationTolerance_W, 'W', ...
    '(vs-vu)*Fd = cs*(vs-vu)^2 must be nonnegative within roundoff.');

% Repeat a nonzero stochastic-profile case to verify exact reproducibility.
roughIndex = find(strcmp(roadNames, 'rough'), 1);
speedIndex = find(speedsKmh == 60, 1);
repeatResult = simulate_case('rough', 60, cfg, cfg.solver);
baseline = results{roughIndex, speedIndex}.data;
determinismError = max(abs(repeatResult.data{:, 3:16} - baseline{:, 3:16}), [], 'all');
add_check('deterministic_repeat_rough_60kmh', 'required', ...
    determinismError <= cfg.validation.determinismTolerance, determinismError, ...
    cfg.validation.determinismTolerance, 'maximum absolute difference', ...
    'Independent repeat compared at every exported sample.');

nConvergenceCases = (numel(roadNames) - 1) * numel(speedsKmh);
convRoad = strings(nConvergenceCases, 1);
convSpeed = zeros(nConvergenceCases, 1);
forceMaxAbsDifference_N = zeros(nConvergenceCases, 1);
forceRelativeDifference = zeros(nConvergenceCases, 1);
accelMaxAbsDifference_mps2 = zeros(nConvergenceCases, 1);
accelRelativeDifference = zeros(nConvergenceCases, 1);
deflectionMaxAbsDifference_m = zeros(nConvergenceCases, 1);
deflectionRelativeDifference = zeros(nConvergenceCases, 1);
convergencePassed = false(nConvergenceCases, 1);
convergenceIndex = 0;

strictSolver = cfg.solver;
strictSolver.RelTol = cfg.validation.strictRelTol;
strictSolver.AbsTol = cfg.validation.strictAbsTol;
for iRoad = 1:numel(roadNames)
    if strcmp(roadNames{iRoad}, 'smooth')
        continue
    end
    for iSpeed = 1:numel(speedsKmh)
        baseResult = results{iRoad, iSpeed};
        strictSolver.MaxStep = baseResult.maxStep_s / 2;
        strictResult = simulate_case(roadNames{iRoad}, speedsKmh(iSpeed), cfg, strictSolver);
        base = baseResult.data;
        strict = strictResult.data;

        forceAbs = max(abs(base.DynamicSuspensionForce_Fs_N - strict.DynamicSuspensionForce_Fs_N));
        accelAbs = max(abs(base.BodyAcceleration_mps2 - strict.BodyAcceleration_mps2));
        deflectionAbs = max(abs(base.SuspensionDeflection_zs_minus_zu_m - ...
            strict.SuspensionDeflection_zs_minus_zu_m));
        forceRel = forceAbs / max(max(abs(strict.DynamicSuspensionForce_Fs_N)), 1);
        accelRel = accelAbs / max(max(abs(strict.BodyAcceleration_mps2)), 1e-6);
        deflectionRel = deflectionAbs / max(max(abs(strict.SuspensionDeflection_zs_minus_zu_m)), 1e-6);
        didPass = max([forceRel, accelRel, deflectionRel]) <= ...
            cfg.validation.convergenceRelativeTolerance;

        convergenceIndex = convergenceIndex + 1;
        convRoad(convergenceIndex) = string(roadNames{iRoad});
        convSpeed(convergenceIndex) = speedsKmh(iSpeed);
        forceMaxAbsDifference_N(convergenceIndex) = forceAbs;
        forceRelativeDifference(convergenceIndex) = forceRel;
        accelMaxAbsDifference_mps2(convergenceIndex) = accelAbs;
        accelRelativeDifference(convergenceIndex) = accelRel;
        deflectionMaxAbsDifference_m(convergenceIndex) = deflectionAbs;
        deflectionRelativeDifference(convergenceIndex) = deflectionRel;
        convergencePassed(convergenceIndex) = didPass;
    end
end

convergence = table(convRoad, convSpeed, forceMaxAbsDifference_N, ...
    forceRelativeDifference, accelMaxAbsDifference_mps2, accelRelativeDifference, ...
    deflectionMaxAbsDifference_m, deflectionRelativeDifference, convergencePassed, ...
    'VariableNames', {'Road', 'Speed_kmh', 'ForceMaxAbsDifference_N', ...
    'ForceRelativeDifference', 'AccelerationMaxAbsDifference_mps2', ...
    'AccelerationRelativeDifference', 'DeflectionMaxAbsDifference_m', ...
    'DeflectionRelativeDifference', 'Passed'});
maxConvergenceError = max([forceRelativeDifference; accelRelativeDifference; ...
    deflectionRelativeDifference]);
add_check('strict_solver_convergence_all_nonzero_cases', 'required', ...
    all(convergencePassed), maxConvergenceError, ...
    cfg.validation.convergenceRelativeTolerance, 'relative difference', ...
    'All six nonzero cases; detailed differences are in convergence_results.csv.');

checks = table(checkName, category, passed, measuredValue, acceptanceThreshold, ...
    units, details, 'VariableNames', {'Check', 'Category', 'Passed', ...
    'MeasuredValue', 'AcceptanceThreshold', 'Units', 'Details'});
required = checks.Category == "required";
validation.checks = checks;
validation.convergence = convergence;
validation.requiredChecksPassed = all(checks.Passed(required));
validation.contactLossCount = contactLossCount;
end
