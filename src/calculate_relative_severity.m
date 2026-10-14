function severity = calculate_relative_severity(cycles, referenceForce_N, exponents)
%CALCULATE_RELATIVE_SEVERITY Dimensionless load severity over one exposure.
% S_m = sum(count_i * (amplitude_i/F_ref)^m). This is not Miner damage,
% fatigue life, or a safety assessment.

severity = zeros(1, numel(exponents));
for k = 1:numel(exponents)
    severity(k) = sum(cycles.CycleCount .* ...
        (cycles.ForceAmplitude_N / referenceForce_N).^exponents(k));
end
end
