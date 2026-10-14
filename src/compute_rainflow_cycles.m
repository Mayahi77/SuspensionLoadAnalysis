function cycles = compute_rainflow_cycles(force_N)
%COMPUTE_RAINFLOW_CYCLES Count signed dynamic-force cycles with MATLAB rainflow.
% The official Signal Processing Toolbox implementation is used without
% filtering or a cycle threshold. MATLAB's residual/endpoint half cycles are
% retained exactly as returned. Means are means of dynamic Fs, not means of
% total component load.

force_N = force_N(:);
variableNames = {'CycleCount', 'ForceRange_N', 'ForceAmplitude_N', ...
    'DynamicForceMean_N'};
if isempty(force_N) || all(force_N == 0)
    cycles = array2table(zeros(0, 4), 'VariableNames', variableNames);
    return
end

raw = rainflow(force_N);
if isempty(raw)
    cycles = array2table(zeros(0, 4), 'VariableNames', variableNames);
    return
end

count = raw(:, 1);
range_N = raw(:, 2);
amplitude_N = range_N / 2;
dynamicMean_N = raw(:, 3);
cycles = table(count, range_N, amplitude_N, dynamicMean_N, ...
    'VariableNames', variableNames);
end
