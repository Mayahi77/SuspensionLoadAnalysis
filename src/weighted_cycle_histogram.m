function counts = weighted_cycle_histogram(cycles, edges)
%WEIGHTED_CYCLE_HISTOGRAM Accumulate rainflow counts, including half cycles.

counts = zeros(numel(edges) - 1, 1);
if isempty(cycles)
    return
end
bin = discretize(cycles.ForceRange_N, edges);
if any(isnan(bin))
    error('Phase3:HistogramRange', ...
        'At least one rainflow range lies outside the common histogram edges.');
end
counts = accumarray(bin, cycles.CycleCount, [numel(edges)-1, 1], @sum, 0);
end
