function phase2 = run_phase2_analysis(projectRoot, cfg, results, roadNames, speedsKmh)
%RUN_PHASE2_ANALYSIS Create Phase 2 metrics, checks, and static figures.

figureDir = fullfile(projectRoot, 'outputs', 'figures');
if ~isfolder(figureDir)
    mkdir(figureDir);
end

phase2.eventMetrics = calculate_event_metrics(results, roadNames, speedsKmh, cfg);
phase2.contactValidity = analyze_contact_validity(results, roadNames, speedsKmh, cfg);
fprintf('Running Phase 2 output-sampling check at 1000 Hz...\n');
phase2.samplingCheck = check_output_sampling(results, roadNames, speedsKmh, cfg);

writetable(phase2.eventMetrics, fullfile(projectRoot, 'outputs', 'event_metrics.csv'));
writetable(phase2.contactValidity, fullfile(projectRoot, 'outputs', 'contact_validity.csv'));
writetable(phase2.samplingCheck, fullfile(projectRoot, 'outputs', 'output_sampling_check.csv'));

fprintf('Creating Phase 2 figures...\n');
create_phase2_figures(results, roadNames, speedsKmh, cfg, figureDir);
end
