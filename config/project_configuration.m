function cfg = project_configuration()
%PROJECT_CONFIGURATION Physical, road, solver, and validation settings.

cfg.vehicle.ms_kg = 320;
cfg.vehicle.mu_kg = 45;
cfg.vehicle.ks_Npm = 18000;
cfg.vehicle.cs_NsPm = 1500;
cfg.vehicle.kt_Npm = 190000;
cfg.vehicle.g_mps2 = 9.81;

cfg.track.length_m = 500;
cfg.output.sampleRate_Hz = 200;

% Isolated profiles use sin^2 shapes, giving zero height and zero slope at
% each endpoint. Both begin after a long flat approach.
cfg.road.obstacleStart_m = 100;
cfg.road.bumpHeight_m = 0.060;
cfg.road.bumpLength_m = 3.50;
cfg.road.potholeDepth_m = 0.045;
cfg.road.potholeLength_m = 1.20;

% Roughness is a deterministic harmonic sum. The amplitude is normalized
% to the nominal RMS below before application of the spatial envelope.
cfg.road.roughSeed = 20261013;
cfg.road.roughHarmonicCount = 32;
cfg.road.roughMinFrequency_cpm = 0.08;
cfg.road.roughMaxFrequency_cpm = 1.50;
cfg.road.roughNominalRMS_m = 0.004;
cfg.road.roughStart_m = 40;
cfg.road.roughRampLength_m = 15;
cfg.road.roughEnd_m = 460;

cfg.solver.RelTol = 1e-7;
cfg.solver.AbsTol = 1e-9;
cfg.solver.outputStep_s = 1 / cfg.output.sampleRate_Hz;
cfg.solver.initialState = zeros(4, 1);

cfg.validation.zeroTolerance = 1e-12;
cfg.validation.forceTolerance_N = 1e-9;
cfg.validation.dissipationTolerance_W = 1e-10;
cfg.validation.determinismTolerance = 1e-12;
cfg.validation.strictRelTol = 1e-9;
cfg.validation.strictAbsTol = 1e-11;
cfg.validation.convergenceRelativeTolerance = 1e-3;

cfg.phase2.eventWindowStart_s = 0;
cfg.phase2.eventWindowEnd_s = 3;
cfg.phase2.eventPlotStart_s = -0.5;
cfg.phase2.eventPlotEnd_s = 3;
cfg.phase2.roughPlotStart_m = 100;
cfg.phase2.roughPlotEnd_m = 160;
cfg.phase2.samplingRate_Hz = 1000;
cfg.phase2.samplingDifferenceThresholdPercent = 1;

cfg.phase3.referenceForce_N = 1000;
cfg.phase3.severityExponents = [3, 5];
cfg.phase3.histogramBinCount = 24;
cfg.phase3.cumulativeThresholdCount = 121;
cfg.phase3.samplingRate_Hz = 1000;
cfg.phase3.scalingRelativeTolerance = 1e-10;
cfg.phase3.identityTolerance = 1e-12;

cfg.phase4.enabled = true;
cfg.phase4.frameRate_Hz = 30;
cfg.phase4.tauStart_s = -0.5;
cfg.phase4.tauEnd_s = 3.0;
cfg.phase4.playbackDuration_s = 12.0;
cfg.phase4.motionScale = 8.0;
cfg.phase4.roadViewHalfWidth_m = 1.5;
end
