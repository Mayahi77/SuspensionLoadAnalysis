function result = simulate_case(roadName, speedKmh, cfg, solverSettings)
%SIMULATE_CASE Integrate one road/speed case and construct signed outputs.

speed_mps = speedKmh / 3.6;
duration_s = cfg.track.length_m / speed_mps;
if isfield(solverSettings, 'outputStep_s')
    dt = solverSettings.outputStep_s;
else
    dt = cfg.solver.outputStep_s;
end
tOutput = (0:dt:duration_s).';
if tOutput(end) < duration_s
    tOutput = [tOutput; duration_s];
end

% Resolve the narrow pothole and the highest rough-road spatial harmonic.
obstacleStep = cfg.road.potholeLength_m / (40 * speed_mps);
roughStep = 1 / (20 * cfg.road.roughMaxFrequency_cpm * speed_mps);
% Output-sampling studies may change dt. Keep the integration settings
% independent by basing the normal solver limit on the configured 200 Hz
% baseline output interval, unless an explicit stricter MaxStep is supplied.
baselineMaxStep = min([cfg.solver.outputStep_s, obstacleStep, roughStep]);
if isfield(solverSettings, 'MaxStep')
    maxStep = min(baselineMaxStep, solverSettings.MaxStep);
else
    maxStep = baselineMaxStep;
end
options = odeset('RelTol', solverSettings.RelTol, ...
    'AbsTol', solverSettings.AbsTol, 'MaxStep', maxStep);

odeFunction = @(t, state) quarter_car_model(t, state, speed_mps, roadName, cfg);
[t_s, states] = ode45(odeFunction, tOutput, solverSettings.initialState, options);

x_m = speed_mps * t_s;
r_m = generate_road(x_m, roadName, cfg.road);
z_s = states(:, 1);
z_u = states(:, 2);
v_s = states(:, 3);
v_u = states(:, 4);
relativeDeflection = z_s - z_u;
relativeVelocity = v_s - v_u;
springForce = cfg.vehicle.ks_Npm * relativeDeflection;
damperForce = cfg.vehicle.cs_NsPm * relativeVelocity;
Fs = springForce + damperForce;
forceOnSprung = -Fs;
bodyAcceleration = forceOnSprung / cfg.vehicle.ms_kg;
dynamicTireNormalForce = cfg.vehicle.kt_Npm * (r_m - z_u);
staticNormalForce = (cfg.vehicle.ms_kg + cfg.vehicle.mu_kg) * cfg.vehicle.g_mps2;
estimatedTotalNormalForce = staticNormalForce + dynamicTireNormalForce;
contactLoss = estimatedTotalNormalForce <= 0;

data = table(t_s, x_m, r_m, z_s, z_u, v_s, v_u, ...
    relativeDeflection, relativeVelocity, springForce, damperForce, Fs, ...
    forceOnSprung, bodyAcceleration, dynamicTireNormalForce, ...
    estimatedTotalNormalForce, contactLoss, ...
    'VariableNames', {'Time_s', 'Distance_m', 'RoadElevation_m', ...
    'SprungDisplacement_m', 'UnsprungDisplacement_m', ...
    'SprungVelocity_mps', 'UnsprungVelocity_mps', ...
    'SuspensionDeflection_zs_minus_zu_m', ...
    'SuspensionRelativeVelocity_vs_minus_vu_mps', ...
    'SpringForce_ks_zs_minus_zu_N', ...
    'DamperForce_cs_vs_minus_vu_N', ...
    'DynamicSuspensionForce_Fs_N', ...
    'ForceOnSprungMass_minusFs_N', 'BodyAcceleration_mps2', ...
    'DynamicTireNormalForce_kt_r_minus_zu_N', ...
    'EstimatedTotalTireNormalForce_N', 'PredictedContactLoss'});

result.roadName = char(roadName);
result.speedKmh = speedKmh;
result.speed_mps = speed_mps;
result.duration_s = duration_s;
result.maxStep_s = maxStep;
result.data = data;
end
