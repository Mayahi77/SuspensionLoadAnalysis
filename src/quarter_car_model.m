function stateDerivative = quarter_car_model(t_s, state, speed_mps, roadName, cfg)
%QUARTER_CAR_MODEL Passive 2-DOF model about static equilibrium.
% State = [z_s; z_u; v_s; v_u], with upward-positive coordinates.
% Fs = ks*(z_s-z_u) + cs*(v_s-v_u). Force on sprung mass is -Fs.

z_s = state(1);
z_u = state(2);
v_s = state(3);
v_u = state(4);
r = generate_road(speed_mps * t_s, roadName, cfg.road);

Fs = cfg.vehicle.ks_Npm * (z_s - z_u) + ...
    cfg.vehicle.cs_NsPm * (v_s - v_u);
dynamicTireNormalForce = cfg.vehicle.kt_Npm * (r - z_u);

a_s = -Fs / cfg.vehicle.ms_kg;
a_u = (Fs + dynamicTireNormalForce) / cfg.vehicle.mu_kg;
stateDerivative = [v_s; v_u; a_s; a_u];
end
