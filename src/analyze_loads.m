function summary = analyze_loads(result)
%ANALYZE_LOADS Calculate metrics over the complete 500 m track.
% RMS is the continuous-time approximation sqrt(trapz(t,y^2)/duration).

d = result.data;
t = d.Time_s;
duration = t(end) - t(1);
Fs = d.DynamicSuspensionForce_Fs_N;
deflection = d.SuspensionDeflection_zs_minus_zu_m;
acceleration = d.BodyAcceleration_mps2;
normalForce = d.EstimatedTotalTireNormalForce_N;

Road = string(result.roadName);
Speed_kmh = result.speedKmh;
TrackLength_m = d.Distance_m(end);
Duration_s = duration;
PeakAbsSuspensionForce_N = max(abs(Fs));
FullTrackRMSSuspensionForce_N = sqrt(trapz(t, Fs.^2) / duration);
PeakAbsSuspensionDeflection_m = max(abs(deflection));
FullTrackRMSBodyAcceleration_mps2 = sqrt(trapz(t, acceleration.^2) / duration);
MinimumEstimatedTotalTireNormalForce_N = min(normalForce);
PredictedContactLoss = any(d.PredictedContactLoss);
SolverMaxStep_s = result.maxStep_s;

summary = table(Road, Speed_kmh, TrackLength_m, Duration_s, ...
    PeakAbsSuspensionForce_N, FullTrackRMSSuspensionForce_N, ...
    PeakAbsSuspensionDeflection_m, FullTrackRMSBodyAcceleration_mps2, ...
    MinimumEstimatedTotalTireNormalForce_N, PredictedContactLoss, SolverMaxStep_s);
end
