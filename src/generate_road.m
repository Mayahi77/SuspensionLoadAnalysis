function r_m = generate_road(x_m, roadName, road)
%GENERATE_ROAD Evaluate a road elevation profile defined in distance.
%   Positive elevation is upward. Identical r(x) geometry is used at every
%   speed; simulate_case performs the conversion x = speed*t.

x = x_m(:);
switch lower(char(roadName))
    case 'smooth'
        r_m = zeros(size(x));

    case 'speed_bump'
        r_m = isolated_sine_squared(x, road.obstacleStart_m, ...
            road.bumpLength_m, road.bumpHeight_m);

    case 'pothole'
        r_m = isolated_sine_squared(x, road.obstacleStart_m, ...
            road.potholeLength_m, -road.potholeDepth_m);

    case 'rough'
        n = road.roughHarmonicCount;
        frequencies = logspace(log10(road.roughMinFrequency_cpm), ...
            log10(road.roughMaxFrequency_cpm), n);
        weights = (frequencies / road.roughMinFrequency_cpm).^(-0.80);
        amplitudes = road.roughNominalRMS_m * weights / sqrt(sum(weights.^2) / 2);
        stream = RandStream('mt19937ar', 'Seed', road.roughSeed);
        phases = 2*pi*rand(stream, 1, n);
        raw = zeros(size(x));
        for k = 1:n
            raw = raw + amplitudes(k) * sin(2*pi*frequencies(k)*x + phases(k));
        end
        r_m = raw .* rough_envelope(x, road);

    otherwise
        error('Phase1:UnknownRoad', 'Unknown road profile: %s', roadName);
end
end

function y = isolated_sine_squared(x, startPosition, length_m, amplitude_m)
y = zeros(size(x));
inside = x >= startPosition & x <= (startPosition + length_m);
phase = (x(inside) - startPosition) / length_m;
y(inside) = amplitude_m * sin(pi * phase).^2;
end

function envelope = rough_envelope(x, road)
% Raised-cosine ramps prevent artificial elevation steps at rough-road edges.
startPosition = road.roughStart_m;
ramp = road.roughRampLength_m;
endPosition = road.roughEnd_m;
envelope = zeros(size(x));

rampUp = x >= startPosition & x < startPosition + ramp;
u = (x(rampUp) - startPosition) / ramp;
envelope(rampUp) = 0.5 - 0.5*cos(pi*u);

steady = x >= startPosition + ramp & x <= endPosition - ramp;
envelope(steady) = 1;

rampDown = x > endPosition - ramp & x <= endPosition;
u = (endPosition - x(rampDown)) / ramp;
envelope(rampDown) = 0.5 - 0.5*cos(pi*u);
end
