% PI velocity controller for one motor (with robot mass and 10ms delay).
% C(s) = Kp * (tau_i s + 1) / (tau_i s)
transferFunction;                 % R L K J D T, num/den (G), pn/pd (Pade)
close all;

%% ---- design in the frequency domain ---------------------------------
PM = 60;         % wanted phase margin (deg)
Ni = 3;          % PI zero Ni times below the crossover frequency

% Phase lost to the PI zero at wc: atan(1/Ni). Find wc where
% angle(Gtot) = -180 + PM + atan(1/Ni).
phiPI = atand(1/Ni);
wc    = interp1(ph(Gtot), w, -180 + PM + phiPI);
tau_i = Ni / wc;
Gwc   = interp1(w, abs(Gtot), wc);
Kp    = 1 / (Gwc * sqrt(1 + 1/Ni^2));   % |C(jwc)*Gtot(jwc)| = 1
fprintf('\nwc = %.1f rad/s, tau_i = %.4f s, Kp = %.4f\n', wc, tau_i, Kp);

% Tuned in simulation (Kp/tau_i sweep). The plant is almost an integrator
% (mechanical pole ~1.7 rad/s), so the PI zero near wc gives ~25% overshoot,
% made worse by the 8V saturation. A higher Kp with a slow integrator
% (zero at 1 rad/s) settles in <0.1s with no overshoot; the integrator
% only has to remove the friction/voltage offsets.
KpDesign = Kp;  tau_iDesign = tau_i;
Kp    = 0.2;
tau_i = 1.0;
L_  = Kp*(1 + 1./(tau_i*jw)) .* Gtot;             % open loop with final PI
kc  = find(abs(L_) < 1, 1);
fprintf('final: crossover %.0f rad/s, phase margin %.0f deg, gain margin %.1f dB\n', ...
        w(kc), 180 + ph(L_(kc)), -20*log10(interp1(ph(L_), abs(L_), -180)));

%% ---- Simulink test ---------------------------------------------------
Vmax  = 8;       % V, motor voltage and integrator limit
wRef  = 100;     % rad/s step at 0.1s
model = 'motorPI';
buildModel(model);

out = sim(model, 'StopTime', '0.5');
y = out.yout{1}.Values;  u = out.yout{2}.Values;
[os, ts] = stepInfo(y, wRef);
fprintf('overshoot %.1f %%, settling time (2%%) %.0f ms\n', os, ts*1e3);

figure;
subplot(2,1,1);  plot(y.Time, y.Data);  grid on;  hold on;
yline(wRef, 'r:');  ylabel('\omega motor (rad/s)');
title(sprintf('PI: Kp = %.3f, \\tau_i = %.3f s', Kp, tau_i));
subplot(2,1,2);  plot(u.Time, u.Data);  grid on;
ylabel('motor voltage (V)');  xlabel('time (s)');

save('piDesign.mat', 'R', 'L', 'K', 'Jm', 'Jr', 'J', 'D', 'T', ...
     'num', 'den', 'pn', 'pd', 'PM', 'Ni', 'wc', 'KpDesign', 'tau_iDesign', 'Kp', 'tau_i', 'Vmax');

%% ---------------------------------------------------------------------
function [os, ts] = stepInfo(y, ref)
    t = y.Time - 0.1;  v = y.Data(:);
    os = max(0, (max(v) - ref) / ref * 100);
    outside = find(abs(v - ref) > 0.02*ref, 1, 'last');
    ts = t(min(outside + 1, numel(t)));
end

function buildModel(model)
    % ref -> PI (saturated integrator) -> saturation -> motor -> delay -> feedback
    if bdIsLoaded(model), close_system(model, 0); end
    new_system(model);
    set_param(model, 'SaveOutput', 'on', 'SaveFormat', 'Dataset', 'MaxStep', '1e-4');
    b = @(lib, name, varargin) add_block(lib, [model '/' name], varargin{:});
    b('simulink/Sources/Step', 'Step', 'Time', '0.1', 'Before', '0', 'After', 'wRef');
    b('simulink/Math Operations/Sum', 'Err', 'Inputs', '+-');
    b('simulink/Math Operations/Gain', 'Kp', 'Gain', 'Kp');
    b('simulink/Math Operations/Gain', 'Ki', 'Gain', 'Kp/tau_i');
    b('simulink/Continuous/Integrator', 'Integrator', 'LimitOutput', 'on', ...
      'UpperSaturationLimit', 'Vmax', 'LowerSaturationLimit', '-Vmax');
    b('simulink/Math Operations/Sum', 'PI', 'Inputs', '++');
    b('simulink/Discontinuities/Saturation', 'Saturation', ...
      'UpperLimit', 'Vmax', 'LowerLimit', '-Vmax');
    b('simulink/Continuous/Transfer Fcn', 'Motor', 'Numerator', 'num', 'Denominator', 'den');
    b('simulink/Continuous/Transport Delay', 'Delay', 'DelayTime', 'T');
    b('simulink/Sinks/Out1', 'omega');
    b('simulink/Sinks/Out1', 'voltage');
    c = @(src, dst) add_line(model, src, dst, 'autorouting', 'on');
    c('Step/1', 'Err/1');        c('Err/1', 'Kp/1');        c('Err/1', 'Ki/1');
    c('Kp/1', 'PI/1');           c('Ki/1', 'Integrator/1'); c('Integrator/1', 'PI/2');
    c('PI/1', 'Saturation/1');   c('Saturation/1', 'Motor/1');
    c('Motor/1', 'Delay/1');     c('Delay/1', 'Err/2');
    c('Motor/1', 'omega/1');     c('Saturation/1', 'voltage/1');
    Simulink.BlockDiagram.arrangeSystem(model);
    save_system(model);
end
