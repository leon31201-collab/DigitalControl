% Motor model parameters from the spikeSequence log.
%   V   = R*i + L*di/dt + Kb*w        (electrical)
%   Kt*i = J*dw/dt + D*w              (mechanical),  Kt = Kb
% Both are integrated over time before fitting, so nothing is differentiated
% and the fits are not dominated by encoder quantisation noise.

file  = 'logs/device-monitor-260909-111426.log';
motor = 2;              % 1 = left, 2 = right
PPR   = 48;             % encoder counts per motor revolution
W     = 20;             % +/- samples for the central-difference velocity

dd   = readmatrix(file);
t    = dd(:,1);   ts = mean(diff(t));
volt = dd(:,3+motor);
enc  = dd(:,5+motor);
cur  = dd(:,13+motor);
K    = 2*pi/PPR;                                    % rad per encoder count

on  = find(diff(volt > 0) ==  1) + 1;               % spike start samples
off = find(diff(volt > 0) == -1) + 1;               % spike end samples

% The firmware current offset is hard-coded for another robot - re-zero on
% the idle samples before the first spike.
cur = cur - mean(cur(1:on(1)-10));

% Shaft speed, central difference over +-W samples. One count is K/ts ~ 260
% rad/s at 0.5ms, so a narrower window is pure quantisation noise.
w = zeros(size(t));
k = W+1 : numel(t)-W;
w(k) = (enc(k+W) - enc(k-W)) * K / (2*W*ts);

%% ---- electrical: R, L, Kb, one fit per spike -------------------------
% Integrating  V = R*i + L*di/dt + Kb*w  gives
%   V*(t-t0) = R*int(i dt) + L*(i-i0) + Kb*(theta-theta0)
P = zeros(numel(on),3);
for n = 1:numel(on)
    s  = on(n):off(n)-1;
    Ii = cumtrapz(t(s), cur(s));
    A  = [Ii, cur(s)-cur(s(1)), (enc(s)-enc(s(1)))*K];
    P(n,:) = (A \ (volt(s).*(t(s)-t(s(1)))))';
end
R = mean(P(:,1));  L = mean(P(:,2));  Kb = mean(P(:,3));  Kt = Kb;

%% ---- mechanical: D/J from the coast-down ----------------------------
% At 0V the bridge coasts (current stays at ~0 while the motor still spins),
% so  J*dw/dt = -D*w  and  log(w) is a straight line with slope -D/J.
DJ = zeros(numel(off),1);
for n = 1:numel(off)
    s = off(n)+W+2 : min(off(n)+560, numel(t)-W);
    s = s(w(s) > 8);                                % before it hits the noise floor
    p = polyfit(t(s), log(w(s)), 1);
    DJ(n) = -p(1);
end
DoverJ = mean(DJ);

%% ---- J from the momentum gained during each spike --------------------
% Integrating  Kt*i = J*dw/dt + D*w  over the spike:
%   Kt*int(i dt) = J*w(t_off) + D*theta = J*(w(t_off) + (D/J)*theta)
Jn = zeros(numel(on),1);
for n = 1:numel(on)
    s  = on(n):off(n)-1;
    Ii = trapz(t(s), cur(s));
    k0 = off(n)+W+2;
    wOff  = w(k0) * exp(DoverJ*(t(k0)-t(off(n))));  % coast back to the spike end
    theta = (enc(off(n)-1) - enc(on(n)))*K;
    Jn(n) = Kt*Ii / (wOff + DoverJ*theta);
end
J = mean(Jn);  D = DoverJ*J;

%% ---- report ---------------------------------------------------------
fprintf('\nper spike:  R          L          Kb         D/J        J\n');
for n = 1:numel(on)
    fprintf('  %d      %7.3f  %9.2e  %9.2e  %7.2f  %9.3e\n', ...
            n, P(n,1), P(n,2), P(n,3), DJ(n), Jn(n));
end
fprintf('\nR  = %6.2f  ohm\n',            R);
fprintf('L  = %6.2f  mH   (tau_e = %.2f ms)\n', L*1e3, L/R*1e3);
fprintf('Kb = Kt = %.4f  V/(rad/s) = Nm/A\n',   Kb);
fprintf('J  = %.3e  kg m^2\n',            J);
fprintf('D  = %.3e  Nm/(rad/s)  (tau_m = J/D = %.1f ms)\n', D, 1e3/DoverJ);

% Consistency check: if the bridge were braking instead of coasting, the
% decay would be at least Kt*Kb/(R*J); it is not, so coasting is right.
fprintf('\ncoast check: Kt*Kb/(R*J) = %.0f 1/s vs measured decay %.0f 1/s\n', ...
        Kt*Kb/(R*J), DoverJ);
