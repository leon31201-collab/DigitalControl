% Motor model parameters from the step log (steady state) and spike log (transient).
%   V   = R*i + L*di/dt + Kb*w + V0     (electrical)
%   Kt*i = J*dw/dt + D*w + Tc           (mechanical),  Kt = Kb
% V is the commanded voltage in the log. V0 is what the firmware's minV/loss
% compensation gets wrong, Tc is Coulomb friction. Both are constant offsets,
% not part of the linear model.
% At 0V the driver brakes (shorts the motor) and the current reading while
% braking is unreliable, so only driven samples are used.

stepFile  = 'logs/device-monitor-260917-114607.log';
spikeFile = 'logs/device-monitor-260917-120144.log';
motor = 1;              % 1 = left, 2 = right. Right current channel reads ~2x low
                        % on Kim (R = 9 ohm vs 3-6 ohm for the class), so use left
K     = 2*pi/48;        % rad per encoder count

%% ---- steady state from the steps: R, Kb, V0 and D, Tc ---------------
[t, volt, enc, vel, cur] = loadLog(stepFile, motor);
Vs = unique(volt(volt > 0));
ss = zeros(numel(Vs), 3);                          % [V i w]
for n = 1:numel(Vs)
    k = find(volt == Vs(n));
    k = k(t(k) > t(k(end)) - 0.2);                 % last 0.2s of the step
    ss(n,:) = [Vs(n), mean(cur(k)), mean(vel(k))];
end

% Stall current from the first spike: 6V, motor has not moved yet
[ts, vs, es, ~, cs] = loadLog(spikeFile, motor);
on  = find(diff(vs > 0) ==  1) + 1;
off = find(diff(vs > 0) == -1) + 1;
k = on(1)+5 : off(1)-1;                            % skip the 2.5ms L/R rise
k = k(es(k) == es(on(1)));
iStall = mean(cs(k));

% V = R*i + Kb*w + V0 on steps + stall
A  = [ss(:,2), ss(:,3), ones(numel(Vs),1);  iStall, 0, 1];
p  = A \ [ss(:,1); vs(on(1))];
R  = p(1);  Kb = p(2);  V0 = p(3);  Kt = Kb;

% Kt*i = D*w + Tc on the steps
p  = [ss(:,3), ones(numel(Vs),1)] \ (Kt*ss(:,2));
D  = p(1);  Tc = p(2);

%% ---- L from the current rise in each spike (lecture method) ---------
% First order: time to 63.2% of the peak current, L = tau_e*R. The current
% is sampled 0.5ms after the voltage is applied, so the first sample is t=0.5ms.
% (The current sensor's output filter, tau ~0.29ms, is included in tau_e.)
tauE = zeros(numel(on),1);
for n = 1:numel(on)
    c  = cs(on(n)+1 : off(n));  tc = (1:numel(c))' * mean(diff(ts));
    j  = find(c >= 0.632*max(c), 1);
    tauE(n) = interp1(c(j-1:j), tc(j-1:j), 0.632*max(c));
end
Ln = tauE * R;
L  = mean(Ln);

%% ---- J from the mechanical time constant of the 3V and 6V steps ------
% dw/dt = -(Kt*Kb/R + D)/J * w + ...  ->  J = tau*(Kt*Kb/R + D)
W = 10;                                            % +-5ms central difference
w = zeros(size(t));  k = W+1:numel(t)-W;
w(k) = (enc(k+W) - enc(k-W))*K / (2*W*mean(diff(t)));
tau = [];
for v = Vs(Vs >= 3)'
    k  = find(volt == v);
    w0 = mean(vel(k(1)-40:k(1)-1));
    w1 = ss(Vs == v, 3);
    tau(end+1) = t(k(find(w(k) > w0 + 0.632*(w1-w0), 1))) - t(k(1)); %#ok<SAGROW>
end
J = mean(tau) * (Kt*Kb/R + D);

%% ---- report + save ---------------------------------------------------
fprintf('\nsteady state:   V      i (A)   w (rad/s)\n');
fprintf('             %5.2f  %7.3f  %8.1f\n', ss');
fprintf('stall current %.3f A,  L per spike: %s mH,  tau_m per step: %s ms\n', ...
        iStall, num2str(Ln'*1e3, '%.2f  '), num2str(tau*1e3, '%.1f  '));
fprintf('\nR  = %6.2f  ohm\n', R);
fprintf('L  = %6.2f  mH   (tau_e = %.2f ms)\n', L*1e3, L/R*1e3);
fprintf('Kb = Kt = %.4f  V/(rad/s) = Nm/A\n', Kb);
fprintf('J  = %.3e  kg m^2   (tau_m = %.1f ms)\n', J, mean(tau)*1e3);
fprintf('D  = %.3e  Nm/(rad/s)   (back-EMF damping Kt*Kb/R = %.2e)\n', D, Kt*Kb/R);
fprintf('offsets: V0 = %.2f V, Tc = %.2e Nm\n', V0, Tc);
save('motorParams.mat', 'R', 'L', 'Kb', 'Kt', 'J', 'D', 'V0', 'Tc');

%% ---------------------------------------------------------------------
function [t, volt, enc, vel, cur] = loadLog(file, motor)
    dd = readmatrix(file, 'FileType', 'text', 'CommentStyle', '%');
    dd = dd(all(isfinite(dd), 2), :);
    dd = dd(1:find([diff(dd(:,1)); -1] < 0, 1), :);   % first log only
    t = dd(:,1);  volt = dd(:,3+motor);  enc = dd(:,5+motor);
    vel = dd(:,7+motor);  cur = dd(:,13+motor);
    idle = volt == 0 & t < t(find(volt ~= 0, 1)) - 0.005;
    cur = cur - mean(cur(idle));                   % offset is per robot
end
