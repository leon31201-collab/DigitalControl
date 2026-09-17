% Total transfer function motor voltage (V) -> motor velocity (rad/s),
% with the robot mass and a ~10ms measurement delay.
% ponytail: plain MATLAB (no Control System Toolbox). With the toolbox this
% is G = tf(num, den); bode(G*tf(pn,pd)); margin(...)

% Motor parameters: R L Kb J D (J = motor + gear + wheel, wheels off the floor)
load('motorParams.mat', 'R', 'L', 'Kb', 'J', 'D');
K = Kb;  Jm = J;

% Robot mass, seen from one motor shaft: half the mass on each wheel,
% moving at v = w*r/gear, so  1/2*(m/2)*v^2 = 1/2*Jr*w^2
m    = 0.95;     % kg
r    = 0.0315;   % wheel radius (m)
gear = 9.6;
Jr   = (m/2)*(r/gear)^2;
J    = Jm + Jr;
fprintf('J motor = %.3e, J robot = %.3e, J total = %.3e kg m^2\n', Jm, Jr, J);

% G(s) = K / ((L s + R)(J s + D) + K^2)
num = K;
den = conv([L R], [J D]) + [0 0 K^2];

% Measurement delay e^(-sT) is not rational -> 1st order Pade approximation
T  = 0.010;
pn = [-T/2 1];   pd = [T/2 1];                   % (1 - sT/2)/(1 + sT/2)

fprintf('G(s)    = %.4g / (%.3g s^2 + %.3g s + %.3g)\n', num, den);
fprintf('Gtot(s) = G(s) * (1 - %.3g s)/(1 + %.3g s)\n', T/2, T/2);

w     = logspace(0, 4, 4000);                    % rad/s
jw    = 1i*w;
G     = polyval(num, jw) ./ polyval(den, jw);
Gtot  = G .* polyval(pn, jw) ./ polyval(pd, jw);
Gexact = G .* exp(-jw*T);

ph    = @(x) unwrap(angle(x))*180/pi;
figure;
subplot(2,1,1);
semilogx(w, 20*log10(abs(Gtot)), w, 20*log10(abs(Gexact)), '--');
grid on;  ylabel('|G| (dB)');  legend('Pade delay', 'exact delay');
title('V_{motor} \rightarrow \omega_{motor}');
subplot(2,1,2);
semilogx(w, ph(Gtot), w, ph(Gexact), '--');  hold on;
yline(-180, 'r:');  ylim([-360 0]);  yticks(-360:90:0);
grid on;  ylabel('phase (deg)');  xlabel('\omega (rad/s)');

% Q1: PI-frequency (phase = -180 deg),  Q2: stability with Kp = 1
w180 = interp1(ph(Gtot), w, -180);
g180 = interp1(w, abs(Gtot), w180);
fprintf('PI-frequency w180 = %.0f rad/s (%.1f Hz)\n', w180, w180/(2*pi));
fprintf('|Gtot(w180)| = %.2f  ->  gain margin %.1f dB\n', g180, -20*log10(g180));
if g180 < 1
    fprintf('Kp = 1 is stable (Kp < %.2f)\n', 1/g180);
else
    fprintf('Kp = 1 is UNSTABLE (needs Kp < %.2f)\n', 1/g180);
end
subplot(2,1,1);  xline(w180, 'r:', 'HandleVisibility', 'off');
subplot(2,1,2);  xline(w180, 'r:', 'HandleVisibility', 'off');
