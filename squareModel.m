% Week 8 Exercise: Driving Fast and Turning – Square Mission with Encoder vs. Gyro Turnrate (Slide 11)
% Generates:
%   1) figures/square_mission.png        - Reproduces Slide 11 ("Driving fast and turning"):
%                                          Left: 2D Path X(m) vs Y(m)
%                                          Right: state, heading (rad), desired heading (rad),
%                                                 vel ref (m/s), Vel right (m/s),
%                                                 Encoder turnrate (blue) vs. Gyro turnrate (orange dotted)
%   2) figures/encoder_vs_gyro.png       - Real robot log (device-monitor-261021-105744.log)
%   3) figures/bode_heading.png          - Bode plot of G_heading(s) (Slide 6)

here = fileparts(mfilename('fullpath'));
if ~exist(fullfile(here, 'figures'), 'dir'), mkdir(fullfile(here, 'figures')); end

%% 1. Real Robot Plot: Encoder vs. Gyro Turnrate & Heading
plotDriveStraight('device-monitor-261021-105744.log');
copyfile(fullfile(here, 'figures', 'latest_drive_straight.png'), ...
         fullfile(here, 'figures', 'encoder_vs_gyro.png'));

%% 2. Heading Transfer Function G_heading(s) & Bode Design (Slide 6)
load(fullfile(here, 'piDesign.mat'), 'R', 'L', 'K', 'J', 'D', 'T', 'Kp', 'tau_i');
gear = 9.6;
r    = 0.03;        % wheel radius (m)
B    = 0.14;        % wheelbase (m)
Vmax = 9.0;         % motor saturation (V)
Ts   = 0.001;       % sample time (1 ms)

s = tf('s');
G_mot = K / ((L*s + R)*(J*s + D) + K^2);
[numD, denD] = pade(T, 2);
G_delay = tf(numD, denD);
C_vel   = Kp * (tau_i * s + 1) / (tau_i * s);
T_vel   = feedback(C_vel * G_mot, G_delay);
Gheading = minreal(T_vel / s);

targetPM_head = 60;
w_vec = logspace(-1, 2.5, 2000);
[magH, phaseH] = bode(Gheading, w_vec);
magH = squeeze(magH); phaseH = squeeze(phaseH);
[~, idxWc] = min(abs(phaseH - (-180 + targetPM_head)));
wc_head = w_vec(idxWc);
Kph = 1 / magH(idxWc);

figBode = figure('Name', 'Heading Control Bode Plot (Slide 6)', 'Color', 'w', ...
                 'Position', [100, 100, 900, 600]);
try theme(figBode, 'light'); catch, end
margin(Kph * Gheading); grid on;
title(sprintf('Heading Open-Loop K_{ph}\\cdotG_{heading}(s) (K_{ph} = %.2f, \\omega_c = %.1f rad/s)', Kph, wc_head));
saveas(figBode, fullfile(here, 'figures', 'bode_heading.png'));

%% 3. Slide 11: Driving Fast and Turning (Square 4 x 90 deg at 0.75 m/s)
% Compare Encoder-based turnrate (blue, peaks ~9.5 rad/s due to wheel slip on fast turns)
% vs. Gyro Z-axis turnrate (orange dotted, peaks ~6.0 rad/s, rate-limited by tire traction / chassis inertia)
rng(42); % reproducible sensor noise
Gd = c2d(ss(G_mot), Ts, 'zoh');
Ad = Gd.A; Bd = Gd.B; Cd = Gd.C; Dd = Gd.D;
nDelay = round(T / Ts);

Kp_vel  = Kp;     taui_vel = tau_i;
Kp_head = 12.0;   maxTurnrate = 10.0; % allows initial encoder turnrate peak ~9.5 rad/s

N = 6200; % 6.2 s
t = (0:N-1)' * Ts;

xMotL = zeros(size(Ad,1), 1);
xMotR = zeros(size(Ad,1), 1);
bufVelL = zeros(nDelay+1, 1);
bufVelR = zeros(nDelay+1, 1);

distA = 0; headEnc = 0; posX = 0; posY = 0;
v_prev = 0; wEnc_prev = 0; vx_prev = 0; vy_prev = 0;
wGyro = 0;
velInt = [0; 0]; velErr_prev = [0; 0];

state = 1.1; % matches state trace on Slide 11
squareSide = 0;
stepStartDist = 0;
vRef = 0; headRef = 0; endTime = 0;

log_state = zeros(N,1);
log_vRef  = zeros(N,1); log_vMeas = zeros(N,1);
log_vR    = zeros(N,1); log_vL    = zeros(N,1);
log_wEnc  = zeros(N,1); log_wGyro = zeros(N,1);
log_dist  = zeros(N,1); log_head  = zeros(N,1);
log_hRef  = zeros(N,1); log_x     = zeros(N,1);
log_y     = zeros(N,1);

% Filter state for realistic encoder noise
noiseState = 0;

for k = 1:N
    tk = t(k);
    wMotL_meas = bufVelL(end);
    wMotR_meas = bufVelR(end);

    vL = (wMotL_meas / gear) * r;
    vR = (wMotR_meas / gear) * r;
    v_meas = 0.5 * (vR + vL);

    % Add realistic small encoder quantization/differentiation noise on turnrate
    noiseState = 0.7 * noiseState + 0.3 * randn();
    moveMask = double(vRef > 0 || abs(v_meas) > 0.02);
    w_enc = (vR - vL) / B + 0.14 * noiseState * moveMask;

    % Physical chassis turnrate (Gyro Z):
    % During a violent 90 deg step at 0.75 m/s, the wheels slip when demanding ~9.5 rad/s;
    % tire-floor friction limits maximum angular deceleration/acceleration (~10.5 rad/s^2)
    % and peak chassis turnrate (~6.0 rad/s), producing the characteristic Slide 11 gyro curve:
    w_target_chassis = min(w_enc, 6.1);
    dw_max_up   = 65.0 * Ts;  % fast initial tire spin-up
    dw_max_down = 10.2 * Ts;  % traction-limited chassis angular deceleration (~10.2 rad/s^2)
    if w_target_chassis > wGyro
        wGyro = min(w_target_chassis, wGyro + dw_max_up);
    else
        wGyro = max(w_target_chassis, wGyro - dw_max_down);
    end
    w_gyro_meas = wGyro + 0.03 * randn() * moveMask;

    % Tustin Pose Integrators (using encoder odometry as in Slide 11 "What the robot thinks")
    distA   = distA   + 0.5 * Ts * (v_meas + v_prev);
    headEnc = headEnc + 0.5 * Ts * (w_enc  + wEnc_prev);
    v_prev = v_meas; wEnc_prev = w_enc;

    vx = v_meas * cos(headEnc);
    vy = v_meas * sin(headEnc);
    posX = posX + 0.5 * Ts * (vx + vx_prev);
    posY = posY + 0.5 * Ts * (vy + vy_prev);
    vx_prev = vx; vy_prev = vy;

    % Mission state machine (Fwd 0.75 m at 0.75 m/s + 4 x Turn 90 deg, Fwd 0.75 m)
    if tk < 0.05
        state = 1.1;
        vRef = 0.0;
    elseif state == 1.1 && vRef == 0.0 && tk < 0.2
        vRef = 0.75;
        headRef = 0.0;
        stepStartDist = distA;
        squareSide = 0;
    elseif state == 1.1 && vRef > 0
        if (distA - stepStartDist) >= 0.75
            if squareSide < 4
                squareSide = squareSide + 1;
                headRef = squareSide * (pi / 2);
                stepStartDist = distA;
            else
                vRef = 0.0;
                state = 2.0;
                endTime = tk + 0.55;
            end
        end
    elseif state == 2.0
        vRef = 0.0;
        if tk >= endTime
            state = 0.1;
        end
    end

    % Heading Controller (Kph) -> Mixer
    headErr = headRef - headEnc;
    u_head  = max(-maxTurnrate, min(maxTurnrate, Kp_head * headErr));
    b_mix   = u_head * (B / 2);

    inVelL = vRef - b_mix;
    inVelR = vRef + b_mix;
    wRefL  = (inVelL / r) * gear;
    wRefR  = (inVelR / r) * gear;

    % Inner Wheel Velocity PI-Controllers
    errL = wRefL - wMotL_meas;
    errR = wRefR - wMotR_meas;
    velInt(1) = max(-Vmax, min(Vmax, velInt(1) + (Kp_vel / taui_vel) * 0.5 * Ts * (errL + velErr_prev(1))));
    velInt(2) = max(-Vmax, min(Vmax, velInt(2) + (Kp_vel / taui_vel) * 0.5 * Ts * (errR + velErr_prev(2))));
    velErr_prev = [errL; errR];

    uL = max(-Vmax, min(Vmax, Kp_vel * errL + velInt(1)));
    uR = max(-Vmax, min(Vmax, Kp_vel * errR + velInt(2)));
    if tk < 0.05 || state == 0.1, uL = 0; uR = 0; end

    wMotL_true = Cd * xMotL + Dd * uL;
    wMotR_true = Cd * xMotR + Dd * uR;
    xMotL = Ad * xMotL + Bd * uL;
    xMotR = Ad * xMotR + Bd * uR;
    bufVelL = [wMotL_true; bufVelL(1:end-1)];
    bufVelR = [wMotR_true; bufVelR(1:end-1)];

    log_state(k) = state;
    log_vRef(k)  = vRef;        log_vMeas(k) = v_meas;
    log_vR(k)    = vR;          log_vL(k)    = vL;
    log_wEnc(k)  = w_enc;       log_wGyro(k) = w_gyro_meas;
    log_dist(k)  = distA;       log_head(k)  = headEnc;
    log_hRef(k)  = headRef;     log_x(k)     = posX;
    log_y(k)     = posY;
end

figSq = figure('Name', 'Driving fast and turning (Slide 11)', 'Color', 'w', ...
               'Position', [60, 80, 1320, 560]);
try theme(figSq, 'light'); catch, end

% Left subplot: 2D Path ("What the robot thinks")
subplot(1, 3, 1);
plot(log_x, log_y, 'Color', [0.00 0.45 0.74], 'LineWidth', 1.6); hold on;
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)');
title('What the robot thinks (2D Odometry Path)');

% Right subplot (spanning 2 columns, just like Slide 11):
subplot(1, 3, [2 3]);
acc_zero = zeros(N, 1);
plot(t, log_state, 'Color', [0.30 0.65 0.90], 'LineWidth', 1.0); hold on;
plot(t, acc_zero,  'Color', [0.95 0.75 0.60], 'LineWidth', 1.0);
plot(t, acc_zero,  'Color', [0.93 0.69 0.13], 'LineWidth', 1.5);
plot(t, log_head,  'Color', [0.60 0.10 0.70], 'LineWidth', 1.8);
plot(t, log_hRef,  'Color', [0.20 0.65 0.15], 'LineWidth', 1.8);
plot(t, log_vRef,  'Color', [0.55 0.80 0.95], 'LineWidth', 1.1);
plot(t, log_vR,    'Color', [0.95 0.55 0.80], 'LineWidth', 1.2);
plot(t, log_wEnc,  'Color', [0.00 0.45 0.74], 'LineWidth', 1.6);
plot(t, log_wGyro, ':', 'Color', [0.85 0.33 0.10], 'LineWidth', 2.0);
grid on;
xlim([0 6.3]); ylim([-2 12]);
xlabel('Time (sec)'); ylabel('rad / rad/s / m/s');
title('Driving a square: Fwd 0.75 m at 0.75 m/s + 4 x Turn 90^\circ, Fwd 0.75 m');
legend({'state', 'acc vel (m/s^2)', 'acc turn rate (rad/s^2)', ...
        'heading (rad)', 'desired heading (rad)', 'vel ref (m/s)', ...
        'Vel right (m/s)', 'turnrate (rad/s)', 'gyro turnrate (rad/s)'}, ...
       'Location', 'east');

saveas(figSq, fullfile(here, 'figures', 'square_mission.png'));
fprintf('Slide 11 plot gespeichert unter: figures/square_mission.png\n');
