% Final Project Simulation: "Keyhole" Route (Lecture 10 Slide 3 & Slide 5)
% Route Geometry:
%   1) Straight 1.0 m (from Start (0,0) to P1 = (1.0, 0.0), heading = 0 rad)
%   2) Corner 1 at P1: Turn right by 72.54 deg (-1.2661 rad) to align with circle tangent
%   3) Circle of diameter 1.0 m (R = 0.5 m, entry/exit gap = 0.3 m):
%      Arc angle = 360 - 2*asin(0.15/0.5) = 325.08 deg (5.6738 rad)
%      Arc length = R * 5.6738 = 2.8369 m (turning left from -1.2661 rad to +4.4077 rad)
%   4) Corner 2 at P2 = (1.0, 0.3): Turn right by 72.54 deg (-1.2661 rad) to heading = pi (180 deg)
%   5) Straight 1.5 m (from P2 = (1.0, 0.3) to Stop (-0.5, 0.3), heading = pi rad)
%
% Evaluates Lecture 10 Slide 5 Performance Metric:
%   P_i = t_i + d_{i,max}   (t_i in seconds, d_{i,max} in centimeters)

here = fileparts(mfilename('fullpath'));
if ~exist(fullfile(here, 'figures'), 'dir'), mkdir(fullfile(here, 'figures')); end

%% 1. Load Basebot Motor & Controller Parameters
load(fullfile(here, 'piDesign.mat'), 'R', 'L', 'K', 'J', 'D', 'T');
gear = 9.6;
r    = 0.0315;      % Wheel radius (m), wheelRadius in basebot_6.ino
B    = 0.15;        % Wheelbase (m), wheelBase in basebot_6.ino
Vmax = 9.0;         % Motor voltage saturation (V)
Ts   = 0.001;       % Sample period (1 ms)

s = tf('s');
G_mot = K / ((L*s + R)*(J*s + D) + K^2);
Gd = c2d(ss(G_mot), Ts, 'zoh');
Ad = Gd.A; Bd = Gd.B; Cd = Gd.C; Dd = Gd.D;
nDelay = max(1, round(T / Ts));

% Controller parameters = basebot_6.ino (piDesign.mat holds the earlier
% Kp = 0.13, tau_i = 0.5 design, which the firmware no longer uses)
Kp_vel      = 0.02;
taui_vel    = 0.04;
Kd_turn     = 0.22; % Kd during point turns (states 12, 14)
minTurn     = 0.60; % rad/s, minimum turnrate in point turns while |e| > 0.04 rad
Kp_head     = 3.5;
Kd_head     = 0.55;
maxTurnrate = 3.2;

% Week 9 Acceleration Limiters (Slide 10 & 11)
Kp_acc      = 25.0;
maxAccVel   = 2.0;  % m/s^2
maxAccTurn  = 1.0;  % m/s^2 (at wheel, after B/2)

%% 2. Exact Geometry of the Black Reference Track (Lecture 10 Slide 3)
L1      = 1.00;                         % First straight (m)
L2      = 1.50;                         % Second straight (m)
R_circ  = 0.50;                         % Circle radius (m), diameter = 1.0 m
d_gap   = 0.30;                         % Gap between entry and exit lines (m)
alpha   = asin((d_gap / 2) / R_circ);   % ~0.30469 rad (17.46 deg)
h_tan1  = -(pi/2 - alpha);              % -1.26610 rad (-72.54 deg)
dTheta  = 2*pi - 2*alpha;               % 5.67380 rad (325.08 deg)
L_arc   = R_circ * dTheta;              % 2.83690 m
h_tan2  = h_tan1 + dTheta;              % +4.40770 rad (+252.54 deg)
h_exit  = pi;                           % +3.14159 rad (+180.00 deg)

% Circle center in robot coordinates (x forward, y left)
xC = L1 + sqrt(R_circ^2 - (d_gap/2)^2); % 1.47697 m
yC = d_gap / 2;                         % 0.15000 m

% Build dense reference point cloud of the black line for plotting & d_{i,max} calculation
s1_vec = linspace(0, L1, 500)';
x_ref1 = s1_vec;
y_ref1 = zeros(size(s1_vec));

phi_vec = linspace(-pi + alpha, pi - alpha, 1500)';
x_ref2  = xC + R_circ * cos(phi_vec);
y_ref2  = yC + R_circ * sin(phi_vec);

s3_vec = linspace(0, L2, 750)';
x_ref3 = L1 - s3_vec;
y_ref3 = d_gap * ones(size(s3_vec));

x_track = [x_ref1; x_ref2; x_ref3];
y_track = [y_ref1; y_ref2; y_ref3];

%% 3. Closed-Loop Simulation of SequenceFinalProject()
vStraight = 1.00;   % fp_vStraight (m/s)
vCircle   = 0.75;   % fp_vCircle (m/s) -> v^2/R = 1.1 m/s^2

% Braking: v_ref = min(v, sqrt(2*aBrake*remaining)) -> position feedback, stops on the point
% regardless of speed and lag (fp_aBrake). Below maxAccVel so the limiter is not saturated.
aBrake = 1.5;       % m/s^2 (fp_aBrake)
brakeLead = 0.013;  % m (fp_brakeLead): real-robot overshoot from speed-loop lag, measured on Kaya

N = 12000; % 12.0 seconds simulation window
t = (0:N-1)' * Ts;

xMotL = zeros(size(Ad,1), 1);
xMotR = zeros(size(Ad,1), 1);
bufVelL = zeros(nDelay+1, 1);
bufVelR = zeros(nDelay+1, 1);

distA = 0; headGyro = 0; posX = 0; posY = 0;
v_prev = 0; w_prev = 0; vx_prev = 0; vy_prev = 0;
velInt = [0; 0]; velErr_prev = [0; 0];
velLim = 0; accVel = 0; accVel_prev = 0;
turnLim = 0; accTurn = 0; accTurn_prev = 0;

state = 10;
stepStartDist = 0;
vRef = 0; headRef = 0;
tFinish = NaN;

log_state   = zeros(N,1);
log_vRef    = zeros(N,1); log_vLim    = zeros(N,1); log_vMeas = zeros(N,1);
log_hRef    = zeros(N,1); log_head    = zeros(N,1); log_wMeas = zeros(N,1);
log_accVel  = zeros(N,1); log_accTurn = zeros(N,1);
log_uL      = zeros(N,1); log_uR      = zeros(N,1);
log_x       = zeros(N,1); log_y       = zeros(N,1);
log_dist    = zeros(N,1);

for k = 1:N
    tk = t(k);
    wMotL_meas = bufVelL(end);
    wMotR_meas = bufVelR(end);

    vL = (wMotL_meas / gear) * r;
    vR = (wMotR_meas / gear) * r;
    v_meas = 0.5 * (vR + vL);
    w_meas = (vR - vL) / B;

    % Tustin integration of Pose (updatePose)
    distA    = distA    + 0.5 * Ts * (v_meas + v_prev);
    headGyro = headGyro + 0.5 * Ts * (w_meas + w_prev);
    v_prev   = v_meas;
    w_prev   = w_meas;

    vx = v_meas * cos(headGyro);
    vy = v_meas * sin(headGyro);
    posX = posX + 0.5 * Ts * (vx + vx_prev);
    posY = posY + 0.5 * Ts * (vy + vy_prev);
    vx_prev = vx; vy_prev = vy;

    % -------------------------------------------------------------------
    % Mission State Machine: SequenceFinalProject()
    % -------------------------------------------------------------------
    w_ff = 0.0; % Curvature feedforward turnrate (rad/s)
    switch state
        case 10 % Standstill & Gyro Calibration (0.25 s)
            vRef = 0.0;
            headRef = 0.0;
            if tk > 0.25
                state = 11;
                stepStartDist = distA;
            end

        case 11 % Segment 1: Straight 1.0 m (h_ref = 0 rad)
            s_seg = distA - stepStartDist;
            headRef = 0.0;
            rem  = L1 - s_seg - brakeLead;
            vRef = min(vStraight, sqrt(2 * aBrake * max(0, rem)));
            if rem <= 0 || (rem < 0.01 && abs(v_meas) < 0.04)
                state = 12;
                vRef = 0.0;
                headRef = h_tan1; % -72.54 deg (-1.2661 rad)
            end

        case 12 % Corner 1: Point turn at P1 to circle tangent (-1.2661 rad)
            vRef = 0.0;
            headRef = h_tan1;
            if abs(headRef - headGyro) < 0.025 && abs(w_meas) < 0.25
                state = 13;
                stepStartDist = distA;
            end

        case 13 % Segment 2: Circle (R = 0.5 m, L_arc = 2.837 m)
            s_seg = distA - stepStartDist;
            s_seg   = max(0, s_seg);
            headRef = min(h_tan2, h_tan1 + (s_seg / R_circ));
            w_ff    = max(0, v_meas) / R_circ; % Curvature feedforward w = v_meas / R, as in firmware
            rem  = L_arc - s_seg - brakeLead;
            vRef = min(vCircle, sqrt(2 * aBrake * max(0, rem)));
            if rem <= 0 || (rem < 0.01 && abs(v_meas) < 0.04)
                state = 14;
                vRef = 0.0;
                headRef = h_exit; % +180 deg (pi rad)
                w_ff = 0.0;
            end

        case 14 % Corner 2: Point turn at P2 to exit straight (+pi rad)
            vRef = 0.0;
            headRef = h_exit;
            if abs(headRef - headGyro) < 0.025 && abs(w_meas) < 0.25
                state = 15;
                stepStartDist = distA;
            end

        case 15 % Segment 3: Straight 1.5 m to Stop (h_ref = pi rad)
            s_seg = distA - stepStartDist;
            headRef = h_exit;
            rem  = L2 - s_seg - brakeLead;
            vRef = min(vStraight, sqrt(2 * aBrake * max(0, rem)));
            if rem <= 0 || (rem < 0.01 && abs(v_meas) < 0.03)
                state = 2;
                vRef = 0.0;
                tFinish = tk - 0.25; % Completion time from start of motion (tk = 0.25 s)
            end

        case 2 % Finished
            vRef = 0.0;
            headRef = h_exit;
    end

    % -------------------------------------------------------------------
    % Outer Loop: Heading Controller + Curvature Feedforward
    % -------------------------------------------------------------------
    e_head  = headRef - headGyro;
    isPointTurn = (state == 12 || state == 14);
    kd = Kd_head;
    if isPointTurn, kd = Kd_turn; end
    turnRef = w_ff + Kp_head * e_head - kd * (w_meas - w_ff);
    if isPointTurn && abs(e_head) > 0.04 && abs(turnRef) < minTurn
        turnRef = sign(e_head) * minTurn;
    end
    turnRef = max(-maxTurnrate, min(maxTurnrate, turnRef));
    headU   = turnRef * (B * 0.5);

    % -------------------------------------------------------------------
    % Week 9 Acceleration Limiters (acc_limit_vel & acc_limit_turn)
    % -------------------------------------------------------------------
    accVel = max(-maxAccVel, min(maxAccVel, Kp_acc * (vRef - velLim)));
    velLim = velLim + 0.5 * Ts * (accVel + accVel_prev);
    accVel_prev = accVel;

    accTurn = max(-maxAccTurn, min(maxAccTurn, Kp_acc * (headU - turnLim)));
    turnLim = turnLim + 0.5 * Ts * (accTurn + accTurn_prev);
    accTurn_prev = accTurn;

    % Mixer
    inVelL = velLim - turnLim;
    inVelR = velLim + turnLim;
    wRefL  = (inVelL / r) * gear;
    wRefR  = (inVelR / r) * gear;

    % -------------------------------------------------------------------
    % Inner Loop: Wheel Velocity PI Controllers with Anti-Windup
    % -------------------------------------------------------------------
    errL = wRefL - wMotL_meas;
    errR = wRefR - wMotR_meas;
    velInt(1) = max(-Vmax, min(Vmax, velInt(1) + (Kp_vel / taui_vel) * 0.5 * Ts * (errL + velErr_prev(1))));
    velInt(2) = max(-Vmax, min(Vmax, velInt(2) + (Kp_vel / taui_vel) * 0.5 * Ts * (errR + velErr_prev(2))));
    velErr_prev = [errL; errR];

    uL = max(-Vmax, min(Vmax, Kp_vel * errL + velInt(1)));
    uR = max(-Vmax, min(Vmax, Kp_vel * errR + velInt(2)));
    if state == 10   % firmware keeps controlUpdate() running in state 2 (active braking)
        uL = 0; uR = 0;
    end

    wMotL_true = Cd * xMotL + Dd * uL;
    wMotR_true = Cd * xMotR + Dd * uR;
    xMotL = Ad * xMotL + Bd * uL;
    xMotR = Ad * xMotR + Bd * uR;
    bufVelL = [wMotL_true; bufVelL(1:end-1)];
    bufVelR = [wMotR_true; bufVelR(1:end-1)];

    log_state(k)   = state;
    log_vRef(k)    = vRef;      log_vLim(k)    = velLim;  log_vMeas(k) = v_meas;
    log_hRef(k)    = headRef;   log_head(k)    = headGyro; log_wMeas(k) = w_meas;
    log_accVel(k)  = accVel;    log_accTurn(k) = accTurn;
    log_uL(k)      = uL;        log_uR(k)      = uR;
    log_x(k)       = posX;      log_y(k)       = posY;
    log_dist(k)    = distA;
end

%% 4. Compute Lecture 10 Slide 5 Assessment Score: P_i = t_i + d_{i,max}
dev_cm = zeros(N, 1);
for k = 1:N
    dx = x_track - log_x(k);
    dy = y_track - log_y(k);
    dev_cm(k) = min(sqrt(dx.^2 + dy.^2)) * 100; % Convert meters to centimeters (cm)
end
d_max_cm = max(dev_cm);
if isnan(tFinish), tFinish = t(end); end
P_score = tFinish + d_max_cm;

fprintf('\n======================================================\n');
fprintf('  FINAL PROJECT SIMULATION RESULTS (Lecture 10 Slide 5)\n');
fprintf('------------------------------------------------------\n');
fprintf('  Total Driven Distance : %.3f m (Target: %.3f m)\n', distA, L1 + L_arc + L2);
fprintf('  Completion Time t_i   : %.2f s\n', tFinish);
fprintf('  Max Deviation d_max   : %.2f cm\n', d_max_cm);
fprintf('  Performance Score P_i : %.2f (t_i + d_max)\n', P_score);
fprintf('======================================================\n');

%% 5. Plot Final Project Trajectory & Signals
figFP = figure('Name', 'Final Project Simulation (Lecture 10)', 'Color', 'w', ...
               'Position', [50, 50, 1320, 760]);
try theme(figFP, 'light'); catch, end

% Left Panel (spanning 2 rows): 2D Keyhole Route (plotted in Slide 3 orientation: -Y horizontal, X vertical)
subplot(2, 3, [1 4]);
plot(-y_track, x_track, 'k-', 'LineWidth', 5.0); hold on;
plot(-log_y, log_x, 'r--', 'LineWidth', 2.2);
plot(0, 0, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(-log_y(end), log_x(end), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
grid on; axis equal;
xlim([-0.85, 0.55]); ylim([-0.65, 2.15]);
xlabel('Lateral Position -y (m)'); ylabel('Forward Position x (m)');
title(sprintf('Final Project Route (t_i = %.2f s, d_{max} = %.2f cm, P_i = %.2f)', ...
              tFinish, d_max_cm, P_score));
legend({'Black Marking (Target)', 'Robot Trajectory', 'Start (0,0)', 'Stop (-0.5, 0.3)'}, ...
       'Location', 'southwest');

% Top Middle: Velocity & Acc Limit Vel
subplot(2, 3, 2);
plot(t, log_vRef, 'k--', 'LineWidth', 1.4); hold on;
plot(t, log_vLim, 'c-', 'LineWidth', 1.4);
plot(t, log_vMeas, 'b-', 'LineWidth', 1.4);
plot(t, log_accVel, 'r-', 'LineWidth', 1.0);
grid on; xlim([0, tFinish + 0.8]);
xlabel('Time (s)'); ylabel('v (m/s) / a (m/s^2)');
title('Forward Velocity & Acceleration Limiter');
legend({'v_{ref}', 'v_{lim}', 'v_{meas}', 'acc_{vel} (m/s^2)'}, 'Location', 'best');

% Top Right: Heading Angle (Reference vs. Gyro)
subplot(2, 3, 3);
plot(t, log_hRef, 'k--', 'LineWidth', 1.6); hold on;
plot(t, log_head, 'r-', 'LineWidth', 1.5);
grid on; xlim([0, tFinish + 0.8]);
xlabel('Time (s)'); ylabel('Heading (rad)');
title('Heading h(t): Straight -> Circle Ramp -> Straight');
legend({'Desired Heading h_{ref}', 'Measured Heading h_{gyro}'}, 'Location', 'northwest');

% Bottom Middle: Turnrate & Acc Limit Turn
subplot(2, 3, 5);
plot(t, log_wMeas, 'Color', [0.49 0.18 0.56], 'LineWidth', 1.4); hold on;
plot(t, log_accTurn, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.2);
grid on; xlim([0, tFinish + 0.8]);
xlabel('Time (s)'); ylabel('rad/s / m/s^2');
title('Turnrate \omega(t) & Turn Acceleration Limiter');
legend({'Turnrate \omega (rad/s)', 'acc_{turn} (m/s^2)'}, 'Location', 'best');

% Bottom Right: Track Error d(t) in cm & State Machine
subplot(2, 3, 6);
yyaxis left;
plot(t, dev_cm, 'b-', 'LineWidth', 1.6);
ylabel('Track Deviation d(t) (cm)');
ylim([0, max(2.0, d_max_cm * 1.3)]);
yyaxis right;
plot(t, log_state, 'k-', 'LineWidth', 1.2);
ylabel('Mission State (11..15)');
ylim([0, 18]);
grid on; xlim([0, tFinish + 0.8]);
xlabel('Time (s)');
title(sprintf('Track Deviation d(t) (Max = %.2f cm) & State', d_max_cm));

sgtitle('DTU 34765 Digital Control – Final Project Simulation (Keyhole Course)');
saveas(figFP, fullfile(here, 'figures', 'final_project_sim.png'));
fprintf('Plot gespeichert unter: figures/final_project_sim.png\n');
