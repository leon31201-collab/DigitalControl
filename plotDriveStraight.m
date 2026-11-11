function plotDriveStraight(logInput)
% plotDriveStraight(logInput)
% Plots the Week 7/8 "Driving Straight", Turnrate & Heading Control log from Basebot.
% Compares Encoder vs. IMU Gyro Turnrate & Heading (as on Week 8 Slides 2 & 10).
%
% Verwendung:
%   plotDriveStraight                                   -> Zeigt Liste der device-monitor-YYMMDD-HHMMSS.log Dateien (Enter = neuestes)
%   plotDriveStraight('device-monitor-261021-105744.log') -> Laedt direkt das angegebene Log
%   plotDriveStraight('261021')                         -> Sucht automatisch nach '*261021*' im Ordner logs/

logFolder = 'logs';
allLogs = dir(fullfile(logFolder, 'device-monitor-*.log'));
% Nur nicht-leere Logs anzeigen (Mindestgroesse 1 kB)
allLogs = allLogs([allLogs.bytes] > 1000);
[~, sortIdx] = sort([allLogs.datenum], 'descend');
allLogs = allLogs(sortIdx);

if isempty(allLogs)
    error('Keine nicht-leere device-monitor-*.log Datei im Ordner logs/ gefunden.');
end

if nargin < 1 || isempty(logInput)
    fprintf('\n======================================================\n');
    fprintf('  Verfuegbare Logs im Ordner %s/:\n', logFolder);
    fprintf('------------------------------------------------------\n');
    for i = 1:min(length(allLogs), 8)
        fprintf('  [%d] %-35s (%.1f kB, %s)\n', ...
            i, allLogs(i).name, allLogs(i).bytes/1024, allLogs(i).date);
    end
    fprintf('======================================================\n');
    userInput = strtrim(input('Log-Dateiname, Datum oder Nummer [1 = neuestes]: ', 's'));
    if isempty(userInput)
        logFile = fullfile(logFolder, allLogs(1).name);
    elseif ~isnan(str2double(userInput)) && str2double(userInput) >= 1 && str2double(userInput) <= length(allLogs)
        logFile = fullfile(logFolder, allLogs(str2double(userInput)).name);
    else
        matches = dir(fullfile(logFolder, ['*' userInput '*.log']));
        if ~isempty(matches)
            logFile = fullfile(logFolder, matches(1).name);
        else
            logFile = fullfile(logFolder, userInput);
        end
    end
elseif exist(logInput, 'file')
    logFile = logInput;
elseif exist(fullfile(logFolder, logInput), 'file')
    logFile = fullfile(logFolder, logInput);
else
    matches = dir(fullfile(logFolder, ['*' logInput '*.log']));
    if ~isempty(matches)
        logFile = fullfile(logFolder, matches(1).name);
    else
        error('Konnte keine Log-Datei zu "%s" finden.', logInput);
    end
end

fprintf('Lese Log-Datei: %s\n', logFile);
lines = readlines(logFile);
numData = [];
for i = 1:numel(lines)
    s = strtrim(lines(i));
    if strlength(s) == 0 || startsWith(s, "%")
        continue;
    end
    % Replace 'ovf' or 'nan' with NaN so str2double works cleanly
    s = replace(s, "ovf", "NaN");
    vals = str2double(split(s));
    if numel(vals) >= 22 && ~isnan(vals(1))
        row = nan(1, 29);
        nCols = min(numel(vals), 29);
        row(1:nCols) = vals(1:nCols).';
        numData = [numData; row]; %#ok<AGROW>
    end
end

if isempty(numData)
    error('Keine gueltigen Datenzeilen in %s gefunden.', logFile);
end

t           = numData(:, 1);
stateCol    = numData(:, 2);
vRef        = numData(:, 3);
uLeft       = numData(:, 4);
uRight      = numData(:, 5);
gyroZ_deg   = numData(:, 13);
vMeas       = numData(:, 16);
turnEnc     = numData(:, 17);
distA       = numData(:, 18);
headingEnc  = numData(:, 19);
posX        = numData(:, 20);
posY        = numData(:, 21);
headU       = numData(:, 22);
gyroTurn    = numData(:, 23);
gyroHead    = numData(:, 24);
headRef     = numData(:, 25);
accVel      = numData(:, 28);
accTurn     = numData(:, 29);

% Falls das Log noch aus 22 Spalten besteht (ohne Spalte 23-25),
% berechnen wir Gyro-Turnrate und Gyro-Heading direkt aus Spalte 13 (gyro Z in deg/s)
% mit 22 Grad Neigungskorrektur (Week 8 Slide 2) und Tustin-Integration:
if all(isnan(gyroTurn)) && any(~isnan(gyroZ_deg))
    idxStand = (stateCol == 10) & ~isnan(gyroZ_deg);
    if any(idxStand)
        gyroOffset = mean(gyroZ_deg(idxStand));
    else
        gyroOffset = gyroZ_deg(1);
    end
    gyroTurn = ((gyroZ_deg - gyroOffset) * (pi / 180)) / cosd(22);
    gyroHead = cumtrapz(t, gyroTurn);
end

hasGyroCols = any(~isnan(gyroTurn));
hasAccCols  = any(~isnan(accVel)) && any(~isnan(accTurn));

fig = figure('Name', ['Basebot Control Analysis: ' logFile], 'Color', 'w', ...
             'Position', [60, 60, 1250, 820]);
try theme(fig, 'light'); catch, end

% 1. Forward Velocity & Acc Limit Vel
subplot(3,2,1);
plot(t, vRef, 'k--', 'LineWidth', 1.5); hold on;
plot(t, vMeas, 'b-', 'LineWidth', 1.4);
if hasAccCols
    plot(t, accVel, 'r-', 'LineWidth', 1.1);
    legend({'Reference v_{ref}', 'Measured v (enc)', 'Acc limit vel (m/s^2)'}, 'Location', 'best');
else
    legend({'Reference v_{ref}', 'Measured v (enc)'}, 'Location', 'best');
end
grid on;
ylabel('v (m/s) / a (m/s^2)'); title('Forward Velocity v(t) & Acc Limit');

% 2. Turnrate: Encoder vs. Gyro & Acc Limit Turn
subplot(3,2,2);
plot(t, turnEnc, 'Color', [0.6 0.2 0.8], 'LineWidth', 0.9); hold on;
if hasGyroCols
    plot(t, gyroTurn, 'c-', 'LineWidth', 1.4);
    if hasAccCols
        plot(t, accTurn, 'r--', 'LineWidth', 1.2);
        legend({'Turnrate Enc (rad/s)', 'Turnrate Gyro (rad/s)', 'Acc limit turn (m/s^2)'}, 'Location', 'best');
    else
        legend({'Turnrate Encoder (rad/s)', 'Turnrate Gyro Z (rad/s)'}, 'Location', 'best');
    end
else
    plot(t, headU, 'm-', 'LineWidth', 1.2);
    legend({'Turnrate Encoder (rad/s)', 'headU (m/s)'}, 'Location', 'best');
end
grid on;
ylabel('Turnrate (rad/s)'); title('Turnrate: Encoder vs. Gyro Z (Week 8/9)');

% 3. Motor Voltages
subplot(3,2,3);
plot(t, uLeft, 'b-', t, uRight, 'r-', 'LineWidth', 1.1); grid on;
ylabel('Voltage (V)'); title('Motor Voltages (Left & Right)');
legend({'Left V', 'Right V'}, 'Location', 'best');

% 4. Heading Angle: Reference vs. Encoder vs. Gyro
subplot(3,2,4);
if any(~isnan(headRef))
    plot(t, headRef, 'k--', 'LineWidth', 1.5); hold on;
end
plot(t, headingEnc, 'g-', 'LineWidth', 1.5); hold on;
if hasGyroCols
    plot(t, gyroHead, 'r-', 'LineWidth', 1.5);
    if any(~isnan(headRef))
        legend({'Desired Heading', 'Heading Encoder (rad)', 'Heading Gyro (rad)'}, 'Location', 'best');
    else
        legend({'Heading Encoder (rad)', 'Heading Gyro (rad)'}, 'Location', 'best');
    end
else
    legend({'Heading Encoder (rad)'}, 'Location', 'best');
end
grid on;
ylabel('Heading (rad)'); title('Heading Angle h(t): Encoder vs. Gyro');

% 5. Distance, x(t), y(t)
subplot(3,2,5);
plot(t, distA, 'g-', 'LineWidth', 1.5); hold on;
plot(t, posX, 'b--', 'LineWidth', 1.5);
plot(t, posY, 'r-', 'LineWidth', 1.5); grid on;
xlabel('Time (s)'); ylabel('Position / Distance (m)');
title('Distance, x(t) and y(t)');
legend({'Distance', 'x (m)', 'y (m)'}, 'Location', 'best');

% 6. 2D Path (x vs y): Compare Encoder Odometry vs. Gyro-Reconstructed Path
subplot(3,2,6);
xEnc = cumtrapz(t, vMeas .* cos(headingEnc));
yEnc = cumtrapz(t, vMeas .* sin(headingEnc));
plot(xEnc, yEnc, 'b-', 'LineWidth', 2); hold on;
if hasGyroCols
    xGyro = cumtrapz(t, vMeas .* cos(gyroHead));
    yGyro = cumtrapz(t, vMeas .* sin(gyroHead));
    plot(xGyro, yGyro, 'r--', 'LineWidth', 1.8);
    plot(xGyro(1), yGyro(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    plot(xGyro(end), yGyro(end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
else
    plot(xEnc(1), yEnc(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    plot(xEnc(end), yEnc(end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
end
grid on; axis equal;
xlabel('x (m)'); ylabel('y (m)'); title('2D Path (x vs y): Encoder vs. Gyro');
if hasGyroCols
    legend({'Encoder Path', 'Gyro Path', 'Start', 'End'}, 'Location', 'best');
end

sgtitle(['Basebot Week 8/9 Analysis: ' strrep(logFile, '\', '/')]);
if ~exist('figures', 'dir'), mkdir('figures'); end
saveas(fig, fullfile('figures', 'latest_drive_straight.png'));

% Zweiter Plot exakt im Stil von Week 8 Slide 11 / Week 9 Slide 11
figSlide = figure('Name', ['Slide 11 Comparison: ' logFile], 'Color', 'w', ...
                  'Position', [120, 120, 950, 560]);
try theme(figSlide, 'light'); catch, end
plot(t, vRef, 'Color', [0.00 0.45 0.74], 'LineWidth', 1.5); hold on;
plot(t, vMeas, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.5);
plot(t, turnEnc, 'Color', [0.93 0.69 0.13], 'LineWidth', 1.2);
if hasGyroCols
    plot(t, gyroTurn, 'Color', [0.49 0.18 0.56], 'LineWidth', 1.5);
end
plot(t, headingEnc, 'Color', [0.47 0.67 0.19], 'LineWidth', 1.8);
if hasGyroCols
    plot(t, gyroHead, 'Color', [0.30 0.75 0.93], 'LineWidth', 1.8);
end
if any(~isnan(headRef))
    plot(t, headRef, 'Color', [0.64 0.08 0.18], 'LineWidth', 1.8);
end
legEntries = {'Desired velocity', 'Velocity', 'Turnrate enc', 'Turnrate gyro', ...
              'Heading enc', 'Heading gyro', 'Desired heading'};
if hasAccCols
    plot(t, accVel, 'Color', [0.00 0.50 0.80], 'LineWidth', 1.6);
    plot(t, accTurn, 'Color', [0.90 0.35 0.10], 'LineWidth', 1.6);
    legEntries = [legEntries, {'Acc limit vel', 'Acc limit turn'}];
end
grid on;
xlabel('Time (sec)');
ylim([-2.5, 7.0]);
if hasAccCols
    title(['Week 9 Slide 11 – Square with Acceleration Limit (' strrep(logFile, '\', '/') ')']);
else
    title(['Week 8 Slide 11 – Square without Acceleration Limit (' strrep(logFile, '\', '/') ')']);
end
legend(legEntries, 'Location', 'northwest');
saveas(figSlide, fullfile('figures', 'latest_slide11.png'));

% Falls es ein Final-Project Keyhole-Lauf ist (Gesamtstrecke > 4.2 m),
% erstelle zusaetzlich den offiziellen Final-Project-Plot mit Score P_i = t_i + d_max!
if max(distA) > 4.2
    L1 = 1.00; L2 = 1.50; R_circ = 0.50; d_gap = 0.30;
    alpha = asin((d_gap / 2) / R_circ);
    xC = L1 + sqrt(R_circ^2 - (d_gap/2)^2);
    yC = d_gap / 2;
    s1_vec = linspace(0, L1, 500)';
    phi_vec = linspace(-pi + alpha, pi - alpha, 1500)';
    s3_vec = linspace(0, L2, 750)';
    x_track = [s1_vec; xC + R_circ * cos(phi_vec); L1 - s3_vec];
    y_track = [zeros(size(s1_vec)); yC + R_circ * sin(phi_vec); d_gap * ones(size(s3_vec))];

    dev_cm = zeros(numel(t), 1);
    for k = 1:numel(t)
        dev_cm(k) = min(hypot(x_track - posX(k), y_track - posY(k))) * 100;
    end
    d_max_cm = max(dev_cm);
    idxStop = find(stateCol == 2, 1, 'first');
    if ~isempty(idxStop)
        tFinish = t(idxStop) - 0.10;
    else
        tFinish = t(end) - 0.10;
    end
    P_score = tFinish + d_max_cm;

    figFP = figure('Name', ['Final Project Real Robot: ' logFile], 'Color', 'w', ...
                   'Position', [50, 50, 1320, 760]);
    try theme(figFP, 'light'); catch, end

    subplot(2, 3, [1 4]);
    plot(-y_track, x_track, 'k-', 'LineWidth', 5.0); hold on;
    plot(-yEnc, xEnc, 'b:', 'LineWidth', 1.6);
    plot(-posY, posX, 'r--', 'LineWidth', 2.2);
    plot(0, 0, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(-posY(end), posX(end), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    grid on; axis equal;
    xlim([-0.85, 0.55]); ylim([-0.65, 2.15]);
    xlabel('Lateral Position -y (m)'); ylabel('Forward Position x (m)');
    title(sprintf('Real Robot Keyhole Run (t_i = %.2f s, d_{max} = %.2f cm, P_i = %.2f)', ...
                  tFinish, d_max_cm, P_score));
    legend({'Black Marking (Target)', 'Encoder Odometry', 'Gyro Trajectory (Real)', 'Start', 'Stop'}, ...
           'Location', 'southwest');

    subplot(2, 3, 2);
    plot(t, vRef, 'k--', 'LineWidth', 1.4); hold on;
    plot(t, vMeas, 'b-', 'LineWidth', 1.4);
    if hasAccCols, plot(t, accVel, 'r-', 'LineWidth', 1.0); end
    grid on; xlabel('Time (s)'); ylabel('v (m/s) / a (m/s^2)');
    title('Forward Velocity & Acc Limiter');
    legend({'v_{ref}', 'v_{meas}', 'acc_{vel}'}, 'Location', 'best');

    subplot(2, 3, 3);
    plot(t, headRef, 'k--', 'LineWidth', 1.6); hold on;
    plot(t, headingEnc, 'g-', 'LineWidth', 1.4);
    plot(t, gyroHead, 'r-', 'LineWidth', 1.5);
    grid on; xlabel('Time (s)'); ylabel('Heading (rad)');
    title('Heading h(t): Reference vs. Encoder vs. Gyro');
    legend({'Desired h_{ref}', 'Heading Enc', 'Heading Gyro'}, 'Location', 'northwest');

    subplot(2, 3, 5);
    plot(t, gyroTurn, 'Color', [0.49 0.18 0.56], 'LineWidth', 1.4); hold on;
    if hasAccCols, plot(t, accTurn, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.2); end
    grid on; xlabel('Time (s)'); ylabel('rad/s / m/s^2');
    title('Gyro Turnrate \omega(t) & Turn Acc Limiter');
    legend({'Turnrate Gyro (rad/s)', 'acc_{turn} (m/s^2)'}, 'Location', 'best');

    subplot(2, 3, 6);
    yyaxis left;
    plot(t, dev_cm, 'b-', 'LineWidth', 1.6);
    ylabel('Track Deviation d(t) (cm)'); ylim([0, max(3.0, d_max_cm * 1.25)]);
    yyaxis right;
    plot(t, stateCol, 'k-', 'LineWidth', 1.2);
    ylabel('Mission State (11..15)'); ylim([0, 18]);
    grid on; xlabel('Time (s)');
    title(sprintf('Track Deviation d(t) (Max = %.2f cm) & State', d_max_cm));

    sgtitle(['DTU 34765 Final Project – Real Robot Telemetry (' strrep(logFile, '\', '/') ')']);
    saveas(figFP, fullfile('figures', 'real_final_project.png'));
    fprintf('Final Project Plot gespeichert unter: figures/real_final_project.png (P_i = %.2f)\n', P_score);
end
fprintf('Plots gespeichert unter: figures/latest_drive_straight.png und figures/latest_slide11.png\n');
end
