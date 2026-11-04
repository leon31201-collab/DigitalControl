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
fprintf('Plots gespeichert unter: figures/latest_drive_straight.png und figures/latest_slide11.png\n');
end
