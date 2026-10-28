function plotDriveStraight(logFile)
% plotDriveStraight(logFile)
% Plots the Week 7/8 "Driving Straight", Turnrate & Heading Control log from Basebot.
% Compares Encoder vs. IMU Gyro Turnrate & Heading (as on Week 8 Slides 2 & 10).
% If no logFile is given, automatically picks the newest non-empty log in logs/.

if nargin < 1 || isempty(logFile)
    files = dir(fullfile('logs', 'device-monitor-*.log'));
    files = files([files.bytes] > 1000);
    if isempty(files)
        error('Keine nicht-leere Log-Datei im Ordner logs/ gefunden.');
    end
    [~, idx] = max([files.datenum]);
    logFile = fullfile('logs', files(idx).name);
elseif ~exist(logFile, 'file') && exist(fullfile('logs', logFile), 'file')
    logFile = fullfile('logs', logFile);
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
        row = nan(1, 25);
        nCols = min(numel(vals), 25);
        row(1:nCols) = vals(1:nCols).';
        numData = [numData; row]; %#ok<AGROW>
    end
end

if isempty(numData)
    error('Keine gueltigen Datenzeilen in %s gefunden.', logFile);
end

t           = numData(:, 1);
vRef        = numData(:, 3);
uLeft       = numData(:, 4);
uRight      = numData(:, 5);
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

hasGyroCols = any(~isnan(gyroTurn));

fig = figure('Name', ['Basebot Control Analysis: ' logFile], 'Color', 'w', ...
             'Position', [80, 80, 1150, 780]);

% 1. Forward Velocity
subplot(3,2,1);
plot(t, vRef, 'k--', 'LineWidth', 1.5); hold on;
plot(t, vMeas, 'b-', 'LineWidth', 1.3); grid on;
ylabel('Velocity (m/s)'); title('Forward Velocity v(t)');
legend({'Reference v_{ref}', 'Measured v (enc)'}, 'Location', 'best');

% 2. Turnrate: Encoder vs. Gyro
subplot(3,2,2);
plot(t, turnEnc, 'Color', [0.6 0.2 0.8], 'LineWidth', 0.9); hold on;
if hasGyroCols
    plot(t, gyroTurn, 'c-', 'LineWidth', 1.4);
    legend({'Turnrate Encoder (rad/s)', 'Turnrate Gyro Z (rad/s)'}, 'Location', 'best');
else
    plot(t, headU, 'm-', 'LineWidth', 1.2);
    legend({'Turnrate Encoder (rad/s)', 'headU (m/s)'}, 'Location', 'best');
end
grid on;
ylabel('Turnrate (rad/s)'); title('Turnrate: Encoder vs. Gyro Z (Week 8 Slide 2)');

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
    legend({'Desired Heading', 'Heading Encoder (rad)', 'Heading Gyro (rad)'}, 'Location', 'best');
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
plot(posX, posY, 'b-', 'LineWidth', 2); hold on;
if hasGyroCols
    dt = [0; diff(t)];
    xGyro = cumtrapz(t, vMeas .* cos(gyroHead));
    yGyro = cumtrapz(t, vMeas .* sin(gyroHead));
    plot(xGyro, yGyro, 'r--', 'LineWidth', 1.8);
end
plot(posX(1), posY(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot(posX(end), posY(end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
grid on; axis equal;
xlabel('x (m)'); ylabel('y (m)'); title('2D Path (x vs y): Encoder vs. Gyro');
if hasGyroCols
    legend({'Encoder Path', 'Gyro Path', 'Start', 'End'}, 'Location', 'best');
end

sgtitle(['Basebot Week 7/8 Analysis: ' strrep(logFile, '\', '/')]);
if ~exist('figures', 'dir'), mkdir('figures'); end
saveas(fig, fullfile('figures', 'latest_drive_straight.png'));
fprintf('Plot gespeichert unter: figures/latest_drive_straight.png\n');
end
