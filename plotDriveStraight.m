function plotDriveStraight(logFile)
% plotDriveStraight(logFile)
% Plots the Week 6/7 "Driving Straight" & Turnrate Control log from Basebot.
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
    % Replace 'ovf' or 'nan' with NaN so sscanf/str2double works cleanly
    s = replace(s, "ovf", "NaN");
    vals = str2double(split(s));
    if numel(vals) >= 22 && ~isnan(vals(1))
        numData = [numData; vals(1:22).']; %#ok<AGROW>
    end
end

if isempty(numData)
    error('Keine gueltigen 22-spaltigen Datenzeilen in %s gefunden.', logFile);
end

t        = numData(:, 1);
vRef     = numData(:, 3);
uLeft    = numData(:, 4);
uRight   = numData(:, 5);
wLeft    = numData(:, 8);
wRight   = numData(:, 9);
vMeas    = numData(:, 16);
turnMeas = numData(:, 17);
distA    = numData(:, 18);
heading  = numData(:, 19);
posX     = numData(:, 20);
posY     = numData(:, 21);
headU    = numData(:, 22);

fig = figure('Name', ['Basebot Drive Straight: ' logFile], 'Color', 'w', ...
             'Position', [100, 100, 1100, 750]);

subplot(3,2,1);
plot(t, vRef, 'k--', 'LineWidth', 1.5); hold on;
plot(t, vMeas, 'b-', 'LineWidth', 1.2); grid on;
ylabel('Velocity (m/s)'); title('Forward Velocity v(t)');
legend({'Reference v_{ref}', 'Measured v'}, 'Location', 'best');

subplot(3,2,2);
plot(t, turnMeas, 'r-', 'LineWidth', 1.1); hold on;
plot(t, headU, 'm-', 'LineWidth', 1.2); grid on;
ylabel('rad/s  |  m/s'); title('Turnrate \omega_z (rad/s) & Controller Output headU (m/s)');
legend({'Turnrate (rad/s)', 'headU (m/s)'}, 'Location', 'best');

subplot(3,2,3);
plot(t, uLeft, 'b-', t, uRight, 'r-', 'LineWidth', 1.1); grid on;
ylabel('Voltage (V)'); title('Motor Voltages (Left & Right)');
legend({'Left V', 'Right V'}, 'Location', 'best');

subplot(3,2,4);
plot(t, heading, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.5); grid on;
ylabel('Heading (rad)'); title('Heading Angle h(t)');

subplot(3,2,5);
plot(t, distA, 'g-', 'LineWidth', 1.5); hold on;
plot(t, posX, 'b--', 'LineWidth', 1.5);
plot(t, posY, 'r-', 'LineWidth', 1.5); grid on;
xlabel('Time (s)'); ylabel('Position / Distance (m)');
title('Distance, x(t) and y(t)');
legend({'Distance', 'x (m)', 'y (m)'}, 'Location', 'best');

subplot(3,2,6);
plot(posX, posY, 'b-', 'LineWidth', 2); grid on;
xlabel('x (m)'); ylabel('y (m)'); title('2D Path (x vs y)');
ylim([-0.2 0.2]);

sgtitle(['Basebot Log Analysis: ' strrep(logFile, '\', '/')]);
if ~exist('figures', 'dir'), mkdir('figures'); end
saveas(fig, fullfile('figures', 'latest_drive_straight.png'));
fprintf('Plot gespeichert unter: figures/latest_drive_straight.png\n');
end
