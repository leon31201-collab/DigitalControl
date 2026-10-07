function plotSamplingExercise(logInput)
% plotSamplingExercise(logInput)
% DTU Digital Control - Basebot Sampling Effects Exercise (Lecture 5)
%
% Verwendung:
%   plotSamplingExercise                            -> Fragt interaktiv nach dem Log (oder Enter fuer neuestes)
%   plotSamplingExercise('device-monitor-80kHz.log') -> Laedt direkt das angegebene Log
%   plotSamplingExercise('80kHz')                   -> Sucht automatisch nach '*80kHz*' im Ordner logs/
%   plotSamplingExercise('111027')                  -> Sucht nach '*111027*' im Ordner logs/
%
% Reproduziert den Folien-Plot aus Lecture 5:
%   - desired/10 (V)
%   - Left (A)
%   - Right (A)
%   - velocity/1000 (kRad/s)
% Bestimmt die dominierende Stoerfrequenz (FFT) und prueft die Aliasing-Theorie.

%% 1. Log-Datei auswaehlen / abfragen
logFolder = 'logs';
if ~exist(logFolder, 'dir')
    error('Ordner "%s" existiert nicht!', logFolder);
end

% Verfuegbare Logs auflisten (nach Datum sortiert, neuestes zuerst)
allLogs = dir(fullfile(logFolder, '*.log'));
[~, sortIdx] = sort([allLogs.datenum], 'descend');
allLogs = allLogs(sortIdx);

if isempty(allLogs)
    error('Keine Log-Dateien im Ordner "%s" gefunden!', logFolder);
end

if nargin < 1 || isempty(logInput)
    fprintf('\n======================================================\n');
    fprintf('  Verfuegbare Logs im Ordner %s/:\n', logFolder);
    fprintf('------------------------------------------------------\n');
    for i = 1:min(length(allLogs), 6)
        fprintf('  [%d] %-35s (%.1f kB, %s)\n', ...
            i, allLogs(i).name, allLogs(i).bytes/1024, allLogs(i).date);
    end
    fprintf('======================================================\n');
    
    userInput = input('Log-Dateiname, Suchbegriff oder Nummer [1]: ', 's');
    userInput = strtrim(userInput);
    if isempty(userInput)
        logFile = fullfile(logFolder, allLogs(1).name);
    elseif ~isnan(str2double(userInput)) && str2double(userInput) >= 1 && str2double(userInput) <= length(allLogs)
        idx = str2double(userInput);
        logFile = fullfile(logFolder, allLogs(idx).name);
    else
        logFile = resolveLogPath(userInput, logFolder);
    end
else
    logFile = resolveLogPath(strtrim(logInput), logFolder);
end

fprintf('\n-> Lade Log-Datei: %s\n', logFile);

%% 2. Header auslesen (PWM-Frequenz und Abtastzeit automatisch erkennen)
pwmFrq = [];
Ts_header = [];

fid = fopen(logFile, 'r');
if fid ~= -1
    for k = 1:25
        line = fgetl(fid);
        if ~ischar(line) || ~startsWith(strtrim(line), '%'), break; end
        
        % PWM Frequenz suchen
        mPwm = regexp(line, 'PWM\s+(\d+)\s*Hz', 'tokens');
        if ~isempty(mPwm), pwmFrq = str2double(mPwm{1}{1}); end
        
        % Abtastzeit suchen
        mTs = regexp(line, 'Sample time\s+(\d+)\s*us', 'tokens');
        if ~isempty(mTs), Ts_header = str2double(mTs{1}{1}) * 1e-6; end
    end
    fclose(fid);
end

% Fallback falls im Header nicht gefunden
if isempty(pwmFrq)
    if contains(logFile, '80k') || contains(logFile, '80000')
        pwmFrq = 80000;
    elseif contains(logFile, '1990')
        pwmFrq = 1990;
    else
        pwmFrq = 80000;
    end
end

%% 3. Messdaten einlesen
dd = readmatrix(logFile, 'FileType', 'text', 'CommentStyle', '%');
dd = dd(all(isfinite(dd), 2), :);

if isempty(dd)
    error('Die Log-Datei "%s" enthaelt keine Datenzeilen!', logFile);
end

% Falls mehrere Laeufe im Log vorhanden sind, den letzten Lauf verwenden
runStarts = [1; find(diff(dd(:,1)) < -0.1) + 1];
if length(runStarts) > 1
    dd = dd(runStarts(end):end, :);
    fprintf('Mehrere Laeufe gefunden. Verwende letzten Lauf (%d Zeilen).\n', size(dd,1));
end

t     = dd(:,1);          % Zeit (s)
state = dd(:,2);          % State
ref   = dd(:,3);          % Sollspannung (V)
voltL = dd(:,4);          % Motorspannung links (V)
voltR = dd(:,5);          % Motorspannung rechts (V)
encL  = dd(:,6);          % Encoder links
encR  = dd(:,7);          % Encoder rechts
velL  = dd(:,8);          % Drehzahl links (rad/s)
velR  = dd(:,9);          % Drehzahl rechts (rad/s)
curL  = dd(:,14);         % Strom links (A)
curR  = dd(:,15);         % Strom rechts (A)

dt = mean(diff(t));
fs = 1 / dt;
vel_kRad = velR / 1000;   % krad/s fuer Folien-Plot

fprintf('Daten geladen: %d Abtastpunkte, Dauer: %.2f s, fs = %.1f Hz, PWM = %d Hz\n', ...
        length(t), max(t) - min(t), fs, pwmFrq);

%% 4. Figure 1: Folien-Plot (Lecture 5 Reproduktion)
fig1 = figure(1); clf;
set(fig1, 'Name', sprintf('Sampling Effects - PWM %d Hz', pwmFrq), 'Color', 'w');

plot(t, ref / 10, 'LineWidth', 1.6, 'Color', [0 0.447 0.741]); hold on;
plot(t, curL, 'LineWidth', 1.0, 'Color', [0.850 0.325 0.098]);
plot(t, curR, 'LineWidth', 1.0, 'Color', [0.929 0.694 0.125]);
plot(t, vel_kRad, 'LineWidth', 1.3, 'Color', [0.494 0.184 0.556]);

grid on;
xlabel('Time (sec)', 'FontSize', 11);
ylim([-1.5 1.0]);
xlim([0 max(t)]);
titleStr = sprintf('sampling %.0f ms and PWM at %d Hz', dt*1e3, pwmFrq);
title(titleStr, 'FontSize', 12, 'FontWeight', 'bold');
legend({'desired/10 (V)', 'Left (A)', 'Right (A)', 'velocity/1000 (kRad/s)'}, ...
       'Location', 'best', 'FontSize', 10);

% Bild im Ordner figures/ abspeichern
if ~exist('figures', 'dir'), mkdir('figures'); end
pngName = fullfile('figures', sprintf('sampling_%.0fms_PWM_%dHz.png', dt*1e3, pwmFrq));
saveas(fig1, pngName);
fprintf('Folienplot gespeichert: %s\n', pngName);

%% 5. Figure 2: Detaillierte Stromansicht
fig2 = figure(2); clf;
set(fig2, 'Name', 'Motor Currents Detailed', 'Color', 'w');

subplot(3,1,1);
plot(t, ref, 'b-', 'LineWidth', 1.2); grid on;
ylabel('Voltage (V)');
title(sprintf('Desired Voltage & Motor Currents (PWM = %d Hz, T_s = %.1f ms)', pwmFrq, dt*1e3));

subplot(3,1,2);
plot(t, curL, 'r-', 'LineWidth', 1.0); grid on;
ylabel('Current Left (A)');

subplot(3,1,3);
plot(t, curR, 'Color', [0.85 0.4 0], 'LineWidth', 1.0); grid on;
ylabel('Current Right (A)');
xlabel('Time (sec)');

linkaxes(findobj(fig2, 'Type', 'axes'), 'x');

%% 6. Figure 3: FFT-Spektralanalyse (Stoerfrequenz)
% Stationaerer Bereich bei der hoechsten Spannungsstufe (6 V)
k_ss = find(ref == 6 & t > (min(t(ref == 6)) + 0.1) & t < max(t(ref == 6)));
if isempty(k_ss) || length(k_ss) < 20
    k_ss = find(ref > 0);
end

if ~isempty(k_ss) && length(k_ss) > 20
    N = length(k_ss);
    
    % Gleichanteil (DC) abziehen
    i_ac_R = curR(k_ss) - mean(curR(k_ss));
    
    % FFT
    I_fft = abs(fft(i_ac_R)) / N;
    f_vec = (0:floor(N/2)) * (fs / N);
    I_mag = 2 * I_fft(1:floor(N/2)+1);
    
    % Dominierenden Peak ab 1 Hz suchen
    valid_idx = find(f_vec >= 1 & f_vec <= fs/2);
    [max_val, p_idx] = max(I_mag(valid_idx));
    f_dom = f_vec(valid_idx(p_idx));
    
    fig3 = figure(3); clf;
    set(fig3, 'Name', 'Disturbance FFT Spectrum', 'Color', 'w');
    stem(f_vec, I_mag, 'LineWidth', 1.2, 'Marker', 'none'); hold on;
    plot(f_vec, I_mag, 'b-', 'LineWidth', 0.8);
    xline(f_dom, 'r--', sprintf('Peak: %.1f Hz (%.3f A)', f_dom, max_val), ...
          'LineWidth', 1.5, 'LabelVerticalAlignment', 'bottom');
    grid on;
    xlim([0 fs/2]);
    xlabel('Frequency (Hz)', 'FontSize', 11);
    ylabel('Amplitude (A)', 'FontSize', 11);
    title(sprintf('Disturbance Spectrum in Current (Dominant = %.1f Hz)', f_dom), 'FontSize', 12);

    % Theoretische Aliasing-Berechnung
    k_fold = round(pwmFrq / fs);
    f_theory = abs(pwmFrq - k_fold * fs);
    
    fprintf('\n======================================================\n');
    fprintf('             SAMPLING EFFECTS ANALYSIS                \n');
    fprintf('======================================================\n');
    fprintf('Abtastrate fs:          %.1f Hz (Ts = %.3f ms)\n', fs, dt*1e3);
    fprintf('PWM-Frequenz:           %d Hz\n', pwmFrq);
    fprintf('------------------------------------------------------\n');
    fprintf('Frage 1: Dominierende Stoerfrequenz?\n');
    fprintf('         -> Gemessen im Spektrum:  %.1f Hz\n', f_dom);
    fprintf('------------------------------------------------------\n');
    fprintf('Frage 2: Stimmt das mit der Theorie ueberein?\n');
    fprintf('         -> Theoretische Faltung (Aliasing):\n');
    fprintf('            f_alias = |f_PWM - k * fs|\n');
    fprintf('                    = |%d - %d * %.0f| = %.1f Hz\n', ...
            pwmFrq, k_fold, fs, f_theory);
    if abs(f_dom - f_theory) < 2
        fprintf('         -> JA! Die gemessene Frequenz stimmt exakt mit der Theorie ueberein.\n');
    else
        fprintf('         -> Hinweis: Pruefe Frequenz-Einstellung oder Laenge des stationaeren Bereichs.\n');
    end
    fprintf('======================================================\n\n');
end

end

%% Hilfsfunktion: Pfad aufloesen
function pathOut = resolveLogPath(userInput, logFolder)
    % 1. Direkte Datei
    if exist(userInput, 'file')
        pathOut = userInput;
        return;
    end
    % 2. Im logs/ Ordner
    p1 = fullfile(logFolder, userInput);
    if exist(p1, 'file')
        pathOut = p1;
        return;
    end
    % 3. Mit .log Endung
    if ~endsWith(userInput, '.log')
        p2 = fullfile(logFolder, [userInput '.log']);
        if exist(p2, 'file')
            pathOut = p2;
            return;
        end
    end
    % 4. Wildcard-Suche (z.B. '80kHz' -> 'device-monitor-80kHz.log')
    matches = dir(fullfile(logFolder, ['*' userInput '*.log']));
    if ~isempty(matches)
        pathOut = fullfile(logFolder, matches(1).name);
        return;
    end
    error('Konnte keine Log-Datei zu "%s" im Ordner "%s" finden!', userInput, logFolder);
end
