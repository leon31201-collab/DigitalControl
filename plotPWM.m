% Lecture 5, exercise on robot: PWM disturbance on the motor current.
% Run 1: Ts = 1 ms, PWM = 80000 Hz.   Run 2: Ts = 1 ms, PWM = 1990 Hz.
% Both use SequenceTwoSteps: 0.1s of 0V, then 1.5V, 3V, 6V, 0.5s each.
% Log columns: 1 t, 2 state, 3 ref, 4-5 volt(L,R), 6-7 enc(L,R),
%              8-9 vel(L,R), 10 batt, 11-13 gyro, 14-15 current(L,R)

here  = fileparts(mfilename('fullpath'));   % paths work from any MATLAB current folder
files = fullfile(here, 'logs', {'pwm_run1_80000.log', 'pwm_run2_1990.log'});   % <- your two logs
pwm   = [80000 1990];                  % Hz, as set in start()
Ts    = 1e-3;                          % sampleTimeUs = 1000

% Theory: sampling at fs folds f_pwm to |f_pwm - k*fs|, k = nearest integer
fs     = 1/Ts;
fAlias = abs(pwm - round(pwm/fs)*fs);

figure(5); clf
for n = 1:2
    dd   = readmatrix(files{n}, 'FileType', 'text', 'CommentStyle', '%');
    dd   = dd(1:find([diff(dd(:,1)); -1] < 0, 1), :);   % first log only, if sent twice
    t    = dd(:,1);
    volt = dd(:,4);
    cur  = dd(:,14:15);
    % Re-zero the current on the 0V baseline before the first step
    cur  = cur - mean(cur(volt == 0 & t < t(find(volt ~= 0, 1)), :));

    ax(n) = subplot(2,1,n);
    plot(t, cur);  grid on
    ylabel('Current (A)');  legend('left', 'right', 'Location', 'northwest')
    title(sprintf('Run %d: Ts = %g ms, PWM = %d Hz', n, Ts*1e3, pwm(n)))

    % Dominant disturbance frequency on each voltage step. The first 0.1s of
    % each step is the mechanical transient, so only the flat part is used.
    fprintf('\nRun %d, PWM %d Hz  (theory: aliased to %g Hz)\n', n, pwm(n), fAlias(n));
    fprintf('   V    left: f (Hz)  ripple (mA rms)   right: f (Hz)  ripple (mA rms)\n');
    for v = unique(volt(volt > 0))'
        k = find(volt == v);
        k = k(t(k) > t(k(1)) + 0.1);
        fprintf('%5.2f', v);
        for m = 1:2
            x = detrend(cur(k,m));
            N = 8*2^nextpow2(numel(x));                 % zero-padded for a finer peak
            P = abs(fft(x, N));
            f = (0:N/2-1)' * fs/N;
            [~, i] = max(P(2:N/2));                     % skip the DC bin
            fprintf('   %13.1f  %15.1f', f(i+1), rms(x)*1e3);
        end
        fprintf('\n');
    end
end
xlabel('Time (sec)')
linkaxes(ax, 'xy')
saveas(gcf, fullfile(here, 'figures', 'pwm_current.png'))
