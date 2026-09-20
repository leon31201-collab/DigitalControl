% Plot the SequenceTwoSteps log: 0.1s of 0V, then 1.5V, 3V, 6V, 0.5s each.
% Wheels off the floor - motor-only steady state.
% Log columns: 1 t, 2 state, 3 ref, 4-5 volt(L,R), 6-7 enc(L,R),
%              8-9 vel(L,R), 10 batt, 11-13 gyro, 14-15 current(L,R)

file  = 'logs/device-monitor-260917-114607.log';   % <- your step log
motor = 2;                                % 1 = left, 2 = right

dd   = readmatrix(file, 'FileType', 'text', 'CommentStyle', '%');
dd   = dd(1:find([diff(dd(:,1)); -1] < 0, 1), :);   % first log only, if sent twice
t    = dd(:,1);
volt = dd(:,3+motor);
vel  = dd(:,7+motor);
cur  = dd(:,13+motor);

% Re-zero the current on the 0V baseline before the first step
cur = cur - mean(cur(volt == 0 & t < t(find(volt ~= 0, 1))));

names = {'left','right'};
figure(2); clf
ax(1) = subplot(3,1,1);  plot(t, volt);  grid on
ylabel('Voltage (V)');  title(['Step response - ' names{motor} ' motor'])
ax(2) = subplot(3,1,2);  plot(t, cur);   grid on
ylabel('Current (A)')
ax(3) = subplot(3,1,3);  plot(t, vel);   grid on
ylabel('Velocity (rad/s)');  xlabel('Time (sec)')
linkaxes(ax, 'x')

% Steady state = mean over the last 0.2s of each step
fprintf('\n   V      i (A)   w (rad/s)\n');
for v = unique(volt(volt > 0))'
    k  = find(volt == v);
    ss = k(t(k) > t(k(end)) - 0.2);
    fprintf('%5.2f  %7.3f  %8.1f\n', v, mean(cur(ss)), mean(vel(ss)));
end
