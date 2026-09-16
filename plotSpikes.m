% Plot the spikeSequence log: 3 x 6V spikes, 10ms each, 300ms apart.
% Wheels off the floor - this is the motor-only transient (R, L).
% Log columns: 1 t, 2 state, 3 ref, 4-5 volt(L,R), 6-7 enc(L,R),
%              8-9 vel(L,R), 10 batt, 11-13 gyro, 14-15 current(L,R)

file = 'logs/device-monitor-260909-111426.log';   % <- your spike log
motor = 2;                                % 1 = left, 2 = right
PPR = 48;                                 % encoder counts per motor rev

dd = readmatrix(file);
t    = dd(:,1);
volt = dd(:,3+motor);
enc  = dd(:,5+motor);
vel  = dd(:,7+motor);
cur  = dd(:,13+motor);
ts   = mean(diff(t));

% The firmware current offset is hard-coded for another robot, so re-zero
% on the samples before the first spike, where the motor is off.
idle = volt == 0 & t < t(find(volt ~= 0, 1));
cur  = cur - mean(cur(idle));

% Velocity from the encoder. One count is 2*pi/PPR rad, so at a 0.5ms sample
% a single count is already ~260 rad/s - raw diff is 0-or-huge, hence movmean.
dEnc   = [0; diff(enc)];
velEst = movmean(dEnc, 20) * (2*pi/PPR) / ts;

figure(1); clf
ax(1) = subplot(4,1,1);
plot(t, volt); grid on
names = {'left','right'};
ylabel('Voltage (V)'); title(['Spike response - ' names{motor} ' motor'])

ax(2) = subplot(4,1,2);
plot(t, cur); grid on
ylabel('Current (A)')

ax(3) = subplot(4,1,3);
stairs(t, dEnc); grid on
ylabel('diff(encoder)'); ylim([-1 max(dEnc)+1])

ax(4) = subplot(4,1,4);
plot(t, velEst, t, vel); grid on
ylabel('Velocity (rad/s)'); xlabel('Time (sec)')
legend('from diff(encoder)', 'logged', 'Location', 'northwest')

linkaxes(ax, 'x')

% Spike edges, for zooming in on one transient
on  = find(diff(volt > 0) ==  1) + 1;
off = find(diff(volt > 0) == -1) + 1;
fprintf('%d spikes, %.1f ms each, peak current %.2f A\n', ...
        numel(on), mean(t(off)-t(on))*1000, max(cur));
