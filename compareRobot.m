% Lecture 5, step 10: the robot P-controller runs against the Simulink model
% (samplingModel.m) with the same Kp and Ts. Wheels up, velocityStep:
% 0.2s of zero, then 0.1 m/s.
% Log columns: 1 t, 2 state, 3 ref (m/s), 4-5 volt(L,R), 6-7 enc(L,R),
%              8-9 vel(L,R) (motor rad/s), 10 batt, 11-13 gyro,
%              14-15 current(L,R), 16-17 velocity estimate delay(L,R) (ms)
here = fileparts(mfilename('fullpath'));
runs = {   % log, Kp
    'pstep_Kp005_Ts1.log',  0.05
    'pstep_Kp003_Ts1.log',  0.03
    'pstep_Kp003_Ts2.log',  0.03
    'pstep_Kp003_Ts5.log',  0.03
    'pstep_Kp003_Ts10.log', 0.03
    'pstep_Kp003_Ts20.log', 0.03 };
% pstep_Kp005_Ts1_fast.log is left out: with 'encf 1' the left velocity
% estimate has the wrong sign, so that loop had positive feedback.

% same plant and model as samplingModel.m
load(fullfile(here, 'motorParams.mat'), 'R', 'L', 'Kb', 'J', 'D');
K    = Kb;
num  = K;
den  = conv([L R], [J D]) + [0 0 K^2];
gear = 9.6;
r    = 0.0315;
Vmax = 9;
T    = 0.010;
model = 'basebot_model_sampling';
load_system(fullfile(here, [model '.slx']));

fprintf('\nSteady state = mean over 0.5-0.6 s (last 0.1 s of the step). Ref 0.1 m/s\n');
fprintf(' Ts (ms)   Kp  | robot L   robot R | model  (m/s) | delay L  R (ms)\n');
for n = 1:size(runs, 1)
    dd  = readmatrix(fullfile(here, 'logs', runs{n,1}), 'FileType', 'text', 'CommentStyle', '%');
    t   = dd(:,1);
    Ts  = round(median(diff(t)), 4);
    Kp  = runs{n,2};
    vel = dd(:,8:9) * r/gear;                  % motor rad/s -> m/s
    out = sim(model, 0.6);

    h = figure(172); clf
    plot(t, dd(:,3), LineWidth=2);  hold on
    plot(t, vel(:,1), LineWidth=1.5);
    plot(t, vel(:,2), LineWidth=1.5);
    plot(out.velP.Time, out.velP.Data(:,3), '--', LineWidth=1.5);
    plot(t, dd(:,4)/10, ':', LineWidth=1);
    xlim([0 0.6]);  grid on
    legend('ref (m/s)', 'robot left (m/s)', 'robot right (m/s)', ...
           'model, 10 ms delay (m/s)', 'robot left voltage/10 (V)', 'Location', 'northwest')
    title(sprintf("Robot vs model: Ts=%.3f sec, Kp=%.3f", Ts, Kp))
    xlabel('Time (sec)');  ylabel('Velocity (m/s)')
    saveas(h, fullfile(here, 'figures', sprintf('robot_Ts%gms_Kp%g.png', Ts*1e3, Kp)))

    k  = t >= 0.5 & t < 0.6;
    km = out.velP.Time >= 0.5;
    fprintf('%7g  %4.2f | %7.3f   %7.3f | %12.3f | %7.1f %5.1f\n', Ts*1e3, Kp, ...
            mean(vel(k,1)), mean(vel(k,2)), mean(out.velP.Data(km,3)), mean(dd(k,16)), mean(dd(k,17)));
end
