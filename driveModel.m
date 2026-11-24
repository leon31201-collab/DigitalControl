% Lecture 6, exercise 1a/1b: robot model with two motors, pose, and
% velocity / turn-rate input (slides 4-7). Builds basebot_drive.slx:
%   Velocity step, Turnrate step -> left = v - w*B/2, right = v + w*B/2
%   -> Robot model (2 inputs, slide 6):
%        Left / Right Motor + velocity control (lecture 5 loop, robot mass)
%        vel = (vl+vr)/2, turnrate = (vr-vl)/B, distance, heading, x, y
% Runs: P-controller as in the firmware (Kp = 0.03), the week-2 PI
% (slide 7, "consider improvements"), and the exercise 2 floor test
% (0.33 m/s for 3 s after 0.2 s, as in velocityStep() in the firmware).
here = fileparts(mfilename('fullpath'));

load(fullfile(here, 'piDesign.mat'), 'R', 'L', 'K', 'J', 'D', 'T');   % J incl. robot mass
num  = K;
den  = conv([L R], [J D]) + [0 0 K^2];       % V -> motor rad/s
gear = 9.6;
r    = 0.0315;       % m
B    = 0.15;         % m, wheel base (slide 4; firmware has 0.14)
Vmax = 9;            % V, firmware motor voltage limit
Ts   = 0.001;        % s, firmware sample time

model = 'basebot_drive';
buildDriveModel(model, here);

pi_ = load(fullfile(here, 'piDesign.mat'), 'Kp', 'tau_i');
tW = 0.5;            % s, turn-rate step time
% Kp, Ki, velocity step (m/s) at tV (s), turn-rate step (rad/s), stop time (s), title, figure
runs = {0.03,    0,                  0.6,  0.1, 1, 2,   'P-control, K_p = 0.03 (firmware)', 'drive_P.png'
        pi_.Kp,  pi_.Kp / pi_.tau_i, 0.6,  0.1, 1, 2,   sprintf('PI-control, K_p = %g, \\tau_i = %g s (week 2)', pi_.Kp, pi_.tau_i), 'drive_PI.png'
        0.03,    0,                  0.33, 0.2, 0, 3.2, 'floor test (ex. 2): 0.33 m/s for 3 s, K_p = 0.03', 'drive_straight.png'};
fprintf('\n%-50s %8s %10s %9s %9s %7s %7s\n', 'run (end values)', 'vel', 'turnrate', 'distance', 'heading', 'x', 'y');
for k = 1:size(runs, 1)
    [Kp, Ki, vStep, tV, wStep] = runs{k, 1:5};
    out = sim(model, runs{k, 6});
    t = out.pose.Time;  d = out.pose.Data;   % vRef wRef vel turnrate distance heading x y
    h = figure(180 + k); clf;  h.Position = [100 100 1500 450];  theme(h, 'light');
    subplot(1, 3, 1)
    plot(t, d(:,1), 'k--', t, d(:,3), t, d(:,2), 'k:', t, d(:,4), LineWidth=1.5);
    legend('velocity ref (m/s)', 'velocity (m/s)', 'turn-rate ref (rad/s)', 'turn-rate (rad/s)', 'Location', 'east')
    title('Driving velocity and turn-rate');  xlabel('Time (sec)');  grid on
    subplot(1, 3, 2)
    plot(t, d(:,7), t, d(:,8), t, d(:,5), t, d(:,6), LineWidth=1.5);
    legend('x (m)', 'y (m)', 'distance (m)', 'heading (rad)', 'Location', 'northwest')
    title('Robot pose');  xlabel('Time (sec)');  grid on
    subplot(1, 3, 3)
    plot(d(:,7), d(:,8), LineWidth=1.5);  hold on;  plot(d(end,7), d(end,8), 'o');
    title('Path');  xlabel('x (m)');  ylabel('y (m)');  axis equal;  grid on
    sgtitle(['Simulink basebot\_drive: ' runs{k, 7}])
    saveas(h, fullfile(here, 'figures', runs{k, 8}))
    e = d(end, :);
    fprintf('%-50s %8.3f %10.3f %9.3f %9.3f %7.3f %7.3f\n', runs{k, 7}, e(3:8));
end

%% ---------------------------------------------------------------------
function buildDriveModel(model, here)
    if bdIsLoaded(model), close_system(model, 0); end
    new_system(model);
    set_param(model, 'MaxStep', '1e-4');
    b = @(lib, name, varargin) add_block(lib, [model '/' name], varargin{:});
    c = @(sys, src, dst) add_line(sys, src, dst, 'autorouting', 'on');

    % --- slide 7: velocity and turn-rate input
    b('simulink/Sources/Step', 'Velocity Step', 'Time', 'tV', 'Before', '0', 'After', 'vStep');
    b('simulink/Sources/Step', 'Turnrate Step', 'Time', 'tW', 'Before', '0', 'After', 'wStep');
    b('simulink/Math Operations/Gain', 'halfB', 'Gain', 'B/2');
    b('simulink/Math Operations/Sum', 'Left', 'Inputs', '+-');
    b('simulink/Math Operations/Sum', 'Right', 'Inputs', '++');
    rm = [model '/Robot model'];
    add_block('built-in/Subsystem', rm);
    buildRobotModel(rm);
    b('simulink/Signal Routing/Mux', 'Mux', 'Inputs', '8');
    b('simulink/Sinks/To Workspace', 'pose', 'VariableName', 'pose', 'SaveFormat', 'Timeseries');
    b('simulink/Sinks/Scope', 'robot', 'NumInputPorts', '1');
    c(model, 'Turnrate Step/1', 'halfB/1');
    c(model, 'Velocity Step/1', 'Left/1');   c(model, 'halfB/1', 'Left/2');
    c(model, 'Velocity Step/1', 'Right/1');  c(model, 'halfB/1', 'Right/2');
    c(model, 'Left/1', 'Robot model/1');           c(model, 'Right/1', 'Robot model/2');
    c(model, 'Velocity Step/1', 'Mux/1');    c(model, 'Turnrate Step/1', 'Mux/2');
    for k = 1:6, c(model, sprintf('Robot model/%d', k), sprintf('Mux/%d', k + 2)); end
    c(model, 'Mux/1', 'pose/1');                   c(model, 'Mux/1', 'robot/1');
    Simulink.BlockDiagram.arrangeSystem(model);
    save_system(model, fullfile(here, [model '.slx']));
end
