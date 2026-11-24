% Lecture 8: drive a square (slides 5-8). Fwd 0.75 m at 0.75 m/s, then
% 4 x: turn 90 deg, fwd 0.75 m. Builds basebot_turn.slx:
%   Sequence -> Heading controller -> B/2 -> Mixer -> Robot model (lecture 6)
% Gains and sequence are the ones in basebot_6.ino (MISSION_MODE 2), so this
% predicts the firmware. The heading controller is the firmware cascade
% (P on heading, limited, then PI on turn rate) instead of the slide's lead.
% The model has no friction, wheel slip or gyro drift.
here = fileparts(mfilename('fullpath'));

load(fullfile(here, 'piDesign.mat'), 'R', 'L', 'K', 'J', 'D', 'T');   % J incl. robot mass
num  = K;
den  = conv([L R], [J D]) + [0 0 K^2];       % V -> motor rad/s
gear = 9.6;
r    = 0.0315;       % m
B    = 0.15;         % m, wheel base
Vmax = 9;            % V
Ts   = 0.001;        % s
Kp   = 0.02;   Ki  = Kp / 0.04;              % wheel velocity PI (Kp_vel, taui_vel)
Kph  = 5;      maxTurn = 5;                  % heading P (Kp_head, maxTurnrate)
Kpt  = 0.5;    Kit = Kpt / 0.15;             % turn rate PI (Kp_turn, taui_turn)

model = 'basebot_turn';
buildTurnModel(model, here);

out = sim(model, 6.2);
t = out.robot.Time;  d = out.robot.Data;
% state vRef headRef vel turnrate distance heading x y refLeft refRight
[state, vRef, headRef, vel, turn, dist, head, x, y, refL, refR] = ...
    deal(d(:,1), d(:,2), d(:,3), d(:,4), d(:,5), d(:,6), d(:,7), d(:,8), d(:,9), d(:,10), d(:,11));
vRight = vel + turn * B/2;
uL = out.logsout.getElement('uLeft').Values.Data;
uR = out.logsout.getElement('uRight').Values.Data;

h = figure(230); clf;  h.Position = [80 80 1250 800];  theme(h, 'light');
subplot(2, 2, 1)
plot(t, headRef, t, head, t, state/10, LineWidth=1.5);
legend('desired heading (rad)', 'heading (rad)', 'state/10', 'Location', 'northwest')
title('Heading');  xlabel('Time (sec)');  grid on
subplot(2, 2, 2)
plot(t, vRef, 'k--', t, vel, t, vRight, t, turn, LineWidth=1.5);
legend('vel ref (m/s)', 'velocity (m/s)', 'right wheel (m/s)', 'turn rate (rad/s)', 'Location', 'northeast')
title('Velocity and turn rate');  xlabel('Time (sec)');  grid on
subplot(2, 2, 3)
plot(t, x, t, y, t, dist, LineWidth=1.5);
legend('x (m)', 'y (m)', 'distance (m)', 'Location', 'northwest')
title('Position and distance');  xlabel('Time (sec)');  grid on
subplot(2, 2, 4)
plot(x, y, LineWidth=1.5);  hold on;  plot(x(1), y(1), 'o', x(end), y(end), 's');
legend('path', 'start', 'end', 'Location', 'best')
title('Path');  xlabel('x (m)');  ylabel('y (m)');  axis equal;  grid on
sgtitle('Square at 0.75 m/s: Simulink simulation (firmware gains), not robot data')
saveas(h, fullfile(here, 'figures', 'square_sim.png'))

fprintf('\nend: t=%.2f s, distance %.3f m, heading %.3f rad (2*pi=%.3f), x %.3f, y %.3f m\n', ...
        t(end), dist(end), head(end), 2*pi, x(end), y(end));
fprintf('peak turn rate %.2f rad/s, peak wheel ref %.2f m/s, peak voltage %.2f V\n', ...
        max(abs(turn)), max(abs([refL; refR])), max(abs([uL(:); uR(:)])));
for s = 11:15
    k = find(state == s, 1, 'last');
    fprintf('leg %d end: t=%.2f, heading error %+.3f rad, x %.3f, y %.3f\n', s - 10, t(k), headRef(k) - head(k), x(k), y(k));
end
assert(abs(head(end) - 2*pi) < 0.02 && dist(end) > 3.75 && dist(end) < 4.2, 'square did not finish');
assert(max(state) == 15 && state(end) == 2, 'sequence did not reach all legs and stop');

%% ---------------------------------------------------------------------
function buildTurnModel(model, here)
    if bdIsLoaded(model), close_system(model, 0); end
    new_system(model);
    set_param(model, 'MaxStep', '1e-4');
    b = @(lib, name, varargin) add_block(lib, [model '/' name], varargin{:});
    c = @(src, dst) add_line(model, src, dst, 'autorouting', 'on');

    b('simulink/Sources/Clock', 'Clock');
    b('simulink/Discrete/Zero-Order Hold', 'ZOHt', 'SampleTime', 'Ts');
    b('simulink/Discrete/Zero-Order Hold', 'ZOHd', 'SampleTime', 'Ts');
    b('simulink/User-Defined Functions/MATLAB Function', 'Sequence');
    chart = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', [model '/Sequence']);
    chart.Script = sprintf([ ...
        'function [vRef, headRef, state] = fcn(t, dist)\n' ...
        '%% as SequenceTwoSteps() in basebot_6.ino, MISSION_MODE 2\n' ...
        'persistent stopped\n' ...
        'if isempty(stopped), stopped = false; end\n' ...
        'side = min(4, floor(dist / 0.75));\n' ...
        'stopped = stopped || dist >= 5 * 0.75;\n' ...
        'headRef = side * pi / 2;\n' ...
        'if stopped\n' ...
        '    vRef = 0;  state = 2;  headRef = 4 * pi / 2;\n' ...
        'elseif t <= 0.1\n' ...
        '    vRef = 0;  state = 10;\n' ...
        'else\n' ...
        '    vRef = 0.75;  state = 11 + side;\n' ...
        'end\n']);

    hc = [model '/Heading controller'];
    add_block('built-in/Subsystem', hc);
    buildHeadingController(hc);
    b('simulink/Math Operations/Gain', 'b_div_2', 'Gain', 'B/2');
    b('simulink/Math Operations/Sum', 'MixLeft', 'Inputs', '+-');
    b('simulink/Math Operations/Sum', 'MixRight', 'Inputs', '++');
    add_block('built-in/Subsystem', [model '/Robot model']);
    buildRobotModel([model '/Robot model']);
    b('simulink/Signal Routing/Mux', 'Mux', 'Inputs', '11');
    b('simulink/Sinks/To Workspace', 'robot', 'VariableName', 'robot', 'SaveFormat', 'Timeseries');
    b('simulink/Sinks/Scope', 'scope', 'NumInputPorts', '1');

    c('Clock/1', 'ZOHt/1');         c('ZOHt/1', 'Sequence/1');
    c('Robot model/3', 'ZOHd/1');   c('ZOHd/1', 'Sequence/2');
    c('Sequence/2', 'Heading controller/1');
    c('Robot model/4', 'Heading controller/2');
    c('Robot model/2', 'Heading controller/3');
    c('Heading controller/1', 'b_div_2/1');
    c('Sequence/1', 'MixLeft/1');   c('b_div_2/1', 'MixLeft/2');
    c('Sequence/1', 'MixRight/1');  c('b_div_2/1', 'MixRight/2');
    c('MixLeft/1', 'Robot model/1');   c('MixRight/1', 'Robot model/2');
    c('Sequence/3', 'Mux/1');  c('Sequence/1', 'Mux/2');  c('Sequence/2', 'Mux/3');
    for k = 1:6, c(sprintf('Robot model/%d', k), sprintf('Mux/%d', k + 3)); end
    c('MixLeft/1', 'Mux/10');  c('MixRight/1', 'Mux/11');
    c('Mux/1', 'robot/1');     c('Mux/1', 'scope/1');

    % log the motor voltages, to see if the 9 V limit is reached
    logs = {'Left', 'uLeft'; 'Right', 'uRight'};
    for k = 1:2
        ph = get_param([model '/Robot model/' logs{k,1} ' Motor + velocity control/Controller'], 'PortHandles');
        set_param(ph.Outport, 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', logs{k,2});
    end
    Simulink.BlockDiagram.arrangeSystem(model);
    save_system(model, fullfile(here, [model '.slx']));
end

function buildHeadingController(sys)
    % as controlUpdate() in the firmware. Uh is the slide's linearization point.
    % ponytail: no turn integrator hold at wheel saturation, add if peak voltage hits 9 V
    b = @(lib, name, varargin) add_block(lib, [sys '/' name], varargin{:});
    c = @(src, dst) add_line(sys, src, dst, 'autorouting', 'on');
    b('built-in/Inport', 'ref');
    b('built-in/Inport', 'heading');
    b('built-in/Inport', 'turnrate');
    b('simulink/Discrete/Zero-Order Hold', 'ADh', 'SampleTime', 'Ts');
    b('simulink/Discrete/Zero-Order Hold', 'ADt', 'SampleTime', 'Ts');
    b('simulink/Math Operations/Sum', 'ErrH', 'Inputs', '+-');
    b('simulink/Math Operations/Gain', 'Kph', 'Gain', 'Kph');
    b('simulink/Discontinuities/Saturation', 'TurnLimit', 'UpperLimit', 'maxTurn', 'LowerLimit', '-maxTurn');
    b('simulink/Math Operations/Sum', 'ErrT', 'Inputs', '+-');
    b('simulink/Discrete/Discrete PID Controller', 'TurnPI', 'Controller', 'PI', ...
      'P', 'Kpt', 'I', 'Kit', 'SampleTime', 'Ts', 'IntegratorMethod', 'Trapezoidal');
    b('simulink/Math Operations/Sum', 'FeedForward', 'Inputs', '++');
    b('simulink/Math Operations/Gain', 'Uh', 'Gain', '1');
    b('built-in/Outport', 'u_head');
    c('heading/1', 'ADh/1');      c('turnrate/1', 'ADt/1');
    c('ref/1', 'ErrH/1');         c('ADh/1', 'ErrH/2');
    c('ErrH/1', 'Kph/1');         c('Kph/1', 'TurnLimit/1');
    c('TurnLimit/1', 'ErrT/1');   c('ADt/1', 'ErrT/2');
    c('ErrT/1', 'TurnPI/1');
    c('TurnLimit/1', 'FeedForward/1');  c('TurnPI/1', 'FeedForward/2');
    c('FeedForward/1', 'Uh/1');   c('Uh/1', 'u_head/1');
    Simulink.BlockDiagram.arrangeSystem(sys);
end
