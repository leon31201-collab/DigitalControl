% Lecture 5, sampling frequency exercise (slides 7-9), MATLAB part.
% Week-2 motor model with a P-controller and a ZOH on each side of it:
%   ref (m/s) -> gear/r -> Err -> Kp -> saturation -> D/A (ZOH)
%     -> motor -> measurement delay -> A/D (ZOH) -> back to Err
% No integrator (slide 8). Wheels up: J is motor + gear + wheel, no robot
% mass (slide 9, step 3). The controller works in motor rad/s as in week 2,
% so Kp is in V/(rad/s).
here = fileparts(mfilename('fullpath'));

load(fullfile(here, 'motorParams.mat'), 'R', 'L', 'Kb', 'J', 'D');
K    = Kb;
num  = K;
den  = conv([L R], [J D]) + [0 0 K^2];       % V -> motor rad/s
gear = 9.6;
r    = 0.0315;       % m (slide: 3 cm)
Vmax = 9;            % V, motor voltage limit in the firmware

% Change these two to compare the readings of the exercise.
% Wheels up with T = 10 ms, Kp must be below ~0.045 to be stable at all.
Kp = 0.05;           % V/(rad/s). Lecture value; ours from week 2 is 0.13
T  = 0.010;          % velocity measurement delay, lecture's "worst case" 10 ms

model = 'basebot_model_sampling';
buildSamplingModel(model, here);

%% sampling time - the slide's plot for each sample time (steps 4-6)
fprintf('\nKp = %g V/(rad/s), delay %g ms.  Ref 0.1 m/s\n', Kp, T*1e3);
fprintf(' Ts (ms)   end value (m/s)   peak (m/s)   p-p in last 0.1s (m/s)\n');
for Ts = [1 2 5 10 20] * 1e-3
    aaa = sim(model, 0.6);
    h = figure(171); clf
    plot(aaa.velP.Time, aaa.velP.Data(:,1), LineWidth=2);
    hold on
    plot(aaa.velP.Time, aaa.velP.Data(:,2), LineWidth=2);
    plot(aaa.velP.Time, aaa.velP.Data(:,3), LineWidth=2);
    legend('ref (m/s)', 'Motor voltage/10 (V)', 'Velocity (m/s)', 'Location', 'north')
    title(sprintf("Sampling: Ts=%.3f sec, Kp=%.3f", Ts, Kp))
    xlabel('Time (sec)')
    ylabel('Velocity (m/s)')
    grid on
    saveas(h, fullfile(here, 'figures', sprintf('sampling_time_%gms_step.png', Ts*1e3)))

    t = aaa.velP.Time;  v = aaa.velP.Data(:,3);  last = v(t > 0.5);
    fprintf('%7g   %15.4f   %10.4f   %22.4f\n', Ts*1e3, v(end), max(v), max(last) - min(last));
end

%% ---------------------------------------------------------------------
function buildSamplingModel(model, here)
    if bdIsLoaded(model), close_system(model, 0); end
    new_system(model);
    set_param(model, 'MaxStep', '1e-4');
    b = @(lib, name, varargin) add_block(lib, [model '/' name], varargin{:});
    b('simulink/Sources/Step', 'Ref', 'Time', '0.2', 'Before', '0', 'After', '0.1');
    b('simulink/Math Operations/Gain', 'toMotor', 'Gain', 'gear/r');      % m/s -> rad/s
    b('simulink/Math Operations/Sum', 'Err', 'Inputs', '+-');
    b('simulink/Math Operations/Gain', 'Kp', 'Gain', 'Kp');
    b('simulink/Discontinuities/Saturation', 'Saturation', ...
      'UpperLimit', 'Vmax', 'LowerLimit', '-Vmax');
    b('simulink/Discrete/Zero-Order Hold', 'DA', 'SampleTime', 'Ts');
    b('simulink/Continuous/Transfer Fcn', 'Motor', 'Numerator', 'num', 'Denominator', 'den');
    b('simulink/Continuous/Transport Delay', 'Delay', 'DelayTime', 'T');
    b('simulink/Discrete/Zero-Order Hold', 'AD', 'SampleTime', 'Ts');
    % digital side, for the plot: ref, voltage/10, measured velocity in m/s
    b('simulink/Math Operations/Gain', 'Vdiv10', 'Gain', '1/10');
    b('simulink/Math Operations/Gain', 'toMs', 'Gain', 'r/gear');         % rad/s -> m/s
    b('simulink/Signal Routing/Mux', 'Mux', 'Inputs', '3');
    b('simulink/Sinks/To Workspace', 'velP', 'VariableName', 'velP', 'SaveFormat', 'Timeseries');
    c = @(src, dst) add_line(model, src, dst, 'autorouting', 'on');
    c('Ref/1', 'toMotor/1');       c('toMotor/1', 'Err/1');     c('Err/1', 'Kp/1');
    c('Kp/1', 'Saturation/1');     c('Saturation/1', 'DA/1');   c('DA/1', 'Motor/1');
    c('Motor/1', 'Delay/1');       c('Delay/1', 'AD/1');        c('AD/1', 'Err/2');
    c('Ref/1', 'Mux/1');           c('DA/1', 'Vdiv10/1');       c('Vdiv10/1', 'Mux/2');
    c('AD/1', 'toMs/1');           c('toMs/1', 'Mux/3');        c('Mux/1', 'velP/1');
    Simulink.BlockDiagram.arrangeSystem(model);
    save_system(model, fullfile(here, [model '.slx']));
end
