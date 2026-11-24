function buildMotor(sys)
    % lecture 5 loop: ref (m/s) -> gear/r -> discrete PI (Ts, ZOH output,
    % saturation + clamping) -> motor -> 10 ms measurement delay -> back.
    % Ki = 0 gives the plain P-controller.
    b = @(lib, name, varargin) add_block(lib, [sys '/' name], varargin{:});
    c = @(src, dst) add_line(sys, src, dst, 'autorouting', 'on');
    b('built-in/Inport', 'ref');
    b('simulink/Math Operations/Gain', 'toMotor', 'Gain', 'gear/r');
    b('simulink/Math Operations/Sum', 'Err', 'Inputs', '+-');
    b('simulink/Discrete/Discrete PID Controller', 'Controller', 'Controller', 'PI', ...
      'P', 'Kp', 'I', 'Ki', 'SampleTime', 'Ts', 'LimitOutput', 'on', ...
      'UpperSaturationLimit', 'Vmax', 'LowerSaturationLimit', '-Vmax', 'AntiWindupMode', 'clamping');
    b('simulink/Continuous/Transfer Fcn', 'Motor', 'Numerator', 'num', 'Denominator', 'den');
    b('simulink/Continuous/Transport Delay', 'Delay', 'DelayTime', 'T');
    b('simulink/Math Operations/Gain', 'toMs', 'Gain', 'r/gear');
    b('built-in/Outport', 'velocity');
    c('ref/1', 'toMotor/1');  c('toMotor/1', 'Err/1');    c('Err/1', 'Controller/1');
    c('Controller/1', 'Motor/1');   c('Motor/1', 'Delay/1');    c('Delay/1', 'Err/2');
    c('Motor/1', 'toMs/1');         c('toMs/1', 'velocity/1');
    Simulink.BlockDiagram.arrangeSystem(sys);
end
