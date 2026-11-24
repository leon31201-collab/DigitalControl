function buildRobotModel(sys)
    % slide 6: robot model with 2 inputs (left/right wheel ref, m/s)
    b = @(lib, name, varargin) add_block(lib, [sys '/' name], varargin{:});
    c = @(src, dst) add_line(sys, src, dst, 'autorouting', 'on');
    b('built-in/Inport', 'ref left');
    b('built-in/Inport', 'ref right');
    for side = {'Left', 'Right'}
        m = [sys '/' side{1} ' Motor + velocity control'];
        add_block('built-in/Subsystem', m);
        buildMotor(m);
    end
    % slide 4-5: pose
    b('simulink/Math Operations/Sum', 'sumV', 'Inputs', '++');
    b('simulink/Math Operations/Gain', 'half', 'Gain', '1/2');
    b('simulink/Math Operations/Sum', 'diffV', 'Inputs', '-+');
    b('simulink/Math Operations/Gain', 'invB', 'Gain', '1/B');
    b('simulink/Continuous/Integrator', 'distance (m)');
    b('simulink/Continuous/Integrator', 'heading (rad)');
    b('simulink/Math Operations/Trigonometric Function', 'sin', 'Operator', 'sin');
    b('simulink/Math Operations/Trigonometric Function', 'cos', 'Operator', 'cos');
    b('simulink/Math Operations/Product', 'vsin');
    b('simulink/Math Operations/Product', 'vcos');
    b('simulink/Continuous/Integrator', 'y (m)');
    b('simulink/Continuous/Integrator', 'x (m)');
    outs = {'vel', 'turnrate', 'distance', 'heading', 'x', 'y'};
    for k = 1:numel(outs), b('built-in/Outport', outs{k}); end
    c('ref left/1', 'Left Motor + velocity control/1');
    c('ref right/1', 'Right Motor + velocity control/1');
    c('Left Motor + velocity control/1', 'sumV/1');   c('Right Motor + velocity control/1', 'sumV/2');
    c('Left Motor + velocity control/1', 'diffV/1');  c('Right Motor + velocity control/1', 'diffV/2');
    c('sumV/1', 'half/1');           c('diffV/1', 'invB/1');
    c('half/1', 'distance (m)/1');   c('invB/1', 'heading (rad)/1');
    c('heading (rad)/1', 'sin/1');  c('heading (rad)/1', 'cos/1');
    c('half/1', 'vsin/1');  c('sin/1', 'vsin/2');  c('vsin/1', 'y (m)/1');
    c('half/1', 'vcos/1');  c('cos/1', 'vcos/2');  c('vcos/1', 'x (m)/1');
    c('half/1', 'vel/1');            c('invB/1', 'turnrate/1');
    c('distance (m)/1', 'distance/1');    c('heading (rad)/1', 'heading/1');
    c('x (m)/1', 'x/1');                  c('y (m)/1', 'y/1');
    Simulink.BlockDiagram.arrangeSystem(sys);
end
