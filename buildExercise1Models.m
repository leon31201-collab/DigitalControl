% buildExercise1Models.m
% DTU 34765 Digital Control - Week 6: Full Robot Control
%
% This script builds and runs:
%   Exercise 1a: basebot_drive.slx (2-motor differential drive with pose calculation)
%   Exercise 1b: basebot_velocity_turnrate.slx (velocity & turn rate inverse kinematics)
%
% Generates verification plots for both exercises.

clear;
close all;

%% 1. Load Parameters
if exist('piDesign.mat', 'file')
    load('piDesign.mat', 'num', 'den', 'Kp', 'T', 'Vmax');
else
    error('piDesign.mat nicht gefunden. Bitte zuerst piDesign.m ausfuehren.');
end

% Basebot Geometry & Physical Parameters
gear        = 9.6;            % Gear ratio
wrad        = 0.0315;         % Wheel radius (m)
B           = 0.15;           % Wheel base (m, as on slide 4)
limit_9V    = 9.0;            % Motor voltage limit (V)
Ts          = 0.001;          % Sample time 1 ms (D/A ZOH)

if ~exist('figures', 'dir'), mkdir('figures'); end

%% ========================================================================
%% 2. Exercise 1a: Build basebot_drive.slx
%% ========================================================================
model1 = 'basebot_drive';
if bdIsLoaded(model1), close_system(model1, 0); end
new_system(model1);
initCmd = 'if exist(''piDesign.mat'',''file''), load(''piDesign.mat''); end; gear=9.6; wrad=0.0315; B=0.15; limit_9V=9; Ts=0.001;';
set_param(model1, 'SaveOutput', 'on', 'SaveFormat', 'Dataset', ...
                  'StopTime', '1.5', 'Solver', 'ode45', ...
                  'InitFcn', initCmd);

% Helper lambda to add blocks
b1 = @(lib, name, varargin) add_block(lib, [model1 '/' name], varargin{:});
c1 = @(src, dst) add_line(model1, src, dst, 'autorouting', 'on');

% --- Single Motor Subsystem Builder Function ---
buildMotorSubsystem(model1, 'Left Motor + velocity control', num, den, Kp, limit_9V, Ts, gear, wrad);
buildMotorSubsystem(model1, 'Right Motor + velocity control', num, den, Kp, limit_9V, Ts, gear, wrad);

% Set Subsystem positions
set_param([model1 '/Left Motor + velocity control'], 'Position', [250, 100, 390, 160]);
set_param([model1 '/Right Motor + velocity control'], 'Position', [250, 220, 390, 280]);

% --- Input Source: Velocity Step (m/s) ---
b1('simulink/Sources/Step', 'Velocity Step (m//s)', ...
   'Time', '0.1', 'Before', '0', 'After', '0.4', 'Position', [80, 175, 120, 205]);

% --- Forward Velocity: v = (vR + vL) / 2 ---
b1('simulink/Math Operations/Sum', 'Sum_v', 'Inputs', '++', 'Position', [450, 120, 470, 150]);
b1('simulink/Math Operations/Gain', 'Gain_half', 'Gain', '1/2', 'Position', [500, 122, 535, 148]);

% --- Turn Rate: hdot = (vR - vL) / B ---
% Top input: -, Bottom input: + (vR - vL)
b1('simulink/Math Operations/Sum', 'Sum_turn', 'Inputs', '-+', 'Position', [450, 230, 470, 260]);
b1('simulink/Math Operations/Gain', 'Gain_invB', 'Gain', '1/B', 'Position', [500, 232, 545, 258]);

% --- Integrators for Distance and Heading ---
b1('simulink/Continuous/Integrator', 'Int_dist', 'Position', [580, 60, 610, 90]);
b1('simulink/Continuous/Integrator', 'Int_head', 'Position', [580, 170, 610, 200]);

% --- Kinematics: xdot = v*cos(h), ydot = v*sin(h) ---
b1('simulink/Math Operations/Trigonometric Function', 'Trig_sin', 'Function', 'sin', 'Position', [660, 260, 690, 290]);
b1('simulink/Math Operations/Trigonometric Function', 'Trig_cos', 'Function', 'cos', 'Position', [720, 260, 750, 290]);

b1('simulink/Math Operations/Product', 'Prod_xdot', 'Inputs', '**', 'Position', [725, 330, 755, 360]);
b1('simulink/Math Operations/Product', 'Prod_ydot', 'Inputs', '**', 'Position', [665, 330, 695, 360]);

b1('simulink/Continuous/Integrator', 'Int_x', 'Position', [725, 390, 755, 420]);
b1('simulink/Continuous/Integrator', 'Int_y', 'Position', [665, 390, 695, 420]);

% --- Mux for out.pose (vel, turnrate, dist, heading, x, y) ---
b1('simulink/Signal Routing/Mux', 'Mux_pose', 'Inputs', '6', 'Position', [600, 460, 605, 540]);
b1('simulink/Sinks/To Workspace', 'out_pose', 'VariableName', 'out_pose', ...
   'SaveFormat', 'Timeseries', 'Position', [480, 485, 550, 515]);
set_param([model1 '/out_pose'], 'Orientation', 'left');

% --- Scope pose ---
b1('simulink/Sinks/Scope', 'pose', 'Position', [550, 400, 580, 430]);
set_param([model1 '/pose'], 'Orientation', 'up');

% --- Outports ---
b1('simulink/Sinks/Out1', 'velocity', 'Port', '1', 'Position', [820, 128, 850, 142]);
b1('simulink/Sinks/Out1', 'Turnrate', 'Port', '2', 'Position', [820, 238, 850, 252]);
b1('simulink/Sinks/Out1', 'distance', 'Port', '3', 'Position', [820, 68, 850, 82]);
b1('simulink/Sinks/Out1', 'heading',  'Port', '4', 'Position', [820, 178, 850, 192]);

% --- Signal Connections ---
c1('Velocity Step (m//s)/1', 'Left Motor + velocity control/1');
c1('Velocity Step (m//s)/1', 'Right Motor + velocity control/1');

c1('Left Motor + velocity control/1', 'Sum_v/1');
c1('Right Motor + velocity control/1', 'Sum_v/2');
c1('Sum_v/1', 'Gain_half/1');

c1('Left Motor + velocity control/1', 'Sum_turn/1');
c1('Right Motor + velocity control/1', 'Sum_turn/2');
c1('Sum_turn/1', 'Gain_invB/1');

c1('Gain_half/1', 'Int_dist/1');
c1('Gain_invB/1', 'Int_head/1');

% Trigonometric heading connections
c1('Int_head/1', 'Trig_sin/1');
c1('Int_head/1', 'Trig_cos/1');

% Products with forward velocity
c1('Gain_half/1', 'Prod_xdot/1');
c1('Trig_cos/1', 'Prod_xdot/2');
c1('Prod_xdot/1', 'Int_x/1');

c1('Gain_half/1', 'Prod_ydot/1');
c1('Trig_sin/1', 'Prod_ydot/2');
c1('Prod_ydot/1', 'Int_y/1');

% Outports
c1('Gain_half/1', 'velocity/1');
c1('Gain_invB/1', 'Turnrate/1');
c1('Int_dist/1', 'distance/1');
c1('Int_head/1', 'heading/1');

% Mux connections
c1('Gain_half/1', 'Mux_pose/1');
c1('Gain_invB/1', 'Mux_pose/2');
c1('Int_dist/1', 'Mux_pose/3');
c1('Int_head/1', 'Mux_pose/4');
c1('Int_x/1', 'Mux_pose/5');
c1('Int_y/1', 'Mux_pose/6');

c1('Mux_pose/1', 'out_pose/1');
c1('Mux_pose/1', 'pose/1');

Simulink.BlockDiagram.arrangeSystem(model1);
save_system(model1);
fprintf('Modell "%s.slx" (Exercise 1a) erfolgreich erstellt.\n', model1);

% Run Simulation Exercise 1a
sim1 = sim(model1);
t1 = sim1.out_pose.Time;
d1 = sim1.out_pose.Data;

fig1 = figure(1); clf;
set(fig1, 'Name', 'Exercise 1a: Pose Verification', 'Color', 'w');
subplot(2,2,1);
plot(t1, d1(:,1), 'b', 'LineWidth', 1.5); grid on;
ylabel('Velocity (m/s)'); title('Forward Velocity v(t)');
subplot(2,2,2);
plot(t1, d1(:,2), 'r', 'LineWidth', 1.5); grid on;
ylabel('Turnrate (rad/s)'); title('Turn Rate \omega_z(t)');
subplot(2,2,3);
plot(t1, d1(:,3), 'g', 'LineWidth', 1.5); grid on;
ylabel('Distance (m)'); xlabel('Time (s)'); title('Distance traveled');
subplot(2,2,4);
plot(t1, d1(:,5), 'm', 'LineWidth', 1.5); grid on;
ylabel('x Position (m)'); xlabel('Time (s)'); title('x-Position');
sgtitle('Exercise 1a: Basebot Pose Simulation (Straight Drive)');
saveas(fig1, fullfile('figures', 'exercise1a_pose.png'));
fprintf('Plot gespeichert: figures/exercise1a_pose.png\n');


%% ========================================================================
%% 3. Exercise 1b: Build basebot_velocity_turnrate.slx
%% ========================================================================
model2 = 'basebot_velocity_turnrate';
if bdIsLoaded(model2), close_system(model2, 0); end
new_system(model2);
set_param(model2, 'SaveOutput', 'on', 'SaveFormat', 'Dataset', ...
                  'StopTime', '1.5', 'Solver', 'ode45', ...
                  'InitFcn', initCmd);

b2 = @(lib, name, varargin) add_block(lib, [model2 '/' name], varargin{:});
c2 = @(src, dst) add_line(model2, src, dst, 'autorouting', 'on');

% --- Add Robot model Subsystem (Containing the 2-Motor + Pose model) ---
subRobot = [model2 '/Robot model'];
add_block('simulink/Ports & Subsystems/Subsystem', subRobot);
Simulink.SubSystem.deleteContents(subRobot);

% Inside Robot model Subsystem:
bRob = @(lib, name, varargin) add_block(lib, [subRobot '/' name], varargin{:});
cRob = @(src, dst) add_line(subRobot, src, dst, 'autorouting', 'on');

% Inports inside Robot model
bRob('simulink/Sources/In1', 'ref (m//s)', 'Position', [50, 100, 80, 115]);
bRob('simulink/Sources/In1', 'ref (m//s)1', 'Position', [50, 220, 80, 235]);

% Motors inside Robot model
buildMotorSubsystem(subRobot, 'Left Motor + velocity control', num, den, Kp, limit_9V, Ts, gear, wrad);
buildMotorSubsystem(subRobot, 'Right Motor + velocity control', num, den, Kp, limit_9V, Ts, gear, wrad);
set_param([subRobot '/Left Motor + velocity control'], 'Position', [150, 80, 290, 140]);
set_param([subRobot '/Right Motor + velocity control'], 'Position', [150, 200, 290, 260]);

% Kinematics inside Robot model
bRob('simulink/Math Operations/Sum', 'Sum_v', 'Inputs', '++', 'Position', [350, 100, 370, 130]);
bRob('simulink/Math Operations/Gain', 'Gain_half', 'Gain', '1/2', 'Position', [400, 102, 435, 128]);

bRob('simulink/Math Operations/Sum', 'Sum_turn', 'Inputs', '-+', 'Position', [350, 210, 370, 240]);
bRob('simulink/Math Operations/Gain', 'Gain_invB', 'Gain', '1/B', 'Position', [400, 212, 445, 238]);

bRob('simulink/Continuous/Integrator', 'Int_dist', 'Position', [480, 50, 510, 80]);
bRob('simulink/Continuous/Integrator', 'Int_head', 'Position', [480, 160, 510, 190]);

% Position (x, y)
bRob('simulink/Math Operations/Trigonometric Function', 'Trig_sin', 'Function', 'sin', 'Position', [560, 240, 590, 270]);
bRob('simulink/Math Operations/Trigonometric Function', 'Trig_cos', 'Function', 'cos', 'Position', [620, 240, 650, 270]);
bRob('simulink/Math Operations/Product', 'Prod_xdot', 'Inputs', '**', 'Position', [625, 310, 655, 340]);
bRob('simulink/Math Operations/Product', 'Prod_ydot', 'Inputs', '**', 'Position', [565, 310, 595, 340]);
bRob('simulink/Continuous/Integrator', 'Int_x', 'Position', [625, 370, 655, 400]);
bRob('simulink/Continuous/Integrator', 'Int_y', 'Position', [565, 370, 595, 400]);

% Outports of Robot model Subsystem
bRob('simulink/Sinks/Out1', 'distance (m)', 'Port', '1', 'Position', [720, 58, 750, 72]);
bRob('simulink/Sinks/Out1', 'vel (m//s)',     'Port', '2', 'Position', [720, 108, 750, 122]);
bRob('simulink/Sinks/Out1', 'heading (rad)', 'Port', '3', 'Position', [720, 168, 750, 182]);
bRob('simulink/Sinks/Out1', 'turnrate (rad//s)', 'Port', '4', 'Position', [720, 218, 750, 232]);
bRob('simulink/Sinks/Out1', 'x (m)', 'Port', '5', 'Position', [720, 378, 750, 392]);
bRob('simulink/Sinks/Out1', 'y (m)', 'Port', '6', 'Position', [720, 428, 750, 442]);

% Internal connections of Robot model
cRob('ref (m//s)/1', 'Left Motor + velocity control/1');
cRob('ref (m//s)1/1', 'Right Motor + velocity control/1');

cRob('Left Motor + velocity control/1', 'Sum_v/1');
cRob('Right Motor + velocity control/1', 'Sum_v/2');
cRob('Sum_v/1', 'Gain_half/1');

cRob('Left Motor + velocity control/1', 'Sum_turn/1');
cRob('Right Motor + velocity control/1', 'Sum_turn/2');
cRob('Sum_turn/1', 'Gain_invB/1');

cRob('Gain_half/1', 'Int_dist/1');
cRob('Gain_invB/1', 'Int_head/1');

cRob('Int_head/1', 'Trig_sin/1');
cRob('Int_head/1', 'Trig_cos/1');
cRob('Gain_half/1', 'Prod_xdot/1');
cRob('Trig_cos/1', 'Prod_xdot/2');
cRob('Prod_xdot/1', 'Int_x/1');

cRob('Gain_half/1', 'Prod_ydot/1');
cRob('Trig_sin/1', 'Prod_ydot/2');
cRob('Prod_ydot/1', 'Int_y/1');

cRob('Int_dist/1', 'distance (m)/1');
cRob('Gain_half/1', 'vel (m//s)/1');
cRob('Int_head/1', 'heading (rad)/1');
cRob('Gain_invB/1', 'turnrate (rad//s)/1');
cRob('Int_x/1', 'x (m)/1');
cRob('Int_y/1', 'y (m)/1');

% --- Outer Level of basebot_velocity_turnrate ---
set_param(subRobot, 'Position', [380, 100, 520, 250]);

% Inputs for Exercise 1b (Slide 7):
% Velocity: 0.6 m/s after 0.1 sec
b2('simulink/Sources/Step', 'Velocity Step (m//s)', ...
   'Time', '0.1', 'Before', '0', 'After', '0.6', 'Position', [80, 105, 120, 135]);

% Turnrate: 1 rad/s after 0.5 sec
b2('simulink/Sources/Step', 'Turnrate Step (rad//s)', ...
   'Time', '0.5', 'Before', '0', 'After', '1.0', 'Position', [80, 195, 120, 225]);

% Gain B/2
b2('simulink/Math Operations/Gain', 'Gain_Bhalf', 'Gain', 'B/2', 'Position', [170, 197, 210, 223]);

% Inverse Differential Drive Kinematics:
% Left wheel:  vL_ref = v_ref - (B/2)*w_ref  -> Sum (+-)
b2('simulink/Math Operations/Sum', 'Sum_vL', 'Inputs', '+-', 'Position', [260, 115, 280, 145]);

% Right wheel: vR_ref = v_ref + (B/2)*w_ref  -> Sum (++)
b2('simulink/Math Operations/Sum', 'Sum_vR', 'Inputs', '++', 'Position', [260, 195, 280, 225]);

% Outer connections
c2('Velocity Step (m//s)/1', 'Sum_vL/1');
c2('Velocity Step (m//s)/1', 'Sum_vR/1');

c2('Turnrate Step (rad//s)/1', 'Gain_Bhalf/1');
c2('Gain_Bhalf/1', 'Sum_vL/2');
c2('Gain_Bhalf/1', 'Sum_vR/2');

c2('Sum_vL/1', 'Robot model/1');
c2('Sum_vR/1', 'Robot model/2');

% Scope and To Workspace
b2('simulink/Signal Routing/Mux', 'Mux_robot', 'Inputs', '6', 'Position', [600, 95, 605, 245]);
b2('simulink/Sinks/Scope', 'robot', 'Position', [650, 140, 680, 170]);
b2('simulink/Sinks/To Workspace', 'out_robot', 'VariableName', 'out_robot', ...
   'SaveFormat', 'Timeseries', 'Position', [650, 185, 720, 215]);

% Connect to Mux: [v_ref, w_ref, dist, vel, heading, turnrate]
c2('Velocity Step (m//s)/1', 'Mux_robot/1');
c2('Turnrate Step (rad//s)/1', 'Mux_robot/2');
c2('Robot model/1', 'Mux_robot/3');
c2('Robot model/2', 'Mux_robot/4');
c2('Robot model/3', 'Mux_robot/5');
c2('Robot model/4', 'Mux_robot/6');

c2('Mux_robot/1', 'robot/1');
c2('Mux_robot/1', 'out_robot/1');

% XY position to Workspace
b2('simulink/Signal Routing/Mux', 'Mux_xy', 'Inputs', '2', 'Position', [600, 280, 605, 330]);
b2('simulink/Sinks/To Workspace', 'out_xy', 'VariableName', 'out_xy', ...
   'SaveFormat', 'Timeseries', 'Position', [650, 290, 720, 320]);
c2('Robot model/5', 'Mux_xy/1');
c2('Robot model/6', 'Mux_xy/2');
c2('Mux_xy/1', 'out_xy/1');

Simulink.BlockDiagram.arrangeSystem(model2);
save_system(model2);
fprintf('Modell "%s.slx" (Exercise 1b) erfolgreich erstellt.\n', model2);

% Run Simulation Exercise 1b
sim2 = sim(model2);
t2 = sim2.out_robot.Time;
d2 = sim2.out_robot.Data;
xy = sim2.out_xy.Data;

% Plot Exercise 1b: Tracking
fig2 = figure(2); clf;
set(fig2, 'Name', 'Exercise 1b: Velocity & Turnrate Tracking', 'Color', 'w');
subplot(2,2,1);
plot(t2, d2(:,1), 'k--', t2, d2(:,4), 'b-', 'LineWidth', 1.5); grid on;
ylabel('Velocity (m/s)'); xlabel('Time (s)');
legend({'Reference v_{ref}', 'Actual v'}, 'Location', 'best');
title('Forward Velocity Tracking');

subplot(2,2,2);
plot(t2, d2(:,2), 'k--', t2, d2(:,6), 'r-', 'LineWidth', 1.5); grid on;
ylabel('Turnrate (rad/s)'); xlabel('Time (s)');
legend({'Reference \omega_{ref}', 'Actual \omega'}, 'Location', 'best');
title('Turn Rate Tracking');

subplot(2,2,3);
plot(t2, d2(:,3), 'g', 'LineWidth', 1.5); grid on;
ylabel('Distance (m)'); xlabel('Time (s)');
title('Distance Traveled');

subplot(2,2,4);
plot(t2, d2(:,5), 'm', 'LineWidth', 1.5); grid on;
ylabel('Heading (rad)'); xlabel('Time (s)');
title('Heading Angle h(t)');

sgtitle('Exercise 1b: Robot Model (Velocity & Turnrate Input)');
saveas(fig2, fullfile('figures', 'exercise1b_velocity_turnrate.png'));
fprintf('Plot gespeichert: figures/exercise1b_velocity_turnrate.png\n');

% Plot Exercise 1b: 2D Trajectory
fig3 = figure(3); clf;
set(fig3, 'Name', 'Exercise 1b: 2D Trajectory (x,y)', 'Color', 'w');
plot(xy(:,1), xy(:,2), 'b-', 'LineWidth', 2); grid on; axis equal;
hold on;
plot(xy(1,1), xy(1,2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot(xy(end,1), xy(end,2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
xlabel('x Position (m)'); ylabel('y Position (m)');
title('Exercise 1b: 2D Robot Trajectory (0.6 m/s, turn 1 rad/s at 0.5s)');
legend({'Trajectory', 'Start', 'End'}, 'Location', 'best');
saveas(fig3, fullfile('figures', 'exercise1b_trajectory_xy.png'));
fprintf('Plot gespeichert: figures/exercise1b_trajectory_xy.png\n');

%% ========================================================================
%% 4. Print Answers to Lecture Questions
%% ========================================================================
fprintf('\n======================================================\n');
fprintf('        ANTWORTEN ZU DEN FRAGEN AUS EXERCISE 1b        \n');
fprintf('======================================================\n');
fprintf('1. "Do we turn as fast as expected?"\n');
fprintf('   -> Nein, die Drehrate springt nicht instantan auf 1 rad/s,\n');
fprintf('      sondern folgt mit einer Verzoegerung erster Ordnung (ca. 40-70 ms).\n');
fprintf('      Ursache: Mechanische Traegheit des Roboters und Motordynamik\n');
fprintf('      sowie die begrenzte Motorspannung (9V Saettigung).\n');
fprintf('      Dadurch hinkt auch der Heading-Winkel (Orientierung) hinterher.\n\n');
fprintf('2. "Consider improvements":\n');
fprintf('   -> a) Heading-Regelung: Ein ueberlagerter Regelkreis fuer den\n');
fprintf('         Kurswinkel h (z.B. mit Gyro/IMU-Rueckfuehrung), um Drift zu verhindern.\n');
fprintf('   -> b) Ruckbegrenzung (Rate Limiter): Sanftere Sollwert-Rampen verhindern\n');
fprintf('         starke Stromspitzen und Radschlupf beim Einlenken.\n');
fprintf('   -> c) Saettigungsmanagement: Bei hoher Vorwaertsgeschwindigkeit\n');
fprintf('         kann das kurvenaeussere Rad in die 9V-Saettigung laufen.\n');
fprintf('======================================================\n\n');

%% ========================================================================
%% Helper Function: Build Motor Subsystem (Slide 3)
%% ========================================================================
function buildMotorSubsystem(parent, subName, num, den, Kp, limit_9V, Ts, gear, wrad)
    subPath = [parent '/' subName];
    add_block('simulink/Ports & Subsystems/Subsystem', subPath);
    Simulink.SubSystem.deleteContents(subPath);

    bm = @(lib, name, varargin) add_block(lib, [subPath '/' name], varargin{:});
    cm = @(src, dst) add_line(subPath, src, dst, 'autorouting', 'on');

    % Inport: ref (m//s)
    bm('simulink/Sources/In1', 'ref (m//s)', 'Position', [50, 100, 80, 115]);

    % Gain: gear/wrad (converts m/s to motor rad/s)
    bm('simulink/Math Operations/Gain', 'Gain_v2w', 'Gain', 'gear/wrad', 'Position', [120, 95, 170, 125]);

    % Controller: Sum (+-) -> Gain Kp -> Saturation limit_9V
    bm('simulink/Math Operations/Sum', 'Sum_err', 'Inputs', '+-', 'Position', [210, 100, 230, 130]);
    bm('simulink/Math Operations/Gain', 'Gain_Kp', 'Gain', 'Kp', 'Position', [260, 102, 295, 128]);
    bm('simulink/Discontinuities/Saturation', 'limit_9V', ...
       'UpperLimit', num2str(limit_9V), 'LowerLimit', num2str(-limit_9V), ...
       'Position', [325, 100, 355, 130]);

    % D/A (zoh)
    bm('simulink/Discrete/Zero-Order Hold', 'DA_zoh', 'SampleTime', num2str(Ts), ...
       'Position', [385, 98, 420, 132]);

    % Motor model: Transfer Fcn Y(s)/U(s)
    bm('simulink/Continuous/Transfer Fcn', 'Motor_model', ...
       'Numerator', 'num', 'Denominator', 'den', 'Position', [450, 97, 520, 133]);

    % Conversion from motor rad/s back to wheel m/s: 1/gear * wrad = wrad/gear
    bm('simulink/Math Operations/Gain', 'Gain_w2v', 'Gain', 'wrad/gear', 'Position', [550, 100, 600, 130]);

    % Outport: velocity (m//s)
    bm('simulink/Sinks/Out1', 'velocity (m//s)', 'Position', [640, 108, 670, 122]);

    % Connections inside Subsystem
    cm('ref (m//s)/1', 'Gain_v2w/1');
    cm('Gain_v2w/1', 'Sum_err/1');
    cm('Sum_err/1', 'Gain_Kp/1');
    cm('Gain_Kp/1', 'limit_9V/1');
    cm('limit_9V/1', 'DA_zoh/1');
    cm('DA_zoh/1', 'Motor_model/1');
    cm('Motor_model/1', 'Gain_w2v/1');
    cm('Gain_w2v/1', 'velocity (m//s)/1');

    % Feedback: motor speed (rad/s) fed back to controller Sum_err
    cm('Motor_model/1', 'Sum_err/2');
end
