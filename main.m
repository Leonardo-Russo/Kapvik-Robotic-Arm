%% Space Robotic Systems - Project

close all
clear
clc 

addpath('Library/')

% Define the Options
options = struct('name', "Options");
options.show_frames = false;
options.show_manipulator = true;

tic     % necessary to evaluate runtime

global Jsym

%% Define Manipulator Properties

% Define Rover Dimensions
L = 0.7;                % m
w = 0.5;                % m
h = 0.3;                % m

% Define Base Shift along y
P_By = 0.1;             % m

% Define Scoop Dimension
scoopLength = 0.05;      % m

% Other Parameters
a2 = 0.46;              % m
a3 = 0.44;              % m
linkL = 0.18;           % m - link length

% Define Links
UpperArm = link(0.46, 40, 2, 3.5);
ForeArm = link(0.44, 40, 2, 3.5);

% Define Joints
Joint_1 = joint(1.15, 8.4, -160, 100, 1.5*10^(-4), -5*10^(-4), 1.44*10^(-6), 8.67, 0.84, 5.3*10^(-6), 10300, 0.025);
Joint_2 = joint(1.28, 8.4,  -90,  90, 1.5*10^(-4), -5*10^(-4), 1.44*10^(-6), 8.67, 0.84, 5.3*10^(-6), 10300, 0.025);
Joint_3 = joint(1.39, 5.3, -150, 110, 1.5*10^(-4), -5*10^(-4), 1.44*10^(-6), 8.67, 0.84, 5.3*10^(-6), 10300, 0.025);
Joint_4 = joint(0.67, 6.7,  -90,   5, 1.5*10^(-4), -5*10^(-4), 1.44*10^(-6), 8.67, 0.84, 5.3*10^(-6), 10300, 0.025);

% Define d3
d3 = -1.5*10^(-3)*UpperArm.Diameter;

% Define Base Frame
R_B = R3(pi);
P_B = [0 -P_By h+linkL/2]';
T_B2S = buildT(R_B, P_B);
T_S2B = inv_trans(T_B2S);

% Define Tool Frame wrt Wrist Frame
R_T = eye(3);
P_T = [scoopLength 0 0]';
T_T2W = buildT(R_T, P_T);

% Create Station Frame
T_S2S = eye(4);


%% Define MDH Table

syms q1 q2 q3 q4 q1d q2d q3d q4d q1dd q2dd q3dd q4dd g real

global Qsym
Qsym = [q1, q2, q3, q4];

TableMDHsym = define_table(q1, q2, q3, q4, a2, a3, d3);


%% Compute Jacobian Matrix

T_W2Bsym = simplify(dirkine(TableMDHsym));
T_T2Ssym = where_fun(T_S2B, T_W2Bsym, T_T2W);
X_W2Bsym = simplify(trans2pose(T_W2Bsym));
X_Tsym = simplify(trans2pose(T_T2Ssym));

Jsym = simplify(jacobian(X_Tsym, [q1 q2 q3 q4]));


%% Dynamical equations (symbolic expression)

[M, V, G, F] = dinEqs(Joint_1, Joint_2, Joint_3, Joint_4, UpperArm, ForeArm, P_T);


% %% Trajectory Generation (Stowage to Navigation)
% 
% NSto2Nav=10; % number of via points
% TSto2Nav=10; % total time from stowage to Navigation [s]
% ft=100; % path update rate [Hz]
% thetaddMax=deg2rad(40); % [rad/s^2] me la sono inventata sulla base delle slide, va aggiustata
% 
% q0Sto2Nav=[pi/2 -pi/2 -pi/2 -pi/6]';
% % Pose at via points
% % We have to decide X for all via points from Stowage to Navigation
% for i=1:N-1
%     [QSto2Nav(:,i)] = invkinAnal(X(:,i), L1, L2, "SuEuc2p"); % shoulder up + elbow up + cos(q2) positivo
% end
% [tSto2Nav, qSto2Nav, qdSto2Nav, qddSto2Nav] = trajectoryGeneration(q0Sto2Nav, QSto2Nav, thetaddMax, TSto2Nav, NSto2Nav, ft);
% 
% % Plot 
% figure(1)
% plot(t,rad2deg(qSto2Nav(1,:)),'r',lineWidth=1.5)
% hold on
% plot(t,rad2deg(qSto2Nav(2,:)),'b',lineWidth=1.5)
% plot(t,rad2deg(qSto2Nav(3,:)),'g',lineWidth=1.5)
% plot(t,rad2deg(qSto2Nav(4,:)),'m',lineWidth=1.5)
% xlabel('$$t$$ [s]','Interpreter','latex')
% ylabel('$$\theta(t)$$ [^\circ]','Interpreter','latex')
% legend('Joint 1 ($$\theta_1$$)', 'Joint 2 ($$\theta_2$$)', 'Joint 3 ($$\theta_3$$)',...
%     'Joint 4 ($$\theta_45$$)', 'Interpreter','latex')
% title('Joint angles','Interpreter','latex')
% set(gca,'FontSize',20)
% 
% figure(2)
% plot(t,rad2deg(qdSto2Nav(1,:)),'r',lineWidth=1.5)
% hold on
% plot(t,rad2deg(qdSto2Nav(2,:)),'b',lineWidth=1.5)
% plot(t,rad2deg(qdSto2Nav(3,:)),'g',lineWidth=1.5)
% plot(t,rad2deg(qdSto2Nav(4,:)),'m',lineWidth=1.5)
% xlabel('$$t$$ [s]','Interpreter','latex')
% ylabel('$$\dot\theta(t)$$ [^\circ/s]','Interpreter','latex')
% legend('Joint 1 ($$\dot\theta_1$$)', 'Joint 2 ($$\dot\theta_2$$)', 'Joint 3 ($$\dot\theta_3$$)',...
%     'Joint 4 ($$\dot\theta_4$$)', 'Interpreter','latex')
% title('Joint velocities','Interpreter','latex')
% set(gca,'FontSize',20)
% 
% figure(3)
% plot(t,rad2deg(qddSto2Nav(1,:)),'r',lineWidth=1.5)
% hold on
% plot(t,rad2deg(qddSto2Nav(2,:)),'b',lineWidth=1.5)
% plot(t,rad2deg(qddSto2Nav(3,:)),'g',lineWidth=1.5)
% plot(t,rad2deg(qddSto2Nav(4,:)),'m',lineWidth=1.5)
% xlabel('$$t$$ [s]','Interpreter','latex')
% ylabel('$$\ddot\theta(t)$$ [^\circ/s^2]','Interpreter','latex')
% legend('Joint 1 ($$\ddot\theta_1$$)', 'Joint 2 ($$\ddot\theta_2$$)', 'Joint 3 ($$\ddot\theta_3$$)',...
%     'Joint 4 ($$\ddot\theta_4$$)', 'Interpreter','latex')
% title('Joint accelerations','Interpreter','latex')
% set(gca,'FontSize',20)


%% Manipulator Visualization

close all

% Set the Initial Joint Variables
global Q
Q=[pi/2 -pi/2 -pi/2 -pi/6];
% Q = [pi/4 -pi/2 deg2rad(110) 0];

% Get Manipulator State
[T_12S, T_22S, T_32S, T_W2S, T_T2S, X_T] = ...
    getManipulatorState(Q, TableMDHsym, X_Tsym, T_B2S, T_T2W);

% Create the Workspace
figure('name', 'Enviroment Simulation')

% Show the Static Enviroment
env = show_env(L, w, h);

% Show the Manipulator
if options.show_manipulator
    joints = show_joints(T_12S, T_22S, T_32S, T_W2S);
    links = show_links(UpperArm, T_22S, ForeArm, T_32S);
    scoop = show_Scoop(scoopLength, T_T2S);
end

% Show the Frames
if options.show_frames
    mframes = show_mainframes(T_S2S, T_B2S, T_W2S, T_T2S);
    jframes = show_jointframes(T_12S, T_22S, T_32S);
    toggleButton = uicontrol('Style', 'pushbutton', 'String', 'Show Legend', 'Position', [20 20 100 20]);
else
    mframes = [];
    jframes = [];
end

axis equal      % necessary after all plots

% Set Light Options
material([0.5, 0.6, 0.6, 0, 0.2]);  % [ambient, diffuse, specular, shininess, specularExponent]
lgt = light('Position', [2 2 5], 'Style', 'local');
lightangle(150, 40);
lighting gouraud

% Create Frames Legend
if options.show_frames
    lgd = legend([mframes.S.arw1, mframes.B.arw1, mframes.W.arw1, mframes.T.arw1, jframes.J1.arw1, jframes.J2.arw1, jframes.J3.arw1], ...
                 {'Station Frame', 'Base Frame', 'Wrist Frame', 'Tool Frame', ...
                  '1^{st} Joint Frame', '2^{nd} Joint Frame', '3^{rd} Joint Frame'}, ...
                  'location', 'best', 'fontsize', 8);
    
    set(lgd, 'Visible', 'off');     % by default legend is hidden
    set(toggleButton, 'Callback', {@toggleLegendCallback, lgd});    % set the callback for the boring button
end

% Add Boring button
boringButton = uicontrol('Style', 'pushbutton', 'String', 'Boring Button', 'Position', [140 20 100 20]);
set(boringButton, 'Callback', {@strobEffectCallback, lgt});

% Create a Panel to Group Sliders and Labels
panelPosition = [0.65 0.01 0.34 0.36];  % [left bottom width height]
sliderPanel = uipanel('Title', 'Joint Controls', ...
             'FontSize', 10, ...
             'FontWeight', 'bold', ...
             'BackgroundColor', 'white', ...
             'Position', panelPosition, ...   % Position of the panel within the figure
             'Parent', gcf);                  % Set the parent to the current figure

% Define Slider and Label Positions Relative to the Panel
sliderPositions = [80 10 100 20; 80 40 100 20; 80 70 100 20; 80 100 100 20];
labelPositions = [10 5 50 20; 10 35 50 20; 10 65 50 20; 10 95 50 20];

% Create the Sliders
slider_q1 = create_slider('q1', -160, 100, @(src, event) updatePlot(src, event, TableMDHsym, X_Tsym, T_B2S, T_T2W, joints, links, scoop, mframes, jframes, options, UpperArm, ForeArm, scoopLength), sliderPositions(1, :), labelPositions(1, :), sliderPanel);
slider_q2 = create_slider('q2', -90, 90, @(src, event) updatePlot(src, event, TableMDHsym, X_Tsym, T_B2S, T_T2W, joints, links, scoop, mframes, jframes, options, UpperArm, ForeArm, scoopLength), sliderPositions(2, :), labelPositions(2, :), sliderPanel);
slider_q3 = create_slider('q3', -150, 110, @(src, event) updatePlot(src, event, TableMDHsym, X_Tsym, T_B2S, T_T2W, joints, links, scoop, mframes, jframes, options, UpperArm, ForeArm, scoopLength), sliderPositions(3, :), labelPositions(3, :), sliderPanel);
slider_q4 = create_slider('q4', -90, 5, @(src, event) updatePlot(src, event, TableMDHsym, X_Tsym, T_B2S, T_T2W, joints, links, scoop, mframes, jframes, options, UpperArm, ForeArm, scoopLength), sliderPositions(4, :), labelPositions(4, :), sliderPanel);


runtime = toc;

fprintf('The program took %.2f seconds to run.\n', runtime)


return

%% Test Analytic InvKine

Qtest = Q;
X_W2B = double(subs(X_W2Bsym, [q1, q2, q3, q4], [Qtest(1), Qtest(2), Qtest(3), Qtest(4)]));

[Qinv] = invkine(X_W2B, UpperArm.Length, ForeArm.Length, d3, "ElbowUp");

Xtest = double(subs(X_W2Bsym, [q1, q2, q3, q4], [Qinv(1), Qinv(2), Qinv(3), Qinv(4)]));

fprintf('\nThe desired pose was:\n [%.4f \t%.4f \t%.4f \t%.4f \t%.4f \t%.4f]\n', X_W2B)
fprintf('\nThe obtained pose is:\n [%.4f \t%.4f \t%.4f \t%.4f \t%.4f \t%.4f]\n', Xtest)
fprintf('\nThe desired joint variables were:\n [%.4f \t%.4f \t%.4f \t%.4f]\n', Qtest)
fprintf('\nThe joint variables are:\n [%.4f \t%.4f \t%.4f \t%.4f]\n', Qinv)




