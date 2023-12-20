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

% Precision Magnetic encoder
sigma=deg2rad(0.025);   % rad

% Define Rover Dimensions
L = 0.7;                % m
w = 0.5;                % m
h = 0.3;                % m

% Define Base Shift along y
P_By = 0.1;             % m

% Define Scoop Dimension and Inertia properties
scoopLength = 0.05;      % m
Mscoop=1;                % kg
Iscoop=(2/3)*Mscoop*scoopLength^2;

% Other Parameters
a2 = 0.46;              % m
a3 = 0.44;              % m
linkL = 0.18;           % m - link length

% Define Links
UpperArm = link(0.46, 40, 2, 3.5);
ForeArm = link(0.44, 40, 2, 3.5);

% Define Joints
Joint_1 = joint(1.15, 8.4*10^3, -160, 100, 1.5*10^(-4), -5*10^(-4), 1.44*10^(-6), 8.67, 0.84, 5.3*10^(-6), 10300, 0.025);
Joint_2 = joint(1.28, 8.4*10^3,  -90,  90, 1.5*10^(-4), -5*10^(-4), 1.44*10^(-6), 8.67, 0.84, 5.3*10^(-6), 10300, 0.025);
Joint_3 = joint(1.39, 5.3*10^3, -150, 110, 1.5*10^(-4), -5*10^(-4), 1.44*10^(-6), 8.67, 0.84, 5.3*10^(-6), 10300, 0.025);
Joint_4 = joint(0.67, 6.7*10^3,  -90,   5, 1.5*10^(-4), -5*10^(-4), 1.44*10^(-6), 8.67, 0.84, 5.3*10^(-6), 10300, 0.025);


% Maximum velocity of joint motors
omegaMax1=(10300*2*pi/60)/Joint_1.Gear_Ratio; % rad/s
omegaMax2=(10300*2*pi/60)/Joint_2.Gear_Ratio; % rad/s
omegaMax3=(10300*2*pi/60)/Joint_3.Gear_Ratio; % rad/s
omegaMax4=(10300*2*pi/60)/Joint_4.Gear_Ratio; % rad/s

% Define d3
d3 = vpa(-1.5*10^(-3)*UpperArm.Diameter);

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

fprintf("Initializing...\n");

T_W2Bsym = simplify(dirkine(TableMDHsym));
T_T2Ssym = where_fun(T_S2B, T_W2Bsym, T_T2W);
X_W2Bsym = simplify(trans2pose(T_W2Bsym));
X_Tsym = simplify(trans2pose(T_T2Ssym));

% Jsym = simplify(jacobian(X_Tsym, [q1 q2 q3 q4]));

%% Dynamical equations (symbolic expression)

[M, V, G, F] = dinEqs(Joint_1, Joint_2, Joint_3, Joint_4, UpperArm, ForeArm, P_T, Mscoop, Iscoop, d3);

%% Trajectory generation
Qstowage=[pi/2 -pi/2 -pi/2 -pi/6];
Qnavigation=[pi/2 -3*pi/20 -pi/6 -pi/6];
Qretrieval=[-pi/2 -pi/2 pi/4 -pi/2];
Qtransfer=[-pi/15 -pi/9 -5*pi/18 -pi/2]; 
ft=100; % path update rate [Hz] (1/timestep)
thetaddMax(1,1)=Joint_1.Tau_m_max/(Joint_1.Gear_Ratio*Joint_1.Motor_Inertia); % [rad/s^2] accMax joint 1
thetaddMax(1,2)=Joint_2.Tau_m_max/(Joint_2.Gear_Ratio*Joint_2.Motor_Inertia); % [rad/s^2] accMax joint 4
thetaddMax(1,3)=Joint_3.Tau_m_max/(Joint_3.Gear_Ratio*Joint_3.Motor_Inertia); % [rad/s^2] accMax joint 4
thetaddMax(1,4)=Joint_4.Tau_m_max/(Joint_4.Gear_Ratio*Joint_4.Motor_Inertia); % [rad/s^2] accMax joint 4

%% Trajectory Stowage to Navigation
TSto2Nav=25; % [s] total time from Stowage to Navigation
q0Sto2Nav=Qstowage';
qSto2NavInter1=[pi/2 0 -pi/2 -pi/6]';
qSto2NavInter2=[pi/2 deg2rad(-15) deg2rad(-75) -pi/6]';
qfSto2Nav=Qnavigation';
[tSto2Nav, qSto2Nav, qdSto2Nav, qddSto2Nav] = trajectoryGenerationSto2Nav...
                                 (q0Sto2Nav, qSto2NavInter1, qSto2NavInter2, qfSto2Nav, thetaddMax, TSto2Nav, ft);
% Plot
showTrajSto2Nav(tSto2Nav, qSto2Nav, qdSto2Nav, qddSto2Nav, omegaMax1, omegaMax2, omegaMax3, omegaMax4)

%% Trajectory Stowage to Retrieval
TSto2Ret=45; % [s] total time from Stowage to Retrieval
q0Sto2Ret=Qstowage';
qSto2RetInter1=[-pi/2 deg2rad(-5) -pi/2 -pi/2]';
qSto2RetInter2=[-pi/2 deg2rad(-30) deg2rad(-60) -pi/2]';
qfSto2Ret=Qretrieval';
[tSto2Ret, qSto2Ret, qdSto2Ret, qddSto2Ret] = trajectoryGenerationSto2Ret...
                                 (q0Sto2Ret, qSto2RetInter1, qSto2RetInter2, qfSto2Ret, thetaddMax, TSto2Ret, ft);

% Plot 
showTrajSto2Ret(tSto2Ret, qSto2Ret, qdSto2Ret, qddSto2Ret, omegaMax1, omegaMax2, omegaMax3, omegaMax4)

%% Trajectory Retrieval to Transfer
TRet2Trans=40; % [s] total time from Retrieval to Transfer
q0Ret2Trans=Qretrieval';
qRet2TransInter1=[Qretrieval(1:3) deg2rad(5)]';
qRet2TransInter2=[Qtransfer(1:3) deg2rad(5)]';
qfRet2Trans=Qtransfer';
[tRet2Trans, qRet2Trans, qdRet2Trans, qddRet2Trans] = trajectoryGenerationRet2Trans...
                                                (q0Ret2Trans, qRet2TransInter1, qRet2TransInter2, qfRet2Trans, thetaddMax, TRet2Trans, ft);

% Plot 
showTrajRet2Trans(tRet2Trans, qRet2Trans, qdRet2Trans, qddRet2Trans, omegaMax1, omegaMax2, omegaMax3, omegaMax4)

%% Control frequency 
fc=1000; % [Hz]

%% Control Stowage to Navigation
kpSto2Nav=[1000 1000 1000 1000];
KpSto2Nav=diag(kpSto2Nav);
kvSto2Nav=2*sqrt(kpSto2Nav);
KvSto2Nav=diag(kvSto2Nav);

% Integration
[tcSto2Nav, thetaSto2Nav, thetadSto2Nav, thetaddSto2Nav, qDesSto2Nav, qdDesSto2Nav,...
    qddDesSto2Nav, ESto2Nav, tauSto2Nav, tauMotorSto2Nav, iMotorSto2Nav] = control(TSto2Nav, Qstowage', [0 0 0 0]', ...
           fc, ft, qSto2Nav, qdSto2Nav, qddSto2Nav, Joint_1, Joint_2, Joint_3, Joint_4, KvSto2Nav, KpSto2Nav, sigma);

% Plot
showControl(tcSto2Nav, thetaSto2Nav, thetadSto2Nav, thetaddSto2Nav, qDesSto2Nav, qdDesSto2Nav,...
    qddDesSto2Nav, ESto2Nav, tauSto2Nav, tauMotorSto2Nav, iMotorSto2Nav,...
    "Sto2Nav", Joint_1.Tau_m_max, Joint_1.i_tau_m_max)

%% Control Stowage to Retrieval
kpSto2Ret=[1000 1000 1000 1000];
KpSto2Ret=diag(kpSto2Ret);
kvSto2Ret=2*sqrt(kpSto2Ret);
KvSto2Ret=diag(kvSto2Ret);

% Integration
[tcSto2Ret, thetaSto2Ret, thetadSto2Ret, thetaddSto2Ret, qDesSto2Ret, qdDesSto2Ret,...
    qddDesSto2Ret, ESto2Ret, tauSto2Ret, tauMotorSto2Ret, iMotorSto2Ret] = control(TSto2Ret, Qstowage', [0 0 0 0]', ...
        fc, ft, qSto2Ret, qdSto2Ret, qddSto2Ret, Joint_1, Joint_2, Joint_3, Joint_4, KvSto2Ret, KpSto2Ret, sigma);

% Plot
showControl(tcSto2Ret, thetaSto2Ret, thetadSto2Ret, thetaddSto2Ret, qDesSto2Ret, qdDesSto2Ret,...
    qddDesSto2Ret, ESto2Ret, tauSto2Ret, tauMotorSto2Ret, iMotorSto2Ret, "Sto2Ret", Joint_1.Tau_m_max, Joint_1.i_tau_m_max)

%% Control Retrieval to Transfer
kpRet2Trans=[1000 1000 1000 1000];
KpRet2Trans=diag(kpRet2Trans);
kvRet2Trans=2*sqrt(kpRet2Trans);
KvRet2Trans=diag(kvRet2Trans);

% Integration
[tcRet2Trans, thetaRet2Trans, thetadRet2Trans, thetaddRet2Trans, qDesRet2Trans, qdDesRet2Trans,...
    qddDesRet2Trans, ERet2Trans, tauRet2Trans, tauMotorRet2Trans, iMotorRet2Trans] = control(TRet2Trans, Qretrieval', [0 0 0 0]', ...
        fc, ft, qRet2Trans, qdRet2Trans, qddRet2Trans, Joint_1, Joint_2, Joint_3, Joint_4, KvRet2Trans, KpRet2Trans, sigma);

% Plot
showControl(tcRet2Trans, thetaRet2Trans, thetadRet2Trans, thetaddRet2Trans, qDesRet2Trans, qdDesRet2Trans,...
    qddDesRet2Trans, ERet2Trans, tauRet2Trans, tauMotorRet2Trans, iMotorRet2Trans, "Ret2Trans", Joint_1.Tau_m_max, Joint_1.i_tau_m_max)

%% Manipulator Visualization

% Set the Initial Joint Variables
global Q

Q=Qtransfer;
% Get Manipulator State
[T_12S, T_22S, T_32S, T_W2S, T_T2S, X_T] = ...
    getManipulatorState(Q, TableMDHsym, X_Tsym, T_B2S, T_T2W);

% Create the Workspace
figure('name', 'Enviroment Simulation', 'WindowState', 'maximized')

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

%% trajectory display
%stowage to navigation
sto2navButton = uicontrol('Style', 'pushbutton', 'String', 'Sto2Nav', 'Position', [250 20 100 20]);
set(sto2navButton, 'Callback',  {@(src,event) updatePlot(TableMDHsym, X_Tsym, T_B2S, T_T2W, joints, links, scoop, mframes, jframes, options, UpperArm, ForeArm, scoopLength, tSto2Nav, qSto2Nav, ft)});

%stowage to retrieval
sto2retrButton = uicontrol('Style', 'pushbutton', 'String', 'Sto2Retr', 'Position', [360 20 100 20]);
set(sto2retrButton, 'Callback',  {@(src,event) updatePlot(TableMDHsym, X_Tsym, T_B2S, T_T2W, joints, links, scoop, mframes, jframes, options, UpperArm, ForeArm, scoopLength, tSto2Ret, qSto2Ret, ft)});

%retrieval to transfer
ret2transButton = uicontrol('Style', 'pushbutton', 'String', 'Retr2Transf', 'Position', [470 20 100 20]);
set(ret2transButton, 'Callback',  {@(src,event) updatePlot(TableMDHsym, X_Tsym, T_B2S, T_T2W, joints, links, scoop, mframes, jframes, options, UpperArm, ForeArm, scoopLength, tRet2Trans, qRet2Trans, ft)});


% % Create a Panel to Group Sliders and Labels
% panelPosition = [0.65 0.01 0.34 0.36];  % [left bottom width height]
% sliderPanel = uipanel('Title', 'Joint Controls', ...
%              'FontSize', 10, ...
%              'FontWeight', 'bold', ...
%              'BackgroundColor', 'white', ...
%              'Position', panelPosition, ...   % Position of the panel within the figure
%              'Parent', gcf);                  % Set the parent to the current figure
% 
% % Define Slider and Label Positions Relative to the Panel
% sliderPositions = [80 10 100 20; 80 40 100 20; 80 70 100 20; 80 100 100 20];
% labelPositions = [10 5 50 20; 10 35 50 20; 10 65 50 20; 10 95 50 20];
% 
% % Create the Sliders
% slider_q1 = create_slider('q1', -160, 100, @(src, event) updatePlot(src, event, TableMDHsym, X_Tsym, T_B2S, T_T2W, joints, links, scoop, mframes, jframes, options, UpperArm, ForeArm, scoopLength), sliderPositions(1, :), labelPositions(1, :), sliderPanel);
% slider_q2 = create_slider('q2', -90, 90, @(src, event) updatePlot(src, event, TableMDHsym, X_Tsym, T_B2S, T_T2W, joints, links, scoop, mframes, jframes, options, UpperArm, ForeArm, scoopLength), sliderPositions(2, :), labelPositions(2, :), sliderPanel);
% slider_q3 = create_slider('q3', -150, 110, @(src, event) updatePlot(src, event, TableMDHsym, X_Tsym, T_B2S, T_T2W, joints, links, scoop, mframes, jframes, options, UpperArm, ForeArm, scoopLength), sliderPositions(3, :), labelPositions(3, :), sliderPanel);
% slider_q4 = create_slider('q4', -90, 5, @(src, event) updatePlot(src, event, TableMDHsym, X_Tsym, T_B2S, T_T2W, joints, links, scoop, mframes, jframes, options, UpperArm, ForeArm, scoopLength), sliderPositions(4, :), labelPositions(4, :), sliderPanel);


runtime = toc;

fprintf('The program took %.2f seconds to run.\n', runtime)

return

%% Test Analytic InvKine
% 
% Qtest = Q;
% X_W2B = double(subs(X_W2Bsym, [q1, q2, q3, q4], [Qtest(1), Qtest(2), Qtest(3), Qtest(4)]));
% 
% [Qinv] = invkine(X_W2B, UpperArm.Length, ForeArm.Length, d3, "ElbowUp");
% 
% Xtest = double(subs(X_W2Bsym, [q1, q2, q3, q4], [Qinv(1), Qinv(2), Qinv(3), Qinv(4)]));
% 
% fprintf('\nThe desired pose was:\n [%.4f \t%.4f \t%.4f \t%.4f \t%.4f \t%.4f]\n', X_W2B)
% fprintf('\nThe obtained pose is:\n [%.4f \t%.4f \t%.4f \t%.4f \t%.4f \t%.4f]\n', Xtest)
% fprintf('\nThe desired joint variables were:\n [%.4f \t%.4f \t%.4f \t%.4f]\n', Qtest)
% fprintf('\nThe joint variables are:\n [%.4f \t%.4f \t%.4f \t%.4f]\n', Qinv)


