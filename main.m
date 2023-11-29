%% Space Robotic Systems - Project

close all
clear
clc 

addpath('Library/')


% To Do's:
% - aggiungi la luce
% - fai la leggenda con il toggle
% - cambia show_frame in modo che accetti anche la matrice di transformazione

%% Define links and joints properties

% Box Parameters
L = 0.7;                % m
w = 0.5;                % m
h = 0.3;                % m

% Base Shift
P_By = 0.1;             % m

% Scoop Dimension
scoopLength = 0.1;      % m

% Other Parameters
a2 = 0.46;              % m
a3 = 0.44;              % m

% Create Links
Upper_Arm = link(0.46, 40, 2, 3.5);
Fore_Arm = link(0.44, 40, 2, 3.5);

% Create Joints
Joint_1=joint(1.15, 8.4, -160, 100, 1.5*10^(-4), -5*10^(-4));
Joint_2=joint(1.28, 8.4,  -90,  90, 1.5*10^(-4), -5*10^(-4));
Joint_3=joint(1.39, 5.3, -150, 110, 1.5*10^(-4), -5*10^(-4));
Joint_4=joint(0.67, 6.7,  -90,   5, 1.5*10^(-4), -5*10^(-4));

% Define d3
d3 = 1.5*10^(-3)*Upper_Arm.Diameter;


%% Define MDH Table

syms q1 q2 q3 q4 q1d q2d q3d q4d q1dd q2dd q3dd q4dd g real

TableMDHsym = define_table(q1, q2, q3, q4, a2, a3, d3);

%% Define Station and Base Reference Frames

R_B = eye(3);
P_B = [0 -P_By h]';
T_B2S = buildT(R_B, P_B);
T_S2B = inv_trans(T_B2S);

R_T = eye(3);

P_T = [scoopLength 0 0]';

T_T2W = buildT(R_T, P_T);

%% Compute Jacobian Matrix

T_W2Bsym = simplify(dirkine(TableMDHsym));
T_T2Ssym = where_fun(T_S2B, T_W2Bsym, T_T2W);
X_Tsym = simplify(trans2pose(T_T2Ssym));

Jsym = simplify(jacobian(X_Tsym, [q1 q2 q3 q4]));


% %% Testing for Inverse Kinematics
% 
% X0 = double(subs(Xsym, [q1, q2, q3, q4], [pi/12 pi/9, -pi/4, pi/4]));
% 
% tic
% Q = invkine(X0, [q1, q2, q3, q4], Xsym);
% stopwatch = toc;
% 
% X1 = double(subs(Xsym, [q1, q2, q3, q4], [Q(1), Q(2), Q(3), Q(4)]));
% 
% fprintf('\nThe desired pose was:\n [%.4f \t%.4f \t%.4f \t%.4f \t%.4f \t%.4f]\n', X0(:))
% fprintf('\nThe retrieved joint variables yield this new pose:\n [%.4f \t%.4f \t%.4f \t%.4f \t%.4f \t%.4f]\n', X1(:))
% fprintf('\nThe time required was: %.2f s\n', stopwatch)

%% Plots

close all

% Set the Joint Variables
Q = [pi/12 pi/9, -pi/4, pi/4];

% Compute Necessary Variables
TableMDH = double(subs(TableMDHsym, [q1, q2, q3, q4], [Q(1), Q(2), Q(3), Q(4)]));
T_W2B = double(subs(T_W2Bsym, [q1, q2, q3, q4], [Q(1), Q(2), Q(3), Q(4)]));
T_T2S = double(subs(T_T2Ssym, [q1, q2, q3, q4], [Q(1), Q(2), Q(3), Q(4)]));
X_T = double(subs(X_Tsym, [q1, q2, q3, q4], [Q(1), Q(2), Q(3), Q(4)]));

T_12B = tableRow2T(TableMDH(1, :));
T_12S = T_B2S * T_12B;
T_221 = tableRow2T(TableMDH(2, :));
T_22S = T_B2S * T_12B * T_221;
T_322 = tableRow2T(TableMDH(3, :));
T_32S = T_B2S * T_12B * T_221 * T_322;
T_W23 = tableRow2T(TableMDH(4, :));
T_W2S = T_B2S * T_12B * T_221 * T_322 * T_W23;
T_T2W = [eye(3), P_T; 0 0 0 1];
% T_T2S = T_B2S * T_12B * T_221 * T_322 * T_W23 * T_T2W;

X_S = zeros(6, 1);
X_B = trans2pose(T_B2S);
X_W = trans2pose(T_W2S);

X_1 = trans2pose(T_12S);
X_2 = trans2pose(T_22S);
X_3 = trans2pose(T_32S);

% Create the Workspace
figure('name', 'Enviroment Simulation')

env = show_env(L, w, h);

mframes = show_mainframes(X_S, X_B, X_W, X_T);

jframes = show_jointframes(X_1, X_2, X_3);

joints = show_joints(T_12S, T_22S, T_32S, T_W2S);

DrawLink(Upper_Arm, T_22S); % first link
DrawLink(Fore_Arm, T_32S);  % second link

plotScoop(scoopLength, T_12S, T_22S, T_W2S) % Scoop

axis equal

% % Add Ligthing
% light('Position', [-1, -1, 1])
% lighting gouraud

% legend('Station frame', 'Base frame', 'Tool frame', '$1^{st}$ joint frame',...
%     '$2^{nd}$ joint frame', '$3^{rd}$ joint frame', '$4^{th}$ joint frame (wrist frame)',...
%     'fontsize', 12,'Interpreter', 'latex')

%%

X_Snew = [1 1 1 0 pi 0]';
update_frame(mframes.S, X_Snew)

global inner_diameter outer_diameter Length
inner_diameter = 0.05;
outer_diameter = 0.07;
Length = 0.18;


T_12S(1, 4) = 1;
update_joint(joints.J1, T_12S);