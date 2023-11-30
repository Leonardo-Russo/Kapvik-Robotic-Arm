%% Space Robotic Systems - Project

close all
clear
clc 

addpath('Library/')

L = 0.7;
h = 0.3;
l = 0.1;

%% Define links and joints properties

Upper_Arm=link(0.46, 40, 2, 3.5);
Fore_Arm=link(0.44, 40, 2, 3.5);

Joint_1=joint(1.15, 8.4, -160, 100, 1.5*10^(-4), -5*10^(-4), 1.44*10^(-6), 8.67, 0.84, 5.3*10^(-6), 10300, 0.025);
Joint_2=joint(1.28, 8.4,  -90,  90, 1.5*10^(-4), -5*10^(-4), 1.44*10^(-6), 8.67, 0.84, 5.3*10^(-6), 10300, 0.025);
Joint_3=joint(1.39, 5.3, -150, 110, 1.5*10^(-4), -5*10^(-4), 1.44*10^(-6), 8.67, 0.84, 5.3*10^(-6), 10300, 0.025);
Joint_4=joint(0.67, 6.7,  -90,   5, 1.5*10^(-4), -5*10^(-4), 1.44*10^(-6), 8.67, 0.84, 5.3*10^(-6), 10300, 0.025);

%% Define MDH Table

syms q1 q2 q3 q4 q1d q2d q3d q4d q1dd q2dd q3dd q4dd g real

TableMDHsym = define_table(q1, q2, q3, q4, 1.5*10^(-3)*Upper_Arm.Diameter);

%% Define Station and Base Reference Frames

R_B = eye(3);
P_B = [0 -l h]';
T_B2S = buildT(R_B, P_B);
T_S2B = inv_trans(T_B2S);

R_T = eye(3);

P_T = [0.05 0 0]';

T_T2W = buildT(R_T, P_T);

%% Compute Jacobian Matrix

T_W2Bsym = simplify(dirkine(TableMDHsym));
T_T2Ssym = where_fun(T_S2B, T_W2Bsym, T_T2W);
Xsym = simplify(trans2pose(T_T2Ssym));

Jsym = simplify(jacobian(Xsym, [q1 q2 q3 q4]));


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
X = double(subs(Xsym, [q1, q2, q3, q4], [Q(1), Q(2), Q(3), Q(4)]));

% Create the Workspace
figure('name', 'Workspace Test', 'WindowState', 'maximized')
show_frame(zeros(6, 1), 'k', "S")                   % Station frame
show_frame([P_B; zeros(3, 1)], '#2b31ed', "B")      % Base frame
show_frame(X, '#e84f1c', "T")                       % Tool frame
plot3DBox(L, 0.5, h);                               % Create the Box
show_plane([1 0 0], [0 1 0], [0 0 0])               % Create the Ground

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
show_frame(trans2pose(T_12S), "g", "1")
show_frame(trans2pose(T_22S), "c", "2")
show_frame(trans2pose(T_32S), "m", "3")
show_frame(trans2pose(T_W2S), "#EDB120", "W")
DrawJoint(0.05, 0.07, 0.18, T_12S) % first joint
DrawJoint(0.05, 0.07, 0.18, T_22S) % second joint
DrawJoint(0.05, 0.07, 0.18, T_32S) % third joint
DrawJoint(0.05, 0.07, 0.18, T_W2S) % fourth joint
DrawLink(Upper_Arm, T_22S); % first link
DrawLink(Fore_Arm, T_32S);  % second link
plotScoop(P_T(1), T_12S, T_22S, T_W2S) % Scoop
legend('Station frame', 'Base frame', 'Tool frame', '$1^{st}$ joint frame',...
    '$2^{nd}$ joint frame', '$3^{rd}$ joint frame', '$4^{th}$ joint frame (wrist frame)',...
    'fontsize', 12,'Interpreter', 'latex')

