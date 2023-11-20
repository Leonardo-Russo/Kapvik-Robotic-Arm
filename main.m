%% Space Robotic Systems - Project

close all
clear
clc 

addpath('Library/')

L = 0.7;
h = 0.3;

%% Define MDH Table

syms q1 q2 q3 q4 real

TableMDH = define_table(q1, q2, q3, q4);

%% Define Station and Base Reference Frames

R_B = eye(3);
P_B = [0 -0.1 h]';
T_B2S = buildT(R_B, P_B);
T_S2B = inv_trans(T_B2S);

R_T = eye(3);
P_T = [0 0 0]';
T_T2W = buildT(R_T, P_T);

%% Compute Jacobian Matrix

[T_W2B] = simplify(dir_kine(TableMDH));
[T_T2S] = where_fun(T_S2B, T_W2B, T_T2W);
X = simplify(trans2pose(T_T2S));

J = simplify(jacobian(X, [q1 q2 q3 q4]));

%% Define links and joints properties

Upper_Arm=link(0.46, 40, 2, 3.5);
Fore_Arm=link(0.44, 40, 2, 3.5);

Joint_1=joint(1.15, 8.4, -160, 100, 1.5*10^(-4), -5*10^(-4));
Joint_2=joint(1.28, 8.4,  -90,  90, 1.5*10^(-4), -5*10^(-4));
Joint_3=joint(1.39, 5.3, -150, 110, 1.5*10^(-4), -5*10^(-4));
Joint_4=joint(0.67, 6.7,  -90,   5, 1.5*10^(-4), -5*10^(-4));


%% Plots

close all

% Compute Necessary Variables
TableMDH = double(subs(TableMDH, [q1, q2, q3, q4], [pi/12 pi/9, -pi/4, pi/4]));
show_table(TableMDH)
T_W2B = dir_kine(TableMDH);
T_T2S = where_fun(T_S2B, T_W2B, T_T2W);
X = trans2pose(T_T2S);

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
show_frame(trans2pose(T_12S), "g", "1")
show_frame(trans2pose(T_22S), "c", "2")
show_frame(trans2pose(T_32S), "m", "3")
show_frame(trans2pose(T_W2S), "#EDB120", "W")
DrawJoint(0.03, 0.05, 0.09, T_12S) % first joint
DrawJoint(0.03, 0.05, 0.09, T_22S) % second joint
DrawJoint(0.03, 0.05, 0.09, T_32S) % third joint
DrawJoint(0.03, 0.05, 0.09, T_W2S) % fourth joint
DrawLink(Upper_Arm, T_22S); % first link
DrawLink(Fore_Arm, T_32S);  % second link
legend('Station frame', 'Base frame', 'Tool frame', '$1^{st}$ joint frame',...
    '$2^{nd}$ joint frame', '$3^{rd}$ joint frame', '$4^{th}$ joint frame (wrist frame)',...
    'fontsize', 12,'Interpreter', 'latex')


%% Test 201241i23y1

I = eye(3);

X_T = trans2pose(T_W2S);

roll = rad2deg(X(4))
pitch = rad2deg(X(5))
yaw = rad2deg(X(6))

% I = R3(X(6))*I;
% I = R2(X(5))*I;
% I = R1(X(4))*I;

I = R3(15, "deg")*I;
I = R2(-20, "deg")*I;
I = R1(90, "deg")*I;

yaw = q1;

pitch = q2 + q3 + q4;

roll = pi/2;


% show_frame(trans2pose(buildT(I, zeros(3, 1))), '#a83232');


% show_frame(trans2pose(buildT(I, zeros(3, 1))), '#a83232');


show_frame(trans2pose(buildT(I', [0.802717233896431, 0.115087434573093, 0.271377230763900]')), '#a83232');


%%

inv_kine_anal()