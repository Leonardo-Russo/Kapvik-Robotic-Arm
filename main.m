%% Space Robotic Systems - Project

close all
clear
clc 

addpath('Library/')

L = 0.7;
h = 0.5;

%% Define MDH Table

syms q1 q2 q3 q4 real

TableMDH = defineTable(q1, q2, q3, q4);

P_B = [0 -0.1 h]';
T_B2S = [eye(3), P_B;
         zeros(1, 3), 1];

T_S2B = inv_trans(T_B2S);

P_T = [0 0 0]';
T_T2W = [eye(3), P_T;
         zeros(1, 3), 1];

% TableMDH = double(subs(TableMDH, [q1, q2, q3, q4], [pi/2, 0, 0, 0]));
% showTable(TableMDH)

%% Define links and joints properties

Upper_Arm=link(0.46, 40, 2, 3.5);
Fore_Arm=link(0.40, 40, 2, 3.5);

Joint_1=joint(1.15, 8.4, -160, 100, 1.5*10^(-4), -5*10^(-4));
Joint_2=joint(1.28, 8.4,  -90,  90, 1.5*10^(-4), -5*10^(-4));
Joint_3=joint(1.39, 5.3, -150, 110, 1.5*10^(-4), -5*10^(-4));
Joint_4=joint(0.67, 6.7,  -90,   5, 1.5*10^(-4), -5*10^(-4));

%% Test

[T_W2B] = simplify(dir_kine(TableMDH));
[T_T2S] = where_fun(T_S2B, T_W2B, T_T2W);
X = simplify(trans2pose(T_T2S));

J = simplify(jacobian(X, [q1 q2 q3 q4]));

%% Plots

% close all
% 
% figure('name', 'Workspace')
% 
% % plot3(0, 0, 0, 'b*')    % Station Origin
% show_frame(zeros(6, 1), 'b')
% xlabel('x')
% ylabel('y')
% zlabel('z')
% hold on
% show_frame([P_B; zeros(3, 1)], 'r')
% 
% grid on
% axis([-1 1 -1 1 -1 1])
% show_frame(X, '#eb34e5')

