%% Space Robotic Systems - Project

close all
clear
clc 

addpath('Library/')

%% Define MDH Table

syms q1 q2 q3 q4

TableMDH = defineTable(q1, q2, q3, q4);

%% Define links and joints properties

Upper_Arm=Link(0.46, 40, 2, 3.5);
Fore_Arm=Link(0.40, 40, 2, 3.5);

Joint_1=Joint(1.15, 8.4, -160, 100, 1.5*10^(-4), -5*10^(-4));
Joint_2=Joint(1.28, 8.4,  -90,  90, 1.5*10^(-4), -5*10^(-4));
Joint_3=Joint(1.39, 5.3, -150, 110, 1.5*10^(-4), -5*10^(-4));
Joint_4=Joint(0.67, 6.7,  -90,   5, 1.5*10^(-4), -5*10^(-4));