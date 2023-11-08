%% Space Robotic Systems - Project

close all
clear all
clc

addpath('Library/')

%% Define MDH Table

syms q1 q2 q3 q4

TableMDH = defineTable(q1, q2, q3, q4);

