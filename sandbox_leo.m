%% Leonardo's Sandbox

close all
clear all
clc

% Substitute specific values for q1, q2, q3, q4
q1_val = 1;
q2_val = 2;
q3_val = 3;
q4_val = 4;

% Use the subs function
b_val = subs(b, [q1, q2, q3, q4], [q1_val, q2_val, q3_val, q4_val]);

% If you want to evaluate to numerical values (for instance, if your q's are in radians)
b_val = double(b_val);