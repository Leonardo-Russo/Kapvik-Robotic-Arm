function [T_W2B] = dir_kine(table)
% Description: This function computes the foward kinematic using
% the MDH table in input.
% Inputs:
% table = MDH table
% 
% Outputs:
% T_B2W = Transformation matrix from base frame to wrist frame

for i = 1:4
    T_W2Bj = tableRow2T(table(i, :));
    T_W2B=T_W2B*T_W2Bj;
end