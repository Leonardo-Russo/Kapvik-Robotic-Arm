function [T_W2B] = solve_fun(T_S2B, T_T2S, T_T2W)
% Description: This function computes the transformation beetween the wrist
% frame and the base frame.
% Inputs:
% table = MDH table
% T_S2B = Transformation matrix from station frame to base frame
% T_T2S = Transformation matrix from tool frame to station frame
% T_T2W = Transformation matrix from tool frame to wrist frame
% 
% Outputs:
% T_W2B = Transformation matrix from tool frame to station frame (INPUT FOR
% INVERSE KINEMATIC)

T_W2B = T_S2B*T_T2S/T_T2W;
end