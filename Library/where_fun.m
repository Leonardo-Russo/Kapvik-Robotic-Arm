function [T_T2S] = where_fun(T_S2B, T_W2B, T_T2W)
% Description: This function computes the transformation beetween the tool
% frame and the station frame (AFTER FOWARD KINEMATIC).
% Inputs:
% table = MDH table
% T_S2B = Transformation matrix from station frame to base frame
% T_W2B = Transformation matrix from wrist frame to base frame (output
% foward kinematic)
% T_T2W = Transformation matrix from tool frame to wrist frame
% 
% Outputs:
% T_T2S = Transformation matrix from tool frame to station frame

T_T2S = T_S2B\T_W2B*T_T2W;
end