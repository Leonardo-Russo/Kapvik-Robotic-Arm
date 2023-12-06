function [X] = trans2pose(T_i2j)
% Description: This function calculate the pose of the robot from
% transformation matrix
% Inputs:
% T_i2j = Transformation matrix from frame i to frame j
% 
% Outputs:
% X = [x y z roll pitch yaw] = 6x1 state vector 
% For the rotation matrix is considered the sequence 3-2-1:
% R=R1(roll)*R2(pitch)*R3(yaw);
% roll = angle along x in the intervall [-pi; pi)
% pitch = angle along y in the intervall [-pi/2; pi/2]
% yaw = angle along z in the intervall [-pi; pi)

R = T_i2j(1:3, 1:3)';
X(1:3,1) = T_i2j(1:3, 4);
pitch = -asin(R(1, 3));
roll = 2*atan((R(2, 3)/cos(pitch))/(1+((R(3,3)/cos(pitch)))));
yaw = 2*atan((R(1, 2)/cos(pitch))/(1+((R(1,1)/cos(pitch)))));
X(4,1) = roll;
X(5,1) = pitch;
X(6,1) = yaw;

end