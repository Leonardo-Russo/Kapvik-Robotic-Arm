function T = tableRow2T(R)
% Description: 
% this function takes as input the i-th row of the MDH Table 
% and computes the transformation matrix from i to i-1.
% 
% Inputs: 
% R = i-th row of the MDH Table
% 
% Outputs: 
% T = transformation matrix from i to i-1

alpha = R(1);
a = R(2);
d = R(3);
theta = R(4);

T = [cos(theta),                -sin(theta),                0,                  a;...
     sin(theta)*cos(alpha),     cos(theta)*cos(alpha),      -sin(alpha),        -sin(alpha)*d;...
     sin(theta)*sin(alpha),     cos(theta)*sin(alpha),      cos(alpha),         cos(alpha)*d;...
     0,                         0                           0,                  1];

end