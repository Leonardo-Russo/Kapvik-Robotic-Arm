function T = buildT(R, P)
% Description: this function computes the transformation matrix starting
% from the rotation matrix and the displacement vector.
% 
% Inputs:
% R = rotation matix
% P = displacement vector
% 
% Outputs:
% T = transformation matrix

if size(P, 1) == 1 || size(P, 2) == 3
    P = P';
    warning('Wrong Dimensions for the Transformation -> input vector was transposed.')
end

if size(P, 1) ~= 3 || size(P, 2) ~= 1
    error('Wrong Dimensions for the Transformation.')
end

T = eye(4);
T(1:3, 1:3) = R;
T(1:3 , 4) = P;

end