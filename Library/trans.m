function y = trans(T, z)
% Description: this function computes the transformation of a vector using
% the transformation matrix in input.
% 
% Inputs:
% T = transformation matrix
% z = pre-transformation 3x1 vector
% 
% Outputs:
% y = post-transformation 3x1 vector

if size(z, 1) == 1 || size(z, 2) == 3
    warning('Wrong Dimensions for the Transformation -> input vector was transposed.')
end

if size(z, 1) ~= 3 || size(z, 2) ~= 1
    error('Wrong Dimensions for the Transformation.')
end

% Expand input vector
z = [z; 1];

% Compute transformed vector
y = T * z;
y = y(1:3);


end