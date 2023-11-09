function J = ObjFun_InvKine(Q, X0)
% Description: this is the cost function for the inverse kinematics. The
% idea is that we must find the vector Q which is the closest to yielding X
% after the backconversion.
% 
% Inputs:
% Q = joint state vector - 4x1
% X0 = initial carthesian state vector - 6x1
% Outputs:
% J = cost value

% Compute the new Carthesian State
X = fkine(Q); %%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Compute Carthesian State Error
eps = X - X0;

% Define scaling factor for attitude error
alpha = 1e3;

% Compute Cost Value
J = sum(abs(eps(1:3))) + alpha*sum(abs(eps(4:6)));

end