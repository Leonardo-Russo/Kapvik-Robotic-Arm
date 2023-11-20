function J = invkineObjFun(Q, X0, Qsym, Xsym)
% Description: this is the cost function for the inverse kinematics. The
% idea is that we must find the vector Q which is the closest to yielding X
% after the backconversion.
% 
% Inputs:
% Q = iterative state of joint variables
% X0 = desired pose
% Qsym = state with symbolic joint variables
% Xsym = state with symbolic pose
% 
% Outputs:
% J = cost function value

% Retrieve Data from Input
q1 = Qsym(1);
q2 = Qsym(2);
q3 = Qsym(3);
q4 = Qsym(4);

% Compute Pose from Q
X = double(subs(Xsym, [q1, q2, q3, q4], [Q(1), Q(2), Q(3), Q(4)]));

% Compute Carthesian State Error
eps = X - X0;

% Define scaling factor for attitude error
alpha = 1;

% Compute Cost Value
J = norm(X - X0);

end