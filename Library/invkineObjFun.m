function J = invkineObjFun(Q, X0, Qsym, X_Tsym)
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

x = X0(1);
y = X0(2);
z = X0(3);
% r = X0(4);
% p = X0(5);
% ya = X0(6);

xsym = X_Tsym(1);
ysym = X_Tsym(2);
zsym = X_Tsym(3);
% rsym = X_Tsym(4);
% psym = X_Tsym(5);
% yasym = X_Tsym(6);

Xsymnew = [xsym, ysym, zsym];
% Xsymnew = [xsym, ysym, zsym, rsym, psym, yasym];
X0new = [x, y, z];
% X0new = [x, y, z, r, p, ya];

% Compute Pose from Q
X = double(subs(Xsymnew, [q1, q2, q3, q4], [Q(1), Q(2), Q(3), Q(4)]));

% Compute Carthesian State Error
eps = X - X0new;

% Define scaling factor for attitude error
alpha = 1;

% Compute Cost Value
J = norm(X - X0new);

end