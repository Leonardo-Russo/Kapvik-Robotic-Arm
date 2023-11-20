function J = ObjFun_InvKine(table, T_S2B, T_T2W, X0)
% Description: this is the cost function for the inverse kinematics. The
% idea is that we must find the vector Q which is the closest to yielding X
% after the backconversion.
% 
% Inputs:
% table = MDH table - 4x4
% T_S2B = Transformation matrix from station frame to base frame
% T_T2W = Transformation matrix from tool frame to wrist frame
% X0 = initial carthesian state vector - 6x1
% Outputs:
% J = cost value

% Compute the new Carthesian State
T_W2B = dir_kine(table);
T_T2S = where_fun(T_S2B, T_W2B, T_T2W);
X = trans2pose(T_T2S);

% Compute Carthesian State Error
eps = X - X0;

% Define scaling factor for attitude error
alpha = 1e3;

% Compute Cost Value
J = sum(abs(eps(1:3))) + alpha*sum(abs(eps(4:6)));

end