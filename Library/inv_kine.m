function Q = inv_kine(X)
% Description: this will be the shell function for the inverse kinematics.
% 
% Inputs:
% X = carthesian state vector
% Outputs:
% Q = joints state vector

% Settings for fmincon()
OptionsFMIN = optimoptions('fmincon', ...
    'PlotFcn', {@optimplotfval}, ...
    'Display', 'iter-detailed', ...
    'StepTolerance', 1e-12, ...
    'ConstraintTolerance', 1e-9, ...
    'Algorithm', 'sqp', ...
    'Diagnostics', 'off', ...
    'MaxFunctionEvaluations', 1000);


% Settings for particleswarm()
OptionsPSO = optimoptions('particleswarm', ...
    'PlotFcn', {@pswplotbestf}, ...
    'HybridFcn', {@fmincon, OptionsFMIN}, ...   % hybrid function setting
    'SwarmSize', 100, ...
    'Display', 'iter', ...
    'InertiaRange', [0.1 1.1], ...
    'UseParallel', true, ...
    'MaxIterations', 100);


% Set the Joint Variable Boundaries
lb = [deg2rad(-160), deg2rad(-90), deg2rad(-150), deg2rad(-90)];
ub = [deg2rad(100), deg2rad(90), deg2rad(110), deg2rad(5)];

dim = 4;        % n° of joint variables

% Perform the Optimization
% [T_W2B] = dir_kine(table);
% [T_T2S] = where_fun(T_S2B, T_W2B, T_T2W);
% [X] = trans2pose(T_T2S);

Q = particleswarm(@(Q) ObjFun_InvKine(Q, X), dim, lb, ub, OptionsPSO);


end