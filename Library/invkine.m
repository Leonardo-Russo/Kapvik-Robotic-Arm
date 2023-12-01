function Q = invkine(X0, Qsym, X_Tsym)
% Description: this will be the shell function for the inverse kinematics.
% 
% Inputs:
% X0 = desired pose
% Qsym = state of symbolic joint variables
% Xsym = symbolic pose
% Outputs:
% Q = joints state vector

% % Settings for fmincon() - testing
% OptionsFMIN = optimoptions('fmincon', ...
%     'PlotFcn', {@optimplotfval}, ...
%     'Display', 'iter-detailed', ...
%     'StepTolerance', 1e-9, ...
%     'ConstraintTolerance', 1e-6, ...
%     'Algorithm', 'interior-point', ...
%     'Diagnostics', 'off', ...
%     'MaxFunctionEvaluations', 1000);

% Settings for fmincon()
OptionsFMIN = optimoptions('fmincon', ...
    'StepTolerance', 1e-9, ...
    'ConstraintTolerance', 1e-6, ...
    'Algorithm', 'interior-point', ...
    'Diagnostics', 'off', ...
    'MaxFunctionEvaluations', 1000);


% Set Initial Conditions
Q0 = zeros(4, 1);

% Set the Joint Variable Boundaries
lb = [deg2rad(-160), deg2rad(-90), deg2rad(-150), deg2rad(-90)];
ub = [deg2rad(100), deg2rad(90), deg2rad(110), deg2rad(5)];

% Set additional parameters for fmincon()
A = [];
B = [];
Aeq = [];
Beq = [];

fprintf('Performing the Inverse Kinematics ...\n')

[Q, fval, exitflag] = fmincon(@(Q) invkineObjFun(Q, X0, Qsym, X_Tsym), Q0, A, B, Aeq, Beq, lb, ub, [], OptionsFMIN);


% % ParticleSwarm Approach
% 
% % Settings for particleswarm()
% OptionsPSO = optimoptions('particleswarm', ...
%     'PlotFcn', {@pswplotbestf}, ...
%     'HybridFcn', {@fmincon, OptionsFMIN}, ...   % hybrid function setting
%     'SwarmSize', 100, ...
%     'Display', 'iter', ...
%     'InertiaRange', [0.1 1.1], ...
%     'UseParallel', false, ...
%     'MaxIterations', 10);
% 
% dim = 4;        % n° of joint variables
% Q = particleswarm(@(Q) ObjFun_invkinePSO(Q, X0, Qsym, Xsym), dim, lb, ub, OptionsPSO);


end