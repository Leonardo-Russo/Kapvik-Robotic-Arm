function Q0 = inv_kine_GM(X0, Qsym, Xsym)
% Description: this function computes the inverse kinematics using the
% Gradient Method.
% 
% Inputs:
% X0 = desired pose
% Qsym = symbolic joint variables
% 
% Outputs:
% q = joint variables

% Nota: bisogna imporre i vincoli agli angoli!!!

% Retrieve Data from Input
q1 = Qsym(1);
q2 = Qsym(2);
q3 = Qsym(3);
q4 = Qsym(4);

% Set Hyperparameters
max_iter = 100;
stop = 0;
alpha = 1;

x_tol = 1e-2;
step_tol = 1e-2;

% Set Initial Guess
Q1 = zeros(4, 1);

% Create Symbolic Variables
J = simplify(jacobian(Xsym, [q1 q2 q3 q4]));

for i = 1 : max_iter

    % Retrieve Iterative Value
    Q0 = Q1;

    % Compute Pose from Q
    Xi = double(subs(Xsym, [q1, q2, q3, q4], [Q0(1), Q0(2), Q0(3), Q0(4)]));
    Ji = double(subs(J, [q1, q2, q3, q4], [Q0(1), Q0(2), Q0(3), Q0(4)]));
    
    % Update Q State
    Q1 = Q0 + alpha * Ji' * (X0 - Xi);

    % Stopping Criterion
    if norm(X0 - Xi) <= x_tol || norm(Q1 - Q0) <= step_tol
        stop = 1;
        break
    end

end

if i == max_iter && stop == 0
    warning('Inverse Kinematics did not converge to a solution!')
end

end