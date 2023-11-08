function [J] = analytic_jacobian(f, q)
% INPUT:    - direct kinematics f(q) 
%           - array of variables q=[q1 q2 q3 q4] ...
% OUTPUT: Jacobian.
    J = [];
    [~,n] = size(q);
    for i = 1:n
        q_i = q(1,i);
        J_i = diff(f,q_i);
        J = [J J_i];
    end

end