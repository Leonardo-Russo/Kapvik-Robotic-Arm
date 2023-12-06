function show_pseudodetJ()
% Description: this function computes the pseudo-determinant of a non
% square matrix A.

global Q Qsym Jsym

q1 = Qsym(1);
q2 = Qsym(2);
q3 = Qsym(3);
q4 = Qsym(4);

J = double(subs(Jsym, [q1, q2, q3, q4], [Q(1), Q(2), Q(3), Q(4)]));

% Perform Singular Value Decomposition
[~, S, ~] = svd(J);

% Extract the diagonal elements (singular values) from S
singular_values = diag(S);

% Filter out the non-zero singular values
non_zero_singular_values = singular_values(singular_values > 0);

% Compute the product of non-zero singular values
pdetJ = prod(non_zero_singular_values);

fprintf('pdet(J)\t=\t%.6f\n', abs(pdetJ));

end