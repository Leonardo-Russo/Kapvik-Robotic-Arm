function showJdet
% Description: show the determinant of the Jacobian.

global Q Qsym Jsym

q1 = Qsym(1);
q2 = Qsym(2);
q3 = Qsym(3);
q4 = Qsym(4);

J = double(subs(Jsym, [q1, q2, q3, q4], [Q(1), Q(2), Q(3), Q(4)]));

detJ = det(J'*J);

fprintf('det(J) =\t%.6f\n', abs(detJ));

end