function inv_kine_anal(Xd, P_B)
% Description: analytical inverse kinematics.

% Retrieve Data from Input
q1 = Qsym(1);
q2 = Qsym(2);
q3 = Qsym(3);
q4 = Qsym(4);

P = Xd(1:3) - P_B;

x = P(1);
y = P(2);
z = P(3);

% q3
c3 = (norm(P)^2 - a2^2 - a3^2)/(2*a2*a3);
s3_p = sqrt(1 - c3^2);
s3_n = -s3_p;

q3_p = atan2(s3_p, c3);
q3_n = atan2(s3_n, c3);


% q1
c1_p = x/(sqrt(x^2 + y^2));
c1_n = -x/(sqrt(x^2 + y^2));
s1_p = y/(sqrt(x^2 + y^2));
s1_n = -y/(sqrt(x^2 + y^2));

q1_pp = atan2(y, x);
q1_pn = atan2(y, -x);
q1_np = atan2(-y, x);
q1_nn = atan2(-y, -x);

% set q1 boundaries

% q2
c2_pp = (a2 + a3*c3)*(c1_p*x + s1_p*y) + a3*s3_p*z;
c2_pn = (a2 + a3*c3)*(c1_p*x + s1_p*y) + a3*s3_n*z;
c2_np = (a2 + a3*c3)*(c1_n*x + s1_n*y) + a3*s3_p*z;




end