function Q = invkineLEO(X0, Qsym, X_Tsym, L1, L2)
% Description: this function computes the analytical inverse 
% kinematics.

nsol = 8;       % n° of invkine analytical solutions

% Retrieve Data from Input
x = X0(1);
y = X0(2);
z = X0(3);
roll = X0(4);
pitch = X0(5);
yaw = X0(6);

q1 = Qsym(1);
q2 = Qsym(2);
q3 = Qsym(3);
q4 = Qsym(4);


% Calculation of c3
c3 = (- L1^2 - L2^2 + x^2 + y^2 + z^2)/(2*L1*L2);

% Calculation of s3 for both cases
s3_1 = sqrt(1 - c3^2);
s3_2 = -sqrt(1 - c3^2);

% Calculation of q3 for both cases
q3_1 = atan2(sqrt(1 - c3^2), c3);
q3_2 = atan2(-sqrt(1 - c3^2), c3);

% Calculation of c2 for four cases
c2_1 = -sqrt(-(z + L2*s3_1 + ((2*L1 + 2*L2*c3)*(L1 + L2*c3 + y/sin(yaw)))/(2*L2*s3_1))/(2*L2*s3_1 + (2*L1 + 2*L2*c3)^2/(2*L2*s3_1)));
c2_2 = -sqrt(-(z + L2*s3_2 + ((2*L1 + 2*L2*c3)*(L1 + L2*c3 + y/sin(yaw)))/(2*L2*s3_2))/(2*L2*s3_2 + (2*L1 + 2*L2*c3)^2/(2*L2*s3_2)));
c2_3 = sqrt((z + L2*s3_1 + ((2*L1 + 2*L2*c3)*(L1 + L2*c3 + y/sin(yaw)))/(2*L2*s3_1))/(2*L2*s3_1 + (2*L1 + 2*L2*c3)^2/(2*L2*s3_1)));
c2_4 = sqrt((z + L2*s3_2 + ((2*L1 + 2*L2*c3)*(L1 + L2*c3 + y/sin(yaw)))/(2*L2*s3_2))/(2*L2*s3_2 + (2*L1 + 2*L2*c3)^2/(2*L2*s3_2)));

% Calculation of s2 for four cases
s2_1 = sqrt(1 - c2_1^2);
s2_2 = sqrt(1 - c2_2^2);
s2_3 = -sqrt(1 - c2_1^2);
s2_4 = -sqrt(1 - c2_2^2);

% Calculation of q2 for eight cases
q2_1 = atan2(s2_1, c2_1);
q2_2 = atan2(s2_1, c2_3);
q2_3 = atan2(s2_2, c2_2);
q2_4 = atan2(s2_2, c2_4);
q2_5 = atan2(s2_3, c2_1);
q2_6 = atan2(s2_3, c2_3);
q2_7 = atan2(s2_4, c2_2);
q2_8 = atan2(s2_4, c2_4);

% Definition of Q Matrix
Q_1 = [yaw; q2_1; q3_1; pitch - q2_1 - q3_1];
Q_2 = [yaw; q2_2; q3_1; pitch - q2_2 - q3_1];
Q_3 = [yaw; q2_4; q3_2; pitch - q2_3 - q3_2];
Q_4 = [yaw; q2_4; q3_2; pitch - q2_4 - q3_2];
Q_5 = [yaw; q2_5; q3_1; pitch - q2_5 - q3_1];
Q_6 = [yaw; q2_6; q3_1; pitch - q2_6 - q3_1];
Q_7 = [yaw; q2_7; q3_2; pitch - q2_7 - q3_2];
Q_8 = [yaw; q2_8; q3_2; pitch - q2_8 - q3_2];

Qs = [Q_1'; Q_2'; Q_3', Q_4'; Q_5'; Q_6'; Q_7'; Q_8'];

% Compute Poses and Costs
Xs = zeros(8, 6);
Js = zeros(8, 1);
for i = 1 : nsol
    Xs(i, :) = double(subs(X_Tsym, [q1, q2, q3, q4], [Qsym(1), Qsym(2), Qsym(3), Qsym(4)]));
    Js(i) = norm(Xs(i, :)' - X0);
end

% Return Q which minimizes Cost
[~, bestJ] = min(Js);
Q = Qs(bestJ, :)';

end
