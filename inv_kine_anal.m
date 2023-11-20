function inv_kine_anal()
% Description: analytical inverse kinematics.

syms q1 q2 q3 q4 real

yaw = q1;
pitch = q2 + q3 + q4;
roll = pi/2;

R = eye(3);

R = vpa(simplify(R1(roll)*R2(pitch)*R3(yaw)*R))';

disp(R)


end