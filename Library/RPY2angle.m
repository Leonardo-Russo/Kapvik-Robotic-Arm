function [q234, prec] = RPY2angle(roll, pitch, yaw)
%This function compute the angle q1+q2+q3

R=R1(roll)*R2(pitch)*R3(yaw);

% syms q_234 ROLL YAW 
% Rsym=R3(spinn=q_234)*R1(nut)*R3(prec)

theta=acos(R(3,3));
s_q234=R(1,3)/sin(theta);
c_q234=R(2,3)/sin(theta);
q234=atan2(s_q234,c_q234);

s_prec=R(3,1)/sin(theta);
c_prec=-R(3,2)/sin(theta);
prec=atan2(s_prec,c_prec);

end