function [q234] = RPY2angle(roll, pitch, yaw)
%This function compute the angle q1+q2+q3 (from roll pitch and yaw, so it
% is the sequence 3-2-1, i.e. R=R1(roll)*R2(pitch)*R3(yaw), to the sequence
% 3-1-3, i.e. R=R3(spinn=q_234)*R1(nutation)*R3(precession))


R=R1(roll)*R2(pitch)*R3(yaw);

% syms q_234 nutation precession 
% Rsym=R3(q_234)*R1(nutation)*R3(precession)

theta=acos(R(3,3));
s_q234=R(1,3)/sin(theta);
c_q234=R(2,3)/sin(theta);
q234=atan2(s_q234,c_q234);
end