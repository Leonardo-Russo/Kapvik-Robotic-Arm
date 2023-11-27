function [omega, omegad, vd, vcd] = velAcc(R, omega0, omega0d, qd, qdd, v0d, z, P, Pc)
% This function computes linear and angular velocities and their
% derivatives for each link:
% INPUT:
% R = rotation matrix from reference frame i to reference frame i+1
% omega0 = angular velocity of reference frame i wrt base frame, expressed
%          in the reference frame i
% omega0d = derivative of angular velocity of reference frame i  wrt base frame,
%          expressed in the reference frame i
% qd = angular rate of joint variable i+1
% qdd = derivative of angular rate of joint variable i+1
% z = z-axis of reference frame i+1 expressed in reference frame i+1 (i.e. [0 0 1]')
% v0d = derivative of linear velocity of reference frame i wrt base frame, 
%         expressed in reference frame i
% P = position of reference frame i+1 wrt reference frame i, expressed in 
%     the reference frame i 
% Pc = position of CDM of the link i+1 expressed in the reference frame of i+1
% OUTPUT:
% omega = angular velocity of reference i+1 wrt base frame, expressed in the
%         reference frame i+1
% omegad = derivative of angular velocity of reference frame  i+1 wrt base
%            frame expressed in the reference frame i+1
% vd =  derivative of linear velocity of reference frame i+1 wrt base frame,
%         expressed in the reference frame i+1
% vcd = derivative of linear velocity of CDM of the link i+1 expressed in
%         the reference frame i+1

omega=R*omega0+qd*z;
omegad=R*omega0d+R*cross(omega0,qd*z)+qdd*z;
vd=R*(cross(omega0d,P)+cross(omega0,cross(omega0,P))+v0d);
vcd=cross(omegad,Pc)+cross(omega,cross(omega,Pc))+vd;

end