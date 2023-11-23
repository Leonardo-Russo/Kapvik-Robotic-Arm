function [omega, omegaDot, vDot, vcDot] = velAcc(R, omega0, omega0Dot, qDot, qDDot, v0Dot, z, P, Pc)
% This function computes linear and angular velocities and their
% derivatives for each link:
% INPUT:
% R = rotation matrix from reference frame i to reference frame i+1
% omega0 = angular velocity of reference frame i wrt base frame, expressed
%          in the reference frame i
% omega0 = derivative of angular velocity of reference frame i  wrt base frame,
%          expressed in the reference frame i
% qDot = angular rate of joint variable i+1
% qDDot = derivative of angular rate of joint variable i+1
% z = z-axis of reference frame i+1 expressed in reference frame i+1 (i.e. [0 0 1]')
% v0Dot = derivative of linear velocity of reference frame i wrt base frame, 
%         expressed in reference frame i
% P = position of reference frame i+1 wrt reference frame i, expressed in 
%     the reference frame i 
% Pc = position of CDM of the link i+1 expressed in the reference frame of i+1
% OUTPUT:
% omega = angular velocity of reference i+1 wrt base frame, expressed in the
%         reference frame i+1
% omegaDot = derivative of angular velocity of reference frame  i+1 wrt base
%            frame expressed in the reference frame i+1
% vDot =  derivative of linear velocity of reference frame i+1 wrt base frame,
%         expressed in the reference frame i+1
% vcDot = derivative of linear velocity of CDM of the link i+1 expressed in
%         the reference frame i+1

omega=R*omega0+qDot*z;
omegaDot=R*omega0Dot+R*cross(omega0,qDot*z)+qDDot*z;
vDot=R*(cross(omega0Dot,P)+cross(omega0,cross(omega0,Pc))+v0Dot);
vcDot=cross(omegaDot,Pc)+cross(omega,cross(omega,Pc))+vDot;

end