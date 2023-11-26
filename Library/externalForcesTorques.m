function [F, N] = externalForcesTorques(m, vcd, omega, omegad, I)

% This function computes external forces and torques as a function of
% linear and angular velocities and acceleration:
% INPUT:
% m = mass of link
% vcd = derivative of linear velocity of CDM of the link i+1 expressed in
%         the reference frame i+1
% omega = angular velocity of reference i+1 wrt base frame, expressed in the
%         reference frame i+1
% omegad = derivative of angular velocity of reference frame i+1 wrt base
%            frame expressed in the reference frame i+1
% I = inertia tensor of link i+1 wrt CDM, expressed in the principal frame
%     of the link i+1 (i.e. the reference frame i+1 traslated to the CDM)
% OUTPUT:
% F = external forces applied to the CDM of link i+1, expressed in the
%     reference frame i+1 (computed as a function of v and omega)
% N = external torques wrt the CDM of link i+1, expressed in the
%     reference frame i+1 (computed as a function of v and omega)

F=m*vcd;
N=I*omegad+cross(omega, I*omega);

end