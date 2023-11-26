function [f, n, tau] = jointForcesTorques(R, fEnd, nEnd, F, N, P, Pc, z)

% This function computes forces and torques on joints
% INPUT:
% R = rotation matrix from reference frame i+1 to reference frame i
% fEnd = forces on joint i+1, expressed in the reference frame i+1
% nEnd = torques on joint i+1, expressed in the reference frame i+1
% F = external forces applied to the CDM of link i, expressed in the
%     reference frame i (computed as a function of v and omega)
% N = external torques wrt the CDM of link i, expressed in the
%     reference frame i (computed as a function of v and omega)
% omegaDot = derivative of angular velocity of reference frame  i+1 wrt base
%            frame expressed in the reference frame i+1
% P = position of reference frame i+1 wrt reference frame i, expressed in 
%     the reference frame i 
% Pc = position of CDM of the link i expressed in the reference frame of i
% z = z-axis of reference frame of joint i expressed in reference frame
%     of joint i (i.e. [0 0 1]')
% OUTPUT:
% f = forces on joint i, expressed in the reference frame i
% n = theoric torques on joint i, expressed in the reference frame i (are 
%     includes also the component of unpheasible torque, which contributes
%     to the vincolat reaction torque at each joint, due to the fact that
%     the only permitted DOF is the one along the z-axis of the joint) 
% tau = practical torques on joint i, expressed in the reference frame of 
%       joint i (along z-axis of reference frame i)

f=R*fEnd+F;
n=N+R*nEnd+cross(Pc,F)+cross(P,R*fEnd);
tau=dot(n,z);

end