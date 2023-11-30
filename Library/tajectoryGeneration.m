function [t, q, qd, qdd] = tajectoryGeneration(q0, q0d, q0dd, qf, qfd, qfdd, dt)
% This function computes the trajectory from q0 to qf with trapezoidal
% method
% INPUT:
% q0 = vector 4x1 of initial joints angles
% q0dot = vector 4x1 of initial joints velocities
% qfd = vector 4x1 of initial joints accelerations (desidered)
% qf = vector 4x1 of final joints angles (desidered)
% qfd = vector 4x1 of final joints velocity (desidered)
% qfd = vector 4x1 of final joints accelerations (desidered)
% dt = delta t beetween via points (1/ft)
% OUTPUT:
% t = vector of times
% q = matrix where each columns are the vector of joints angles at each
% time
% qd = matrix where each columns are the vector of joints velocities at each
%      time
% qd = matrix where each columns are the vector of joints acceleration at
%      each time

%% Initial condition
theta10=q0(1);
theta20=q0(2);
theta30=q0(3);
theta40=q0(4);
theta10d=q0d(1);
theta20d=q0d(2);
theta30d=q0d(3);
theta40d=q0d(4);
theta10dd=q0dd(1);
theta20dd=q0dd(2);
theta30dd=q0dd(3);
theta40dd=q0dd(4);


%% Final condition
theta1f=qf(1);
theta2f=qf(2);
theta3f=qf(3);
theta4f=qf(4);
theta1fd=qfd(1);
theta2fd=qfd(2);
theta3fd=qfd(3);
theta4fd=qfd(4);
theta1fdd=qfdd(1);
theta2fdd=qfdd(2);
theta3fdd=qfdd(3);
theta4fdd=qfdd(4);

%% Joints angles at each time
theta1=linspace(theta10, theta1f, 100);
theta2=linspace(theta20, theta2f, 100);
theta3=linspace(theta30, theta3f, 100);
theta4=linspace(theta40, theta4f, 100);
q=[theta1; theta2; theta3; theta4];

%% First segment
theta1dd=ones(4,1); % preallocation
t1=ones(4,1); % preallocation
t2=ones(4,1); % preallocation
theta12d=ones(4,1); % preallocation
t12=ones(4,1); % preallocation
for j=1:4
    theta1dd(j)=sign(q(j,2)-q(j,1))*q0dd(j);
    t1(j)=dt-sqrt((dt^2)-((2*(q(j,2)-q(j,1)))/q0dd(j)));
    t2(j)=t1(j);
    theta12d(j)=(q(j,2)-q(j,1))/(dt-(t1(j)/2));
    t12(j)=dt-t1-(t2(j)/2);
end

%% Last segment
thetaNdd=ones(4,1); % preallocation
tN=ones(4,1); % preallocation
tN_1=ones(4,1); % preallocation
thetaN_1_Nd=ones(4,1); % preallocation
tN_1_N=ones(4,1); % preallocation
for j=1:4
    thetaNdd(j)=sign(q(j,end-1)-q(j,end))*qfdd(j);
    tN(j)=dt-sqrt((dt^2)-((2*(q(j,end-1)-q(j,end)))/qfdd(j)));
    tN_1(j)=t1(j);
    thetaN_1_Nd(j)=(q(j,end)-q(j,end-1))/(dt-(tN(j)/2));
    tN_1_N(j)=dt-tN-(tN_1(j)/2);
end
end