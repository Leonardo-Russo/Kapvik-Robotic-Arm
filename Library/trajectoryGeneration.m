function [t, q, qd, qdd] = trajectoryGeneration(q0, Q, thetaddMAX, T, N, ft)

% This function computes the trajectory from q0 to qf with trapezoidal
% method
% INPUT:
% q0 = vector 4x1 of initial joints angles [rad]
% Q = vector 4xN-1 of intermediate and final joint angles 
% thetaddMAX = MAX angular accelleration of motor [rad/s^2]
% T = total time from stowage to Navigation [s]
% N = N-2 are the number of via points
% ftVIApoints = % path update rate [Hz]
% OUTPUT:
% t = vector of times [s]
% q = matrix where each columns are the vector of joints angles at each
%     time [rad]
% qd = matrix where each columns are the vector of joints velocities at each
%      time [rad/s]
% qdd = matrix where each columns are the vector of joints acceleration at
%      each time [rad/s^2]

%% Accelleration of every joint
thetadd=0.9*thetaddMAX;

%% Joints angles @via points (angles on via points are not real joint angles at each time)
THETA=[q0, Q];

%% Times
dt=T/N;
ftVIApoints=ft/N;

%% First segment
thetadd1=sign(THETA(:,2)-q0)*abs(thetadd); % vector 4x1 of joint accelleration at initial time (in the first semi-parabolic reagion)
t1=dt-sqrt((dt^2)-((2*(THETA(2,2)-q0(1))/thetadd1(2)))); % first blending period (the same for each joint)
t2=t1; % second blending period = first blending period (the same for each joint)
theta12d=(THETA(:,2)-q0)/(dt-(t1/2)); % vector 4x1 of joint velocity (in the first linear reagion)
t12=dt-t1-(t2/2); % first linear period (the same for each joint)

% Preallocation 
q=ones(4,ftVIApoints);
qd=ones(4,ftVIApoints);
qdd=ones(4,ftVIApoints);
% Initial condition (middle of first parabolic part)
q(:,1)=q0;
qd(:,1)=theta12d;
qdd(:,1)=thetadd1;

% First second-half parabolic part
tENDpar1=t1/2;
t=linspace(0,tENDpar1,ftVIApoints);
for i=1:length(t(1,:))-1
    q(:,i+1)=q(:,i)+theta12d*((t(i+1)-t(i))/2)+(thetadd1/2)*((t(i+1)-t(i))/2)^2;
    qd(:,i+1)=theta12d+thetadd1*((t(i+1)-t(i))/2);
    qdd(:,i+1)=thetadd1;
end

% First linar part
tENDlin1=t12;
t=[t t(1,end)+linspace(0,tENDlin1,ftVIApoints)];
for i=length(qdd(1,:)):length(t)-1
    q(:,i+1)=q(:,i)+theta12d*(t(i+1)-t(i));
    qd(:,i+1)=theta12d;
    qdd(:,i+1)=0;
end

%% Last segment (part 1)
thetaddEND=sign(THETA(:,end-1)-qf)*abs(thetadd); % vector 4x1 of joint accelleration at final time (in the last semi-parabolic reagion)
tEND=dt-sqrt((dt^2)-((2*(THETA((1),end-1)-qf(1))/thetaddEND(1)))); % last blending period
tEND_1=tEND; % second-last period = last blending period
thetaEND_1_ENDd=(THETA(:,2)-qf)/(dt-(tEND/2)); % vector 4x1 of joint velocity (in the last linear reagion)
tEND_1_END=dt-tEND-(tEND_1/2); % last linear period 

%% Internal points
% loop on via points
thetaJKd=ones(4,N-2); % preallocation
thetaddK=ones(4,N-2); % preallocation
for j=1:N-2 
    thetaJKd(:,j)=(THETA(:,j+2)-THETA(:,j+1))/dt; % vector 4xN-2 of joint velocities (linear velocity beetween joint J and joint K at time j)
end
for j=1:N-2
    if j==N-2
        thetaddK(:,j)=sign(thetaEND_1_ENDd-thetaJKd(:,j))*abs(thetadd); % vector 4xN-2 of K-ith joint accelleration at time j (parabolic reagion)
        tK=(thetaEND_1_ENDd(2)-thetaJKd(2,j))/thetaddK(2,j); % blending period of joint K
        tJ=tK; % blending period of joint J
        tJK=dt+(tK/2)+(tJ/2); % linear period
    else
        thetaddK(:,j)=sign(thetaJKd(:,j+1)-thetaJKd(:,j))*abs(thetadd); % vector 4xN-2 of K-ith joint accelleration at time j (parabolic reagion)
        tK=(thetaJKd(2,j+1)-thetaJKd(2,j))/thetaddK(2,j); % blending period of joint K
        tJ=tK; % blending period of joint J
        tJK=dt+(tK/2)+(tJ/2); % linear period
    end
    if j~=N+2
        % Parabolic part
        tENDpar=tK;
        t=[t t(1,end)+linspace(0,tENDpar,ftVIApoints)];
        for i=length(qdd(1,:)):length(t(1,:))-1
            q(:,i+1)=q(:,i)+thetaJKd(:,j)*((t(i+1)-t(i))/2)+(thetaddK(:,j)/2)*((t(i+1)-t(i))/2)^2;
            qd(:,i+1)=thetaJKd(:,j)+thetaddK(:,j)*((t(i+1)-t(i))/2);
            qdd(:,i+1)=thetaddK(:,j);
        end
        % Linear part
        tENDlin=tJK;
        t=[t t(1,end)+linspace(0,tENDlin,ftVIApoints)];
        for i=length(qdd(1,:)):length(t(1,:))-1
            q(:,i+1)=q(:,i)+thetaJKd(:,j)*(t(i+1)-t(i));
            qd(:,i+1)=thetaJKd(:,j);
            qdd(:,i+1)=0;
        end
    else
        % Second-last first half parabolic part
        tENDparSec=tJ/2;
        t=[t(1,:) t(1,end)+linspace(0,tENDparSec,ftVIApoints)];
        for i=length(qdd(1,:)):length(t(1,:))-1
            q(:,i+1)=q(:,i)+thetaJKd(:,j)*((t(i+1)-t(i))/2)+(thetaddK(:,j)/2)*((t(i+1)-t(i))/2)^2;
            qd(:,i+1)=thetaJKd(:,j)+thetaddK(:,j)*((t(i+1)-t(i))/2);
            qdd(:,i+1)=thetaddK(:,j);
        end
    end
end

%% Last segment (part 2)

% Second-last second-half parabolic part
tENDparEND_1=tEND_1/2;
t=[t t(1,:)+linspace(0,tENDparEND_1,ftVIApoints)];
for i=length(qdd(1,:))+1:length(t(1,:))
    q(:,i+1)=q(:,i)+thetaEND_1_ENDd*((t(i+1)-t(i))/2)+(thetaddK(:,N-2)/2)*((t(i+1)-t(i))/2)^2;
    qd(:,i+1)=thetaEND_1_ENDd+thetaddK(:,N-2)*((t(i+1)-t(i))/2);
    qdd(:,i+1)=thetaddK(:,N-2);
end

% Last linar part
tENDlinEND=tEND_1_END;
t=[t t(1,end)+linspace(0,tENDlinEND,ftVIApoints)];
for i=length(qdd(1,:))+1:length(t(1,:))
    q(:,i+1)=q(:,i)+thetaEND_1_ENDd*(t(i+1)-t(i));
    qd(:,i+1)=thetaEND_1_ENDd;
    qdd(:,i+1)=0;
end

% Last first-half parabolic part
tENDparEND=tEND/2;
t=[t t(1,end)+linspace(0,tENDparEND,ftVIApoints)];
for i=length(qdd(1,:))+1:length(t(1,:))
    q(:,i+1)=q(:,i)+thetaEND_1_ENDd*((t(i+1)-t(i))/2)+(thetaddEND/2)*((t(i+1)-t(i))/2)^2;
    qd(:,i+1)=thetaEND_1_ENDd+thetaddEND*((t(i+1)-t(i))/2);
    qdd(:,i+1)=thetaddEND;
end

end