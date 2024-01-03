function [t, q, qd, qdd] = trajectoryGenerationSto2Nav(q0, qinter1, qinter2, qf, thetaddMAX, T, ft)

% This function computes the trajectory from q0 to qf with trapezoidal
% method
% INPUT:
% q0 = vector 4x1 of initial joints angles [rad]
% qinter1 = vector 4x1 joint angles at first intermediate point
% qinter2 = vector 4x1 joint angles at first intermediate point
% qf = vector 4x1 of final joint angles 
% thetaddMAX = MAX angular accelleration of motor [rad/s^2]
% T = total time from Stowage to Navigation [s]
% ft = path update rate [Hz] (1/timestep)
% OUTPUT:
% t = vector of times [s] for joint, angles, velocities and acceleration in
% the first part
% t = vector of times
% q = vector 4xN of joints angles at each time [rad]
% qd = vector 4xN of joints velocities at each time [rad/s]
% qdd = vector 4xN of joints acceleration at each time [rad/s^2]

%% Accelleration of every joint
thetadd=0.9*thetaddMAX;

%% Times
dt1=0.7*T; % time from start point 1 to the intermediate point 1
dt2=0.1*T; % time from the intermediate point 1 to the intermediate point 2
dt3=0.2*T; % time from intermediate point 2 to final point 3
timestep=1/ft;

%% parabola(1)-linear-parabola(2)-linear-parabola(3)-linear-parabola(4)
% 1-->Starting point
% 2-->Intermediate point 1
% 3-->Intermediate point 2
% 4-->Ending point

thetadd1=sign(qinter1(2)-q0(2))*abs(thetadd(2)); % second joint accelleration in the first parabolic reagion
thetadd4=sign(qinter2(2:3)-qf(2:3)).*abs(thetadd(2:3)'); % second and third joint accelleration in the fourth and last parabolic reagion

t1=dt1-sqrt((dt1^2)-((2*(qinter1(2)-q0(2))/thetadd1))); % first blending period  
t4=dt3-sqrt((dt3^2)-((2*(qinter2(2:3)-qf(2:3))./thetadd4))); % fourth and last blending period 

thetad12=(qinter1(2)-q0(2))/(dt1-(t1/2)); % second joint velocity in the first linear reagion
thetad23=(qinter2(2:3)-qinter1(2:3))/dt2; % second and third joint velocities in the second linear reagion
thetad34=(qf(2:3)-qinter2(2:3))./(dt3-(t4/2)); % second and third joint velocities in the third linear reagion

thetadd2=sign(thetad23-[thetad12 0]').*abs(thetadd(2:3)'); % second and third joint accelleration in the second parabolic reagion
thetadd3=sign(thetad34-thetad23).*abs(thetadd(2:3)'); % second and third joint accelleration in the third parabolic reagion

t2=(thetad23-[thetad12 0]')./thetadd2; % second blending period
t3=(thetad34-thetad23)./thetadd3; % third blending period 

t12=dt1-t1-(t2/2); % first linear period 
t23=dt2-(t2/2)-(t3/2); % second linear period
t34=dt3-t4-(t3/2); % third linear period

%% First part
% Times vectors
tENDpar1=t1;
tpar1=0:timestep:tENDpar1;
tENDlin1=t12;
tlin1=[tpar1(1:end-1) tpar1(1,end)+(0:timestep:tENDlin1)];
tENDfirstSemiPar2=t2/2;
tq2_1=[tlin1(1:end-1) tlin1(1,end)+(0:timestep:tENDfirstSemiPar2(1))];
tq3_1=[tlin1(1:end-1) tlin1(1,end)+(0:timestep:tENDfirstSemiPar2(2))];

% Preallocation 
q2part1=ones(1,length(tq2_1));  
qd2part1=ones(1,length(tq2_1));
qdd2part1=ones(1,length(tq2_1)); 
q3part1=ones(1,length(tq3_1));  
qd3part1=ones(1,length(tq3_1));
qdd3part1=ones(1,length(tq3_1)); 

% Initial condition 
q2part1(1)=q0(2); 
qd2part1(1)=0; 
qdd2part1(1)=thetadd1;
q3part1(1)=q0(3); 
qd3part1(1)=0; 
qdd3part1(1)=0;

% First parabola
for i=1:length(tpar1)-1
    q2part1(i+1)=q2part1(i)+qd2part1(i)*(tq2_1(i+1)-tq2_1(i))+(thetadd1/2)*(tq2_1(i+1)-tq2_1(i))^2;
    qd2part1(i+1)=qd2part1(i)+thetadd1*(tq2_1(i+1)-tq2_1(i));
    qdd2part1(i+1)=thetadd1;
    q3part1(i+1)=q0(3);
    qd3part1(i+1)=0;
    qdd3part1(i+1)=0;
end

% First linear reagion
for i=length(tpar1):length(tlin1)-1
    q2part1(i+1)=q2part1(i)+thetad12*(tq2_1(i+1)-tq2_1(i));
    qd2part1(i+1)=thetad12;
    qdd2part1(i+1)=0;
    q3part1(i+1)=q0(3);
    qd3part1(i+1)=0;
    qdd3part1(i+1)=0;
end

% Second parabola (first half parabola)
for i=length(tlin1):length(tq2_1)-1
    q2part1(i+1)=q2part1(i)+qd2part1(i)*(tq2_1(i+1)-tq2_1(i))+(thetadd2(1)/2)*(tq2_1(i+1)-tq2_1(i))^2;
    qd2part1(i+1)=qd2part1(i)+thetadd2(1)*(tq2_1(i+1)-tq2_1(i));
    qdd2part1(i+1)=thetadd2(1);
end
for i=length(tlin1):length(tq3_1)-1
    q3part1(i+1)=q3part1(i)+qd3part1(i)*(tq3_1(i+1)-tq3_1(i))+(thetadd2(2)/2)*(tq3_1(i+1)-tq3_1(i))^2;
    qd3part1(i+1)=qd3part1(i)+thetadd2(2)*(tq3_1(i+1)-tq3_1(i));
    qdd3part1(i+1)=thetadd2(2);
end

%% Second part
% Times vectors
tENDsecondSemiPar2=t2/2;
tq2par2=(0:timestep:tENDsecondSemiPar2(1));
tq3par2=(0:timestep:tENDsecondSemiPar2(2));
tENDlin2=t23;
tq2lin2=[tq2par2(1:end-1) tq2par2(1,end)+(0:timestep:tENDlin2(1))];
tq3lin2=[tq3par2(1:end-1) tq3par2(1,end)+(0:timestep:tENDlin2(2))];
tENDfirstSemiPar3=t3/2;
tq2=[tq2lin2(1:end-1) tq2lin2(1,end)+(0:timestep:tENDfirstSemiPar3(1))];
tq3=[tq3lin2(1:end-1) tq3lin2(1,end)+(0:timestep:tENDfirstSemiPar3(2))];

% Preallocation
q2Part2=ones(1,length(tq2)); 
qd2Part2=ones(1,length(tq2)); 
qdd2Part2=ones(1,length(tq2));
q3Part2=ones(1,length(tq3));
qd3Part2=ones(1,length(tq3));
qdd3Part2=ones(1,length(tq3));

% Initial condition
q2Part2(1)=q2part1(end); 
qd2Part2(1)=qd2part1(end); 
qdd2Part2(1)=qdd2part1(end); 
q3Part2(1)=q3part1(end); 
qd3Part2(1)=qd3part1(end); 
qdd3Part2(1)=qdd3part1(end); 

% Second parabola (second half parabola)
for i=1:length(tq2par2)-1
    q2Part2(i+1)=q2Part2(1,i)+qd2Part2(1,i)*(tq2(i+1)-tq2(i))+(thetadd2(1)/2)*(tq2(i+1)-tq2(i))^2;
    qd2Part2(i+1)=qd2Part2(1,i)+thetadd2(1)*(tq2(i+1)-tq2(i));
    qdd2Part2(i+1)=thetadd2(1);
end
for i=1:length(tq3par2)-1
    q3Part2(i+1)=q3Part2(1,i)+qd3Part2(1,i)*(tq3(i+1)-tq3(i))+(thetadd2(2)/2)*(tq3(i+1)-tq3(i))^2;
    qd3Part2(i+1)=qd3Part2(1,i)+thetadd2(2)*(tq3(i+1)-tq3(i));
    qdd3Part2(i+1)=thetadd2(2);
end

% Second linear reagion
for i=length(tq2par2):length(tq2lin2)-1
    q2Part2(i+1)=q2Part2(1,i)+thetad23(1)*(tq2(i+1)-tq2(i));
    qd2Part2(i+1)=thetad23(1);
    qdd2Part2(i+1)=0;
end
for i=length(tq3par2):length(tq3lin2)-1
    q3Part2(i+1)=q3Part2(1,i)+thetad23(2)*(tq3(i+1)-tq3(i));
    qd3Part2(i+1)=thetad23(2);
    qdd3Part2(i+1)=0;
end

% Third parabola (first half)
for i=length(tq2lin2):length(tq2)-1
    q2Part2(i+1)=q2Part2(i)+qd2Part2(i)*(tq2(i+1)-tq2(i))+(thetadd3(1)/2)*(tq2(i+1)-tq2(i))^2;
    qd2Part2(i+1)=qd2Part2(i)+thetadd3(1)*(tq2(i+1)-tq2(i));
    qdd2Part2(i+1)=thetadd3(1);
end
for i=length(tq3lin2):length(tq3)-1
    q3Part2(i+1)=q3Part2(i)+qd3Part2(1,i)*(tq3(i+1)-tq3(i))+(thetadd3(2)/2)*(tq3(i+1)-tq3(i))^2;
    qd3Part2(i+1)=qd3Part2(i)+thetadd3(2)*(tq3(i+1)-tq3(i));
    qdd3Part2(i+1)=thetadd3(2);
end

% Collect times
Tq2=[tq2_1(1:end-1) tq2_1(end)+tq2];
Tq3=[tq3_1(1:end-1) tq3_1(end)+tq3];

% Remove first component (because it is the last component of part 1)
q2Part2(1)=[];
qd2Part2(1)=[];
qdd2Part2(1)=[];
q3Part2(1)=[];
qd3Part2(1)=[];
qdd3Part2(1)=[];

%% Third part
% Times vectors
tENDfirstSemiPar3=t3/2;
tq2par3=(0:timestep:tENDfirstSemiPar3(1));
tq3par3=(0:timestep:tENDfirstSemiPar3(2));
tENDlin3=t34;
tq2lin3=[tq2par3(1:end-1) tq2par3(1,end)+(0:timestep:tENDlin3(1))];
tq3lin3=[tq3par3(1:end-1) tq3par3(1,end)+(0:timestep:tENDlin3(2))];
tENDpar4=t4;
tq2=[tq2lin3(1:end-1) tq2lin3(1,end)+(0:timestep:tENDpar4(1))];
tq3=[tq3lin3(1:end-1) tq3lin3(1,end)+(0:timestep:tENDpar4(2))];

% Preallocation
q2Part3=ones(1,length(tq2)); 
qd2Part3=ones(1,length(tq2)); 
qdd2Part3=ones(1,length(tq2));
q3Part3=ones(1,length(tq3));
qd3Part3=ones(1,length(tq3));
qdd3Part3=ones(1,length(tq3));

% Initial condition
q2Part3(1)=q2Part2(end); 
qd2Part3(1)=qd2Part2(end); 
qdd2Part3(1)=thetadd3(1); 
q3Part3(1)=q3Part2(end); 
qd3Part3(1)=qd3Part2(end); 
qdd3Part3(1)=thetadd3(2); 

% Third parabola (second half parabola)
for i=1:length(tq2par3)-1
    q2Part3(i+1)=q2Part3(1,i)+qd2Part3(1,i)*(tq2(i+1)-tq2(i))+(thetadd3(1)/2)*(tq2(i+1)-tq2(i))^2;
    qd2Part3(i+1)=qd2Part3(1,i)+thetadd3(1)*(tq2(i+1)-tq2(i));
    qdd2Part3(i+1)=thetadd3(1);
end
for i=1:length(tq3par3)-1
    q3Part3(i+1)=q3Part3(1,i)+qd3Part3(1,i)*(tq3(i+1)-tq3(i))+(thetadd3(2)/2)*(tq3(i+1)-tq3(i))^2;
    qd3Part3(i+1)=qd3Part3(1,i)+thetadd3(2)*(tq3(i+1)-tq3(i));
    qdd3Part3(i+1)=thetadd3(2);
end

% Third linear reagion
for i=length(tq2par3):length(tq2lin3)-1
    q2Part3(i+1)=q2Part3(1,i)+thetad34(1)*(tq2(i+1)-tq2(i));
    qd2Part3(i+1)=thetad34(1);
    qdd2Part3(i+1)=0;
end
for i=length(tq3par3):length(tq3lin3)-1
    q3Part3(i+1)=q3Part3(1,i)+thetad34(2)*(tq3(i+1)-tq3(i));
    qd3Part3(i+1)=thetad34(2);
    qdd3Part3(i+1)=0;
end

% Fourth and last parabola
for i=length(tq2lin3):length(tq2)-1
    q2Part3(i+1)=q2Part3(1,i)+qd2Part3(1,i)*(tq2(i+1)-tq2(i))+(thetadd4(1)/2)*(tq2(i+1)-tq2(i))^2;
    qd2Part3(i+1)=qd2Part3(1,i)+thetadd4(1)*(tq2(i+1)-tq2(i));
    qdd2Part3(i+1)=thetadd4(1);
end
for i=length(tq3lin3):length(tq3)-1
    q3Part3(i+1)=q3Part3(1,i)+qd3Part3(1,i)*(tq3(i+1)-tq3(i))+(thetadd4(2)/2)*(tq3(i+1)-tq3(i))^2;
    qd3Part3(i+1)=qd3Part3(1,i)+thetadd4(2)*(tq3(i+1)-tq3(i));
    qdd3Part3(i+1)=thetadd4(2);
end

% Remove first component (because it is the last component of part 2)
q2Part3(1)=[];
qd2Part3(1)=[];
qdd2Part3(1)=[];
q3Part3(1)=[];
qd3Part3(1)=[];
qdd3Part3(1)=[];

%% Final times vectors
Tq2=[Tq2(1,1:end-1) Tq2(1,end)+tq2];
Tq3=[Tq3(1,1:end-1) Tq3(1,end)+tq3];

%% Unique time vector
t=[Tq2(1:end-1) (Tq2(end):timestep:T)];

%% Set the rimainder empty component of q, qd, qdd (Part 4)
q2Part4=q2Part3(end)*ones(1,length(t)-length(Tq2));
q3Part4=q3Part3(end)*ones(1,length(t)-length(Tq3));
qd2Part4=zeros(1,length(t)-length(Tq2));
qd3Part4=zeros(1,length(t)-length(Tq3));
qdd2Part4=zeros(1,length(t)-length(Tq2));
qdd3Part4=zeros(1,length(t)-length(Tq3));


%% Joint angles, velocities and accelerations
q1=q0(1)*ones(1,length(t));
q2=[q2part1 q2Part2 q2Part3 q2Part4];
q3=[q3part1 q3Part2 q3Part3 q3Part4];
q4=q0(4)*ones(1,length(t));
qd1=zeros(1,length(t));
qd2=[qd2part1 qd2Part2 qd2Part3 qd2Part4];
qd3=[qd3part1 qd3Part2 qd3Part3 qd3Part4];
qd4=zeros(1,length(t));
qdd1=zeros(1,length(t));
qdd2=[qdd2part1 qdd2Part2 qdd2Part3 qdd2Part4];
qdd3=[qdd3part1 qdd3Part2 qdd3Part3 qdd3Part4];
qdd4=zeros(1,length(t));

q=[q1; q2; q3; q4];
qd=[qd1; qd2; qd3; qd4];
qdd=[qdd1; qdd2; qdd3; qdd4];

end