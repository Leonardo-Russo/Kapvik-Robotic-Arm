function [t, q, qd, qdd] = trajectoryGenerationSto2Ret(q0, qinter1, qinter2, qf, thetaddMAX, T, ft)

% This function computes the trajectory from q0 to qf with trapezoidal
% method
% INPUT:
% q0 = vector 4x1 of initial joints angles [rad]
% qinter1 = vector 4x1 joint angles at first intermediate point
% qinter2 = vector 4x1 joint angles at first intermediate point
% qf = vector 4x1 of final joint angles 
% thetaddMAX = MAX angular accelleration of motor [rad/s^2]
% T = total time from Stowage to Retrieval [s]
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
dt1=0.6*T; % time from start point 1 to the intermediate point 1
dt2=0.1*T; % time from the intermediate point 1 to the intermediate point 2
dt3=0.3*T; % time from intermediate point 2 to final point 3
timestep=1/ft;

%% parabola(1)-linear-parabola(2)-linear-parabola(3)-linear-parabola(4)
% 1-->Starting point
% 2-->Intermediate point 1
% 3-->Intermediate point 2
% 4-->Ending point

thetadd1(1,1)=sign(qinter1(1)-q0(1))*abs(thetadd(1)); % first joint accelleration in the first parabolic reagion
thetadd1(2,1)=sign(qinter1(2)-q0(2))*abs(thetadd(2)); % second joint accelleration in the first parabolic reagion
thetadd1(3,1)=sign(qinter1(4)-q0(4))*abs(thetadd(4)); % fourth joint accelleration in the first parabolic reagion
thetadd4=sign(qinter2(2:3)-qf(2:3)).*abs(thetadd(2:3)); % second and third joint accelleration in the fourth an last parabolic reagion

t1(1,1)=dt1-sqrt((dt1^2)-((2*(qinter1(1)-q0(1))/thetadd1(1,1)))); % first blending period  of first joint
t1(2,1)=dt1-sqrt((dt1^2)-((2*(qinter1(2)-q0(2))/thetadd1(2,1)))); % first blending period  of second joint
t1(3,1)=dt1-sqrt((dt1^2)-((2*(qinter1(4)-q0(4))/thetadd1(3,1)))); % first blending period  of fourth joint
t4=dt3-sqrt((dt3^2)-((2*(qinter2(2:3)-qf(2:3))./thetadd4))); % fourth and last blending period of second and third joint

thetad12(1,1)=(qinter1(1)-q0(1))/(dt1-(t1(1)/2)); % first joint velocity in the first linear reagion
thetad12(2,1)=(qinter1(2)-q0(2))/(dt1-(t1(2)/2)); % second joint velocity in the first linear reagion
thetad12(3,1)=(qinter1(4)-q0(4))/(dt1-(t1(3)/2)); % fourth joint velocity in the first linear reagion

thetad23=(qinter2(2:3)-qinter1(2:3))/dt2; % second and third joint velocities in the second linear reagion

thetad34=(qf(2:3)-qinter2(2:3))./(dt3-(t4/2)); % second and third joint velocity in the third linear reagion

thetadd2=sign([0; thetad23; 0]-[thetad12(1) thetad12(2) 0 thetad12(3)]').*abs(thetadd); % first, second, third and fourth joint accelleration in the second parabolic reagion
thetadd3=sign(thetad34-thetad23).*abs(thetadd(2:3)); % second and third joint accelleration in the third parabolic reagion

t2=([0; thetad23; 0]-[thetad12(1) thetad12(2) 0 thetad12(3)]')./thetadd2; % second blending period (of first, second, third and fourth joint)
t3=(thetad34-thetad23)./thetadd3; % third blending period (of second and third joint)

t12=dt1-[t1(1) t1(2) t1(3)]'-([t2(1) t2(2) t2(3)]'/2); % first linear period (of first, second and fourth joint)
t23=dt2-([t2(2) t2(3)]'/2)-(t3/2); % second linear period of second and third joint
t34=dt3-t4-(t3/2); % third linear period of second and third joint

%% First part (q1, q2, q3 and q4 change)
% Times vectors
tENDpar1=t1;
tpar1q1_1=0:timestep:tENDpar1(1);
tpar1q2_1=0:timestep:tENDpar1(2);
tpar1q4_1=0:timestep:tENDpar1(3);
tENDlin1=t12;
tlin1q1_1=[tpar1q1_1(1:end-1) tpar1q1_1(1,end)+(0:timestep:tENDlin1(1))];
tlin1q2_1=[tpar1q2_1(1:end-1) tpar1q2_1(1,end)+(0:timestep:tENDlin1(2))];
tlin1q4_1=[tpar1q4_1(1:end-1) tpar1q4_1(1,end)+(0:timestep:tENDlin1(3))];
tENDfirstSemiPar2=t2/2;
tq1_1=[tlin1q1_1(1:end-1) tlin1q1_1(1,end)+(0:timestep:tENDfirstSemiPar2(1))];
tq2_1=[tlin1q2_1(1:end-1) tlin1q2_1(1,end)+(0:timestep:tENDfirstSemiPar2(2))];
tq3_1=[tlin1q2_1(1:end-1) tlin1q2_1(1,end)+(0:timestep:tENDfirstSemiPar2(3))];
tq4_1=[tlin1q4_1(1:end-1) tlin1q4_1(1,end)+(0:timestep:tENDfirstSemiPar2(4))];

% Preallocation 
q1part1=ones(1,length(tq1_1));  
qd1part1=ones(1,length(tq1_1));
qdd1part1=ones(1,length(tq1_1)); 
q2part1=ones(1,length(tq2_1));  
qd2part1=ones(1,length(tq2_1));
qdd2part1=ones(1,length(tq2_1)); 
q3part1=ones(1,length(tq3_1));  
qd3part1=ones(1,length(tq3_1));
qdd3part1=ones(1,length(tq3_1));
q4part1=ones(1,length(tq4_1));  
qd4part1=ones(1,length(tq4_1));
qdd4part1=ones(1,length(tq4_1));

% Initial condition 
q1part1(1)=q0(1); 
qd1part1(1)=0; 
qdd1part1(1)=thetadd1(1);
q2part1(1)=q0(2); 
qd2part1(1)=0; 
qdd2part1(1)=thetadd1(2);
q3part1(1)=q0(3); 
qd3part1(1)=0; 
qdd3part1(1)=0;
q4part1(1)=q0(4); 
qd4part1(1)=0; 
qdd4part1(1)=thetadd1(3);

% First parabola
for i=1:length(tpar1q1_1)-1
    q1part1(i+1)=q1part1(i)+qd1part1(i)*(tq1_1(i+1)-tq1_1(i))+(thetadd1(1)/2)*(tq1_1(i+1)-tq1_1(i))^2;
    qd1part1(i+1)=qd1part1(i)+thetadd1(1)*(tq1_1(i+1)-tq1_1(i));
    qdd1part1(i+1)=thetadd1(1);
end
for i=1:length(tpar1q2_1)-1
    q2part1(i+1)=q2part1(i)+qd2part1(i)*(tq2_1(i+1)-tq2_1(i))+(thetadd1(2)/2)*(tq2_1(i+1)-tq2_1(i))^2;
    qd2part1(i+1)=qd2part1(i)+thetadd1(2)*(tq2_1(i+1)-tq2_1(i));
    qdd2part1(i+1)=thetadd1(2);
    q3part1(i+1)=q0(3);
    qd3part1(i+1)=0;
    qdd3part1(i+1)=0;
end
for i=1:length(tpar1q4_1)-1
    q4part1(i+1)=q4part1(i)+qd4part1(i)*(tq4_1(i+1)-tq4_1(i))+(thetadd1(3)/2)*(tq4_1(i+1)-tq4_1(i))^2;
    qd4part1(i+1)=qd4part1(i)+thetadd1(3)*(tq4_1(i+1)-tq4_1(i));
    qdd4part1(i+1)=thetadd1(3);
end

% First linear reagion
for i=length(tpar1q1_1):length(tlin1q1_1)-1
    q1part1(i+1)=q1part1(i)+thetad12(1)*(tq1_1(i+1)-tq1_1(i));
    qd1part1(i+1)=thetad12(1);
    qdd1part1(i+1)=0;
end
for i=length(tpar1q2_1):length(tlin1q2_1)-1
    q2part1(i+1)=q2part1(i)+thetad12(2)*(tq2_1(i+1)-tq2_1(i));
    qd2part1(i+1)=thetad12(2);
    qdd2part1(i+1)=0;
    q3part1(i+1)=q0(3);
    qd3part1(i+1)=0;
    qdd3part1(i+1)=0;
end
for i=length(tpar1q4_1):length(tlin1q4_1)-1
    q4part1(i+1)=q4part1(i)+thetad12(3)*(tq4_1(i+1)-tq4_1(i));
    qd4part1(i+1)=thetad12(3);
    qdd4part1(i+1)=0;
end

% Second parabola (first half parabola)
for i=length(tlin1q1_1):length(tq1_1)-1
    q1part1(i+1)=q1part1(i)+qd1part1(i)*(tq1_1(i+1)-tq1_1(i))+(thetadd2(1)/2)*(tq1_1(i+1)-tq1_1(i))^2;
    qd1part1(i+1)=qd1part1(i)+thetadd2(1)*(tq1_1(i+1)-tq1_1(i));
    qdd1part1(i+1)=thetadd2(1);
end
for i=length(tlin1q2_1):length(tq2_1)-1
    q2part1(i+1)=q2part1(i)+qd2part1(i)*(tq2_1(i+1)-tq2_1(i))+(thetadd2(2)/2)*(tq2_1(i+1)-tq2_1(i))^2;
    qd2part1(i+1)=qd2part1(i)+thetadd2(2)*(tq2_1(i+1)-tq2_1(i));
    qdd2part1(i+1)=thetadd2(2);
end
for i=length(tlin1q2_1):length(tq3_1)-1
    q3part1(i+1)=q3part1(i)+qd3part1(i)*(tq3_1(i+1)-tq3_1(i))+(thetadd2(3)/2)*(tq3_1(i+1)-tq3_1(i))^2;
    qd3part1(i+1)=qd3part1(i)+thetadd2(3)*(tq3_1(i+1)-tq3_1(i));
    qdd3part1(i+1)=thetadd2(3);
end
for i=length(tlin1q4_1):length(tq4_1)-1
    q4part1(i+1)=q4part1(i)+qd4part1(i)*(tq4_1(i+1)-tq4_1(i))+(thetadd2(4)/2)*(tq4_1(i+1)-tq4_1(i))^2;
    qd4part1(i+1)=qd4part1(i)+thetadd2(4)*(tq4_1(i+1)-tq4_1(i));
    qdd4part1(i+1)=thetadd2(4);
end

%% Second part (q1, q2, q3 and q4 change)
% Times vectors
tENDsecondSemiPar2=t2/2;
tq1par2=(0:timestep:tENDsecondSemiPar2(1));
tq2par2=(0:timestep:tENDsecondSemiPar2(2));
tq3par2=(0:timestep:tENDsecondSemiPar2(3));
tq4par2=(0:timestep:tENDsecondSemiPar2(4));
tENDlin2=t23;
tq1lin2=[tq1par2(1:end-1) tq1par2(1,end)+(0:timestep:tENDlin2(1))];
tq2lin2=[tq2par2(1:end-1) tq2par2(1,end)+(0:timestep:tENDlin2(1))];
tq3lin2=[tq3par2(1:end-1) tq3par2(1,end)+(0:timestep:tENDlin2(2))];
tq4lin2=[tq4par2(1:end-1) tq4par2(1,end)+(0:timestep:tENDlin2(1))];
tENDfirstSemiPar3=t3/2;
tq1=[tq1lin2(1:end-1) tq1lin2(1,end)+(0:timestep:tENDfirstSemiPar3(1))];
tq2=[tq2lin2(1:end-1) tq2lin2(1,end)+(0:timestep:tENDfirstSemiPar3(1))];
tq3=[tq3lin2(1:end-1) tq3lin2(1,end)+(0:timestep:tENDfirstSemiPar3(2))];
tq4=[tq4lin2(1:end-1) tq4lin2(1,end)+(0:timestep:tENDfirstSemiPar3(1))];

% Preallocation
q1Part2=ones(1,length(tq1)); 
qd1Part2=ones(1,length(tq1)); 
qdd1Part2=ones(1,length(tq1));
q2Part2=ones(1,length(tq2)); 
qd2Part2=ones(1,length(tq2)); 
qdd2Part2=ones(1,length(tq2));
q3Part2=ones(1,length(tq3));
qd3Part2=ones(1,length(tq3));
qdd3Part2=ones(1,length(tq3));
q4Part2=ones(1,length(tq4)); 
qd4Part2=ones(1,length(tq4)); 
qdd4Part2=ones(1,length(tq4));

% Initial condition
q1Part2(1)=q1part1(end); 
qd1Part2(1)=qd1part1(end); 
qdd1Part2(1)=thetadd2(1);  
q2Part2(1)=q2part1(end); 
qd2Part2(1)=qd2part1(end); 
qdd2Part2(1)=thetadd2(2);  
q3Part2(1)=q3part1(end); 
qd3Part2(1)=qd3part1(end); 
qdd3Part2(1)=thetadd2(3);  
q4Part2(1)=q4part1(end); 
qd4Part2(1)=qd4part1(end); 
qdd4Part2(1)=thetadd2(4); 

% Second parabola (second half parabola)
for i=1:length(tq1par2)-1
    q1Part2(i+1)=q1Part2(1,i)+qd1Part2(1,i)*(tq1(i+1)-tq1(i))+(thetadd2(1)/2)*(tq1(i+1)-tq1(i))^2;
    qd1Part2(i+1)=qd1Part2(1,i)+thetadd2(1)*(tq1(i+1)-tq1(i));
    qdd1Part2(i+1)=thetadd2(1);
end
for i=1:length(tq2par2)-1
    q2Part2(i+1)=q2Part2(1,i)+qd2Part2(1,i)*(tq2(i+1)-tq2(i))+(thetadd2(2)/2)*(tq2(i+1)-tq2(i))^2;
    qd2Part2(i+1)=qd2Part2(1,i)+thetadd2(2)*(tq2(i+1)-tq2(i));
    qdd2Part2(i+1)=thetadd2(2);
end
for i=1:length(tq3par2)-1
    q3Part2(i+1)=q3Part2(1,i)+qd3Part2(1,i)*(tq3(i+1)-tq3(i))+(thetadd2(3)/2)*(tq3(i+1)-tq3(i))^2;
    qd3Part2(i+1)=qd3Part2(1,i)+thetadd2(3)*(tq3(i+1)-tq3(i));
    qdd3Part2(i+1)=thetadd2(3);
end
for i=1:length(tq4par2)-1
    q4Part2(i+1)=q4Part2(1,i)+qd4Part2(1,i)*(tq4(i+1)-tq4(i))+(thetadd2(4)/2)*(tq4(i+1)-tq4(i))^2;
    qd4Part2(i+1)=qd4Part2(1,i)+thetadd2(4)*(tq4(i+1)-tq4(i));
    qdd4Part2(i+1)=thetadd2(4);
end

% Second linear reagion
for i=length(tq1par2):length(tq1lin2)-1
    q1Part2(i+1)=q1Part2(i);
    qd1Part2(i+1)=0;
    qdd1Part2(i+1)=0;
end
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
for i=length(tq4par2):length(tq4lin2)-1
    q4Part2(i+1)=q4Part2(i);
    qd4Part2(i+1)=0;
    qdd4Part2(i+1)=0;
end

% Third parabola (first half)
for i=length(tq1lin2):length(tq1)-1
    q1Part2(i+1)=q1Part2(i);
    qd1Part2(i+1)=0;
    qdd1Part2(i+1)=0;
end
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
for i=length(tq4lin2):length(tq4)-1
    q4Part2(i+1)=q4Part2(i);
    qd4Part2(i+1)=0;
    qdd4Part2(i+1)=0;
end

% Collect times
Tq1=[tq1_1(1:end-1) tq1_1(end)+tq1];
Tq2=[tq2_1(1:end-1) tq2_1(end)+tq2];
Tq3=[tq3_1(1:end-1) tq3_1(end)+tq3];
Tq4=[tq4_1(1:end-1) tq4_1(end)+tq4];

% Remove first component (because it is the last component of part 1)
q1Part2(1)=[];
qd1Part2(1)=[];
qdd1Part2(1)=[];
q2Part2(1)=[];
qd2Part2(1)=[];
qdd2Part2(1)=[];
q3Part2(1)=[];
qd3Part2(1)=[];
qdd3Part2(1)=[];
q4Part2(1)=[];
qd4Part2(1)=[];
qdd4Part2(1)=[];

%% Third part (only q2 and q3 change)
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
Tq1f=[Tq1(1,1:end-1) Tq1(end)+tq1];
Tq2f=[Tq2(1,1:end-1) Tq2(end)+tq2];
Tq3f=[Tq3(1,1:end-1) Tq3(end)+tq3];
Tq4f=[Tq4(1,1:end-1) Tq4(end)+tq4];

%% Unique time vector
t=[Tq2f(1:end-1) (Tq2f(end):timestep:T)];

%% Set the rimainder empty component of q, qd, qdd (Part 4)
q1Part34=q1Part2(end)*ones(1,length(t)-length(Tq1));
qd1Part34=zeros(1,length(t)-length(Tq1));
qdd1Part34=zeros(1,length(t)-length(Tq1));

q2Part4=q2Part3(end)*ones(1,length(t)-length(Tq2f));
qd2Part4=zeros*ones(1,length(t)-length(Tq2f));
qdd2Part4=zeros(1,length(t)-length(Tq2f));

q3Part4=q3Part3(end)*ones(1,length(t)-length(Tq3f));
qd3Part4=zeros(1,length(t)-length(Tq3f));
qdd3Part4=zeros(1,length(t)-length(Tq3f));

q4Part34=q4Part2(end)*ones(1,length(t)-length(Tq4));
qd4Part34=zeros(1,length(t)-length(Tq4));
qdd4Part34=zeros(1,length(t)-length(Tq4));

%% Joint angles, velocities and accelerations
q1=[q1part1 q1Part2 q1Part34];
q2=[q2part1 q2Part2 q2Part3 q2Part4];
q3=[q3part1 q3Part2 q3Part3 q3Part4];
q4=[q4part1 q4Part2 q4Part34];
qd1=[qd1part1 qd1Part2 qd1Part34];
qd2=[qd2part1 qd2Part2 qd2Part3 qd2Part4];
qd3=[qd3part1 qd3Part2 qd3Part3 qd3Part4];
qd4=[qd4part1 qd4Part2 qd4Part34];
qdd1=[qdd1part1 qdd1Part2 qdd1Part34];
qdd2=[qdd2part1 qdd2Part2 qdd2Part3 qdd2Part4];
qdd3=[qdd3part1 qdd3Part2 qdd3Part3 qdd3Part4];
qdd4=[qdd4part1 qdd4Part2 qdd4Part34];

q=[q1; q2; q3; q4];
qd=[qd1; qd2; qd3; qd4];
qdd=[qdd1; qdd2; qdd3; qdd4];

end