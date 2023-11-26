clear
close all
clc

%% Robot
a2=0.46;
a3=0.44;
theta1=pi/2; %[rad]
theta2=0; %[rad]
theta3=0; %[rad]
theta4=0; %[rad]
robot = SerialLink( [RevoluteMDH('alpha',0,'a',0,'d',0)...
    RevoluteMDH('alpha',pi/2,'a',0,'d',0) RevoluteMDH('alpha',0,'a',a2,'d',0)...
    RevoluteMDH('alpha',0,'a',a3,'d',0)], 'name', '2-link');
robot.teach([theta1 theta2 theta3 theta4]);

%% Cinematica diretta
T=robot.fkine([theta1 theta2 theta3 theta4]);

%% Define links and joints properties

Upper_Arm=link(0.46, 40, 2, 3.5);
Fore_Arm=link(0.44, 40, 2, 3.5);

Joint_1=joint(1.15, 8.4, -160, 100, 1.5*10^(-4), -5*10^(-4));
Joint_2=joint(1.28, 8.4,  -90,  90, 1.5*10^(-4), -5*10^(-4));
Joint_3=joint(1.39, 5.3, -150, 110, 1.5*10^(-4), -5*10^(-4));
Joint_4=joint(0.67, 6.7,  -90,   5, 1.5*10^(-4), -5*10^(-4));

%% Proprietà robot
% Mass proprerty of the link
m=[0 0 Upper_Arm.Mass Fore_Arm.Mass]'; % link mass (at CDM)
I=zeros(3,3,4);
I(:,:,3)=Upper_Arm.Inertia;
I(:,:,4)=Fore_Arm.Inertia;

% Link length
L=[0 0 Upper_Arm.Length Fore_Arm.Length];

syms g real
robot.gravity=([0 0 g]);
for i=1:4
    robot.links(i).m=m(i);
    robot.links(i).r=[L(i) 0 0];
    robot.links(i).I=I(:,:,i);
end

%% Matrici equazioni dinamiche
syms q1 q2 q3 q4 q1d q2d q3d q4d real
q=[q1 q2 q3 q4];
qd=[q1d q2d q3d q4d];
M=vpa(simplify(robot.inertia(q)));
V=vpa(simplify(robot.coriolis(q,qd))*[q1d q2d q3d q4d]');
G=vpa(simplify(robot.gravload(q)))';