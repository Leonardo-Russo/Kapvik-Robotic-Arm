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