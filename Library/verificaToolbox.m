clear
close all
clc

%% Robot
a2=0.46;
a3=0.44;
d=-0.04*1.5;
theta1=0; %[rad]
theta2=0; %[rad]
theta3=0; %[rad]
theta4=0; %[rad]
robot = SerialLink( [RevoluteMDH('alpha',0,'a',0,'d',0)...
    RevoluteMDH('alpha',pi/2,'a',0,'d',0) RevoluteMDH('alpha',0,'a',a2,'d',d,'offset',pi)...
    RevoluteMDH('alpha',0,'a',a3,'d',0)], 'name', 'TeamWork robot');
robot.teach([theta1 theta2 theta3 theta4]);

%% Cinematica diretta
T=robot.fkine([theta1 theta2 theta3 theta4]);