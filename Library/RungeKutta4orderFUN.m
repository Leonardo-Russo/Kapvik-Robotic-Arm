function [Fun] = RungeKutta4orderFUN(thetad, thetadd)

Fun=[thetad; thetadd];

% %%%%%%%%%%%%%%%%%%%%%%%%%% Runge Kutta 4 order
% [Fun] = RungeKutta4orderFUN(thetad(:,i-1), thetadd(:,i-1));
% k1=Fun;
% k2=RungeKutta4orderFUN(thetad(:,i-1)+k1(5:8)*(dt/2), thetadd(:,i-1));
% k3=RungeKutta4orderFUN(thetad(:,i-1)+k2(5:8)*(dt/2), thetadd(:,i-1));
% k4=RungeKutta4orderFUN(thetad(:,i-1)+k3(5:8)*dt, thetadd(:,i-1));
% Y=[theta(:,i-1); thetad(:,i-1)] + (dt/6)*(k1+2*k2+2*k3+k4);
% theta(:,i)=Y(1:4);
% thetad(:,i)=Y(5:8);
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end