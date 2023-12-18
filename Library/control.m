function [t, theta, thetad, thetadd, qDes, qdDes , qddDes, E, tau, tauMotor, iMotor] =...
           control(T, theta0, thetad0, M0, V0, G0, F0, fc, ft,...
           qSto2Nav, qdSto2Nav, qddSto2Nav, Joint_1, Joint_2, Joint_3, Joint_4, Kv, Kp)

%% Motor costant (the same for each joint)
km=Joint_1.Tau_m_max/Joint_1.i_tau_m_max;

%% Times
dt=1/fc;
t=0:dt:T;
tTraj=0:(1/ft):T;

%% Interpolate trajectory at each sempling time of the control
qDes=zeros(4,length(t)); % preallocation
qdDes=zeros(4,length(t)); % preallocation
qddDes=zeros(4,length(t)); % preallocation
for j=1:4
    qDes(j,:)=interp1(tTraj, qSto2Nav(j,:), t);
    qdDes(j,:)=interp1(tTraj, qdSto2Nav(j,:), t);
    qddDes(j,:)=interp1(tTraj, qddSto2Nav(j,:), t);
end


%% Preallocation
theta=zeros(4,length(t));
thetad=zeros(4,length(t));
thetadd=zeros(4,length(t));
tau=zeros(4,length(t));
E=zeros(4,length(t));
Edot=zeros(4,length(t));
tauP=zeros(4,length(t));

%% Integrazione (metodo di Eulero)
for i=1:length(t)

    if i==1

        thetad(:,1)=thetad0;
        theta(:,1)=theta0;

        alfa=M0;
        beta=V0+G0+F0;

        Edot(:,1)=qdDes(:,1)-thetad(:,1);
        E(:,1)=qDes(:,1)-theta(:,1);

        tauP(:,1)=qddDes(:,1)+Kv*Edot(:,1)+Kp*E(:,1);
        tau(:,1)=alfa*tauP(:,1)+beta;

        thetadd(:,1)=M0\(tau(:,1)-V0-G0-F0);
        
    else

        M=MassMatrix(theta(1,i-1), theta(2,i-1), theta(3,i-1), theta(4,i-1));
        V=Coriolis(theta(1,i-1), theta(2,i-1), theta(3,i-1), theta(4,i-1), thetad(1,i-1), thetad(2,i-1), thetad(3,i-1), thetad(4,i-1));
        G=Gravity(theta(1,i-1), theta(2,i-1), theta(3,i-1), theta(4,i-1));
        
        if theta(1,i)>0
            Tcoul1=Joint_1.Friction_Torque_max;
        else
            Tcoul1=Joint_1.Friction_Torque_min;
        end
        if theta(2,i)>0
            Tcoul2=Joint_2.Friction_Torque_max;
        else
            Tcoul2=Joint_2.Friction_Torque_min;
        end
        if theta(3,i)>0
            Tcoul3=Joint_3.Friction_Torque_max;
        else
            Tcoul3=Joint_3.Friction_Torque_min;
        end
        if theta(4,i)>0
            Tcoul4=Joint_4.Friction_Torque_max;
        else
            Tcoul4=Joint_4.Friction_Torque_min;
        end
        
        F=Friction(theta(1,i-1), theta(2,i-1), theta(3,i-1), theta(4,i-1), Tcoul1, Tcoul2, Tcoul3, Tcoul4);
        alfa=M;
        beta=V+G+F;
        Edot(:,i)=qdDes(:,i-1)-theta(:,i-1);
        E(:,i)=qDes(:,i-1)-theta(:,i-1);
        tauP(:,i)=qddDes(:,i)+Kv*Edot(:,i)+Kp*E(:,i);
        tau(:,i)=alfa*tauP(:,i)+beta;
        thetadd(:,i)=M\(tau(:,i)-V-G-F);
        thetad(:,i)=thetad(:,i-1)+thetadd(:,i-1)*dt;
        theta(:,i)=theta(:,i-1)+thetad(:,i-1)*dt+(0.5)*thetadd(:,i-1)*dt^2;
    end

end

tauMotor=[tau(1,:)/Joint_1.Gear_Ratio; tau(2,:)/Joint_2.Gear_Ratio;...
    tau(3,:)/Joint_3.Gear_Ratio; tau(4,:)/Joint_4.Gear_Ratio];
iMotor=km*tauMotor;
end