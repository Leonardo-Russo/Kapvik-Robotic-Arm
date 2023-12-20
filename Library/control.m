function [t, theta, thetad, thetadd, qDes, qdDes , qddDes, E, tau, tauMotor, iMotor] =...
           control(T, theta0, thetad0, fc, ft,...
           q_des, qd_des, qdd_des, Joint_1, Joint_2, Joint_3, Joint_4, Kv, Kp, sigma)
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
    qDes(j,:)=interp1(tTraj, q_des(j,:), t);
    qdDes(j,:)=interp1(tTraj, qd_des(j,:), t);
    qddDes(j,:)=interp1(tTraj, qdd_des(j,:), t);
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

        M0=MassMatrix(theta0(1), theta0(2), theta0(3), theta0(4));
        V0=Coriolis(theta0(1), theta0(2), theta0(3), theta0(4), thetad0(1), thetad0(2), thetad0(3), thetad0(4));
        G0=Gravity(theta0(1), theta0(2), theta0(3), theta0(4));
        if theta0(1)>0
            Tcoul1=Joint_1.Friction_Torque_max;
        else
            Tcoul1=Joint_1.Friction_Torque_min;
        end
        if theta0(2)>0
            Tcoul2=Joint_2.Friction_Torque_max;
        else
            Tcoul2=Joint_2.Friction_Torque_min;
        end
        if theta0(3)>0
            Tcoul3=Joint_3.Friction_Torque_max;
        else
            Tcoul3=Joint_3.Friction_Torque_min;
        end
        if theta0(4)>0
            Tcoul4=Joint_4.Friction_Torque_max;
        else
            Tcoul4=Joint_4.Friction_Torque_min;
        end
        F0=Friction(thetad0(1), thetad0(2), thetad0(3), thetad0(4), Tcoul1, Tcoul2, Tcoul3, Tcoul4);

        alfa=M0;
        beta=V0+G0+F0;

        Edot(:,1)=qdDes(:,1)-thetad(:,1);
        E(:,1)=qDes(:,1)-theta(:,1);

        tauP(:,1)=qddDes(:,1)+Kv*Edot(:,1)+Kp*E(:,1);
        tau(:,1)=alfa*tauP(:,1)+beta;

        if abs(tau(1,1))>Joint_1.Tau_m_max*Joint_1.Gear_Ratio
            tau(1,1)=sign(tau(1,1))*Joint_1.Tau_m_max*Joint_1.Gear_Ratio;
        end
        if abs(tau(2,1))>Joint_2.Tau_m_max*Joint_2.Gear_Ratio
            tau(2,1)=sign(tau(2,1))*Joint_2.Tau_m_max*Joint_2.Gear_Ratio;
        end
        if abs(tau(3,1))>Joint_3.Tau_m_max*Joint_3.Gear_Ratio
            tau(3,1)=sign(tau(3,1))*Joint_3.Tau_m_max*Joint_3.Gear_Ratio;
        end
        if abs(tau(4,1))>Joint_4.Tau_m_max*Joint_4.Gear_Ratio
            tau(4,1)=sign(tau(4,1))*Joint_4.Tau_m_max*Joint_4.Gear_Ratio;
        end

        thetadd(:,1)=M0\(tau(:,1)-V0-G0+F0);

    else

        thetad(:,i)=thetad(:,i-1)+thetadd(:,i-1)*dt;
        theta(:,i)=theta(:,i-1)+thetad(:,i-1)*dt;

        M=MassMatrix(theta(1,i), theta(2,i), theta(3,i), theta(4,i));
        V=Coriolis(theta(1,i), theta(2,i), theta(3,i), theta(4,i), thetad(1,i), thetad(2,i), thetad(3,i), thetad(4,i));
        G=Gravity(theta(1,i), theta(2,i), theta(3,i), theta(4,i));

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
        
        F=Friction(thetad(1,i), thetad(2,i), thetad(3,i), thetad(4,i), Tcoul1, Tcoul2, Tcoul3, Tcoul4);
        alfa=M;
        beta=V+G+F;
        Edot(:,i)=qdDes(:,i)-thetad(:,i);
        E(:,i)=qDes(:,i)-theta(:,i);
        tauP(:,i)=qddDes(:,i)+Kv*Edot(:,i)+Kp*E(:,i);
        tau(:,i)=alfa*tauP(:,i)+beta;

        if abs(tau(1,i))>Joint_1.Tau_m_max*Joint_1.Gear_Ratio
            tau(1,i)=sign(tau(1,i))*Joint_1.Tau_m_max*Joint_1.Gear_Ratio;
        end
        if abs(tau(2,i))>Joint_2.Tau_m_max*Joint_2.Gear_Ratio
            tau(2,i)=sign(tau(2,i))*Joint_2.Tau_m_max*Joint_2.Gear_Ratio;
        end
        if abs(tau(3,i))>Joint_3.Tau_m_max*Joint_3.Gear_Ratio
            tau(3,i)=sign(tau(3,i))*Joint_3.Tau_m_max*Joint_3.Gear_Ratio;
        end
        if abs(tau(4,i))>Joint_4.Tau_m_max*Joint_4.Gear_Ratio
            tau(4,i)=sign(tau(4,i))*Joint_4.Tau_m_max*Joint_4.Gear_Ratio;
        end

%         if isnan(det(M))
%             theta(:,i-1)
%             i
%             t(i)
%             error('Singularity')
%         end
%         M
%         det(M)
        thetadd(:,i)=M\(tau(:,i)-V-G+F);
        
    end

end

tauMotor=[tau(1,:)/Joint_1.Gear_Ratio; tau(2,:)/Joint_2.Gear_Ratio;...
    tau(3,:)/Joint_3.Gear_Ratio; tau(4,:)/Joint_4.Gear_Ratio];
iMotor=tauMotor/km;
end