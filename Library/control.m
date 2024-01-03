function [t, theta, thetad, thetadd, qDes, qdDes , qddDes, E, tau, tauMotor, iMotor] =...
           control(T, theta0, thetad0, thetadd0, fc, ft,q_des, qd_des, qdd_des,...
           Joint_1, Joint_2, Joint_3, Joint_4, Kv, Kp, sigma, typeOfTraj, P_T)
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
Edotdot=zeros(4,length(t));
tauP=zeros(4,length(t));
thetaHAT=zeros(4,length(t));
thetadHAT=zeros(4,length(t));
thetaddHAT=zeros(4,length(t));

%% Integrazione (metodo di Eulero)
for i=1:length(t)

    if i==1

        thetadd(:,1)=thetadd0;
        thetad(:,1)=thetad0;
        theta(:,1)=theta0;

        thetaddHAT(:,1)=thetadd(:,1);
        thetadHAT(:,1)=thetad(:,1);
        thetaHAT(:,1)=theta(:,1)+randn(4,1)*sigma;

        E(:,1)=qDes(:,1)-thetaHAT(:,1);
        Edot(:,1)=qdDes(:,1)-thetadHAT(:,1);

        if theta(1,1)>0
            Tcoul1=Joint_1.Friction_Torque_max;
        else
            Tcoul1=Joint_1.Friction_Torque_min;
        end
        if theta(2,1)>0
            Tcoul2=Joint_2.Friction_Torque_max;
        else
            Tcoul2=Joint_2.Friction_Torque_min;
        end
        if theta(3,1)>0
            Tcoul3=Joint_3.Friction_Torque_max;
        else
            Tcoul3=Joint_3.Friction_Torque_min;
        end
        if theta(4,1)>0
            Tcoul4=Joint_4.Friction_Torque_max;
        else
            Tcoul4=Joint_4.Friction_Torque_min;
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if thetaHAT(1,1)>0
            Tcoul1HAT=Joint_1.Friction_Torque_max;
        else
            Tcoul1HAT=Joint_1.Friction_Torque_min;
        end
        if thetaHAT(2,1)>0
            Tcoul2HAT=Joint_2.Friction_Torque_max;
        else
            Tcoul2HAT=Joint_2.Friction_Torque_min;
        end
        if thetaHAT(3,1)>0
            Tcoul3HAT=Joint_3.Friction_Torque_max;
        else
            Tcoul3HAT=Joint_3.Friction_Torque_min;
        end
        if thetaHAT(4,1)>0
            Tcoul4HAT=Joint_4.Friction_Torque_max;
        else
            Tcoul4HAT=Joint_4.Friction_Torque_min;
        end

        if typeOfTraj=="Sto2Nav"

            M0=MassMatrix(theta(1,1), theta(2,1), theta(3,1), theta(4,1));
            V0=Coriolis(theta(1,1), theta(2,1), theta(3,1), theta(4,1), thetad(1,1), thetad(2,1), thetad(3,1), thetad(4,1));
            G0=Gravity(theta(1,1), theta(2,1), theta(3,1), theta(4,1));
            F0=Friction(thetad(1,1), thetad(2,1), thetad(3,1), thetad(4,1), Tcoul1, Tcoul2, Tcoul3, Tcoul4);
            React=zeros(4,1);

            M0HAT=MassMatrix(thetaHAT(1,1), thetaHAT(2,1), thetaHAT(3,1), thetaHAT(4,1));
            V0HAT=Coriolis(thetaHAT(1,1), thetaHAT(2,1), thetaHAT(3,1), thetaHAT(4,1), thetadHAT(1,1), thetadHAT(2,1), thetadHAT(3,1), thetadHAT(4,1));
            G0HAT=Gravity(thetaHAT(1,1), thetaHAT(2,1), thetaHAT(3,1), thetaHAT(4,1));
            F0HAT=Friction(thetadHAT(1,1), thetadHAT(2,1), thetadHAT(3,1), thetadHAT(4,1), Tcoul1HAT, Tcoul2HAT, Tcoul3HAT, Tcoul4HAT);
            ReactHAT=zeros(4,1);

        elseif typeOfTraj=="Sto2Ret"

            M0=MassMatrix(theta(1,1), theta(2,1), theta(3,1), theta(4,1));
            V0=Coriolis(theta(1,1), theta(2,1), theta(3,1), theta(4,1), thetad(1,1), thetad(2,1), thetad(3,1), thetad(4,1));
            G0=Gravity(theta(1,1), theta(2,1), theta(3,1), theta(4,1));
            F0=Friction(thetad(1,1), thetad(2,1), thetad(3,1), thetad(4,1), Tcoul1, Tcoul2, Tcoul3, Tcoul4);
            React=zeros(4,1);

            M0HAT=MassMatrix(thetaHAT(1,1), thetaHAT(2,1), thetaHAT(3,1), thetaHAT(4,1));
            V0HAT=Coriolis(thetaHAT(1,1), thetaHAT(2,1), thetaHAT(3,1), thetaHAT(4,1), thetadHAT(1,1), thetadHAT(2,1), thetadHAT(3,1), thetadHAT(4,1));
            G0HAT=Gravity(thetaHAT(1,1), thetaHAT(2,1), thetaHAT(3,1), thetaHAT(4,1));
            F0HAT=Friction(thetadHAT(1,1), thetadHAT(2,1), thetadHAT(3,1), thetadHAT(4,1), Tcoul1HAT, Tcoul2HAT, Tcoul3HAT, Tcoul4HAT);
            ReactHAT=zeros(4,1);

        elseif typeOfTraj=="Ret2Trans"

            ToolCdm2Edge=(R3(theta(4,1))*R3(theta(3,1))*R3(theta(2,1))*...
                          [1 0 0; 0 0 1; 0 -1 0]*R3(theta(1,1)))'*[P_T(1); P_T(1)/2; 0]; % in the station frame
            zT = ToolHeight(theta(1,1), theta(2,1), theta(3,1), theta(4,1));
            z=zT+ToolCdm2Edge(3);

            ToolCdm2EdgeHAT=(R3(thetaHAT(4,1))*R3(thetaHAT(3,1))*R3(thetaHAT(2,1))*...
                          [1 0 0; 0 0 1; 0 -1 0]*R3(thetaHAT(1,1)))'*[P_T(1); P_T(1)/2; 0]; % in the station frame
            zTHAT = ToolHeight(thetaHAT(1,1), thetaHAT(2,1), thetaHAT(3,1), thetaHAT(4,1));
            zHAT=zTHAT+ToolCdm2EdgeHAT(3);

            if z<0 % reaction of the terrain, (we consider the scoop empty untill z>0)

                [f5, n5] = Reaction(P_T, theta(1,1), theta(2,1), theta(3,1), theta(4,1),...
                    thetad(1,1), thetad(2,1), thetad(3,1), thetad(4,1),...
                    thetadd0(1), thetadd0(2), thetadd0(3), thetadd0(4));
                M0=MassMatrix(theta(1,1), theta(2,1), theta(3,1), theta(4,1));
                V0=Coriolis(theta(1,1), theta(2,1), theta(3,1), theta(4,1), thetad(1,1), thetad(2,1), thetad(3,1), thetad(4,1));
                G0=Gravity(theta(1,1), theta(2,1), theta(3,1), theta(4,1));
                F0=Friction(thetad(1,1), thetad(2,1), thetad(3,1), thetad(4,1), Tcoul1, Tcoul2, Tcoul3, Tcoul4);
                [React] = TauREACTION(theta0(1), theta0(2), theta0(3), theta0(4), f5, n5);

            else % augmented mass of tool (because it is full) [probably this case is useless because for the c.i. of Ret2Trans z is less then 0]

                M0=MassMatrixRet2Trans(theta(1,1), theta(2,1), theta(3,1), theta(4,1));
                V0=CoriolisRet2Trans(theta(1,1), theta(2,1), theta(3,1), theta(4,1), thetad(1,1), thetad(2,1), thetad(3,1), thetad(4,1));
                G0=GravityRet2Trans(theta(1,1), theta(2,1), theta(3,1), theta(4,1));
                F0=FrictionRet2Trans(thetad(1,1), thetad(2,1), thetad(3,1), thetad(4,1), Tcoul1, Tcoul2, Tcoul3, Tcoul4);
                React=zeros(4,1);

            end

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if zHAT<0
                [f5HAT, n5HAT] = Reaction(P_T, thetaHAT(1,1), thetaHAT(2,1), thetaHAT(3,1), thetaHAT(4,1),...
                    thetadHAT(4,1), thetadHAT(2,1), thetadHAT(3,1), thetadHAT(4,1),...
                    thetadd0(1), thetadd0(2), thetadd0(3), thetadd0(4));
                M0HAT=MassMatrix(thetaHAT(1,1), thetaHAT(2,1), thetaHAT(3,1), thetaHAT(4,1));
                V0HAT=Coriolis(thetaHAT(1,1), thetaHAT(2,1), thetaHAT(3,1), thetaHAT(4,1), thetadHAT(1,1), thetadHAT(2,1), thetadHAT(3,1), thetadHAT(4,1));
                G0HAT=Gravity(thetaHAT(1,1), thetaHAT(2,1), thetaHAT(3,1), thetaHAT(4,1));
                F0HAT=Friction(thetadHAT(1,1), thetadHAT(2,1), thetadHAT(3,1), thetadHAT(4,1), Tcoul1HAT, Tcoul2HAT, Tcoul3HAT, Tcoul4HAT);
                [ReactHAT] = TauREACTION(thetaHAT(1,1), thetaHAT(1,2), thetaHAT(1,3), thetaHAT(1,4), f5HAT, n5HAT);

            else
                M0HAT=MassMatrixRet2Trans(thetaHAT(1,1), thetaHAT(2,1), thetaHAT(3,1), thetaHAT(4,1));
                V0HAT=CoriolisRet2Trans(thetaHAT(1,1), thetaHAT(2,1), thetaHAT(3,1), thetaHAT(4,1), thetadHAT(1,1), thetadHAT(2,1), thetadHAT(3,1), thetadHAT(4,1));
                G0HAT=GravityRet2Trans(thetaHAT(1,1), thetaHAT(2,1), thetaHAT(3,1), thetaHAT(4,1));
                F0HAT=FrictionRet2Trans(thetadHAT(1,1), thetadHAT(2,1), thetadHAT(3,1), thetadHAT(4,1), Tcoul1HAT, Tcoul2HAT, Tcoul3HAT, Tcoul4HAT);
                ReactHAT=zeros(4,1);
            end

        end

        alfa=M0HAT;
        beta=V0HAT+G0HAT-F0HAT+ReactHAT;

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

        thetadd(:,1)=M0\(tau(:,1)-V0-G0+F0-React);
        Edotdot(:,1)=-Kv*Edot(:,1)-Kp*E(:,1)+(M0HAT\((M0-M0HAT)*thetadd(:,1)+(V0-V0HAT)+(G0-G0HAT)+(-F0+F0HAT)+(React-ReactHAT)));

    else %i>1

        Edot(:,i)=Edot(:,i-1)+Edotdot(:,i-1)*dt;
        E(:,i)=E(:,i-1)+Edot(:,i-1)*dt;

        theta(:,i)=qDes(:,i)-E(:,i);
        thetad(:,i)=qdDes(:,i)-Edot(:,i);

        thetaHAT(:,i)=theta(:,i)+randn(4,1)*sigma;
        thetadHAT(:,i)=thetad(:,i);

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

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if thetaHAT(1,i)>0
            Tcoul1HAT=Joint_1.Friction_Torque_max;
        else
            Tcoul1HAT=Joint_1.Friction_Torque_min;
        end
        if thetaHAT(2,i)>0
            Tcoul2HAT=Joint_2.Friction_Torque_max;
        else
            Tcoul2HAT=Joint_2.Friction_Torque_min;
        end
        if thetaHAT(3,i)>0
            Tcoul3HAT=Joint_3.Friction_Torque_max;
        else
            Tcoul3HAT=Joint_3.Friction_Torque_min;
        end
        if thetaHAT(4,i)>0
            Tcoul4HAT=Joint_4.Friction_Torque_max;
        else
            Tcoul4HAT=Joint_4.Friction_Torque_min;
        end

        if typeOfTraj=="Sto2Nav"

            M=MassMatrix(theta(1,i), theta(2,i), theta(3,i), theta(4,i));
            V=Coriolis(theta(1,i), theta(2,i), theta(3,i), theta(4,i), thetad(1,i), thetad(2,i), thetad(3,i), thetad(4,i));
            G=Gravity(theta(1,i), theta(2,i), theta(3,i), theta(4,i));
            F=Friction(thetad(1,i), thetad(2,i), thetad(3,i), thetad(4,i), Tcoul1, Tcoul2, Tcoul3, Tcoul4);
            React=zeros(4,1);

            MHAT=MassMatrix(thetaHAT(1,i), thetaHAT(2,i), thetaHAT(3,i), thetaHAT(4,i));
            VHAT=Coriolis(thetaHAT(1,i), thetaHAT(2,i), thetaHAT(3,i), thetaHAT(4,i), thetadHAT(1,i), thetadHAT(2,i), thetadHAT(3,i), thetadHAT(4,i));
            GHAT=Gravity(thetaHAT(1,i), thetaHAT(2,i), thetaHAT(3,i), thetaHAT(4,i));
            FHAT=Friction(thetadHAT(1,i), thetadHAT(2,i), thetadHAT(3,i), thetadHAT(4,i), Tcoul1HAT, Tcoul2HAT, Tcoul3HAT, Tcoul4HAT);
            ReactHAT=zeros(4,1);

        elseif typeOfTraj=="Sto2Ret"

            ToolCdm2Edge=(R3(theta(4,i))*R3(theta(3,i))*R3(theta(2,i))*...
                          [1 0 0; 0 0 1; 0 -1 0]*R3(theta(1,i)))'*[P_T(1); P_T(1)/2; 0]; % in the station frame
            zT = ToolHeight(theta(1,i), theta(2,i), theta(3,i), theta(4,i));
            z=zT+ToolCdm2Edge(3);

            if z<0 % reaction of the terrain, (we consider the scoop empty untill z>0)

                [f5, n5] = Reaction(P_T, theta(1,i), theta(2,i), theta(3,i), theta(4,i),...
                                  thetad(1,i), thetad(2,i), thetad(3,i), thetad(4,i),...
                                  thetadd(1,i), thetadd(2,i), thetadd(3,i), thetadd(4,i));
                M=MassMatrix(theta(1,i), theta(2,i), theta(3,i), theta(4,i));
                V=Coriolis(theta(1,i), theta(2,i), theta(3,i), theta(4,i), thetad(1,i), thetad(2,i), thetad(3,i), thetad(4,i));
                G=Gravity(theta(1,i), theta(2,i), theta(3,i), theta(4,i));
                F=Friction(thetad(1,i), thetad(2,i), thetad(3,i), thetad(4,i), Tcoul1, Tcoul2, Tcoul3, Tcoul4);
                [React] = TauREACTION(theta(1,i), theta(2,i), theta(3,i), theta(4,i), f5, n5); 

            else 

                M=MassMatrix(theta(1,i), theta(2,i), theta(3,i), theta(4,i));
                V=Coriolis(theta(1,i), theta(2,i), theta(3,i), theta(4,i), thetad(1,i), thetad(2,i), thetad(3,i), thetad(4,i));
                G=Gravity(theta(1,i), theta(2,i), theta(3,i), theta(4,i));
                F=Friction(thetad(1,i), thetad(2,i), thetad(3,i), thetad(4,i), Tcoul1, Tcoul2, Tcoul3, Tcoul4);
                React=zeros(4,1);
            end

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            ToolCdm2EdgeHAT=(R3(thetaHAT(4,i))*R3(thetaHAT(3,i))*R3(thetaHAT(2,i))*...
                          [1 0 0; 0 0 1; 0 -1 0]*R3(thetaHAT(1,i)))'*[P_T(1); P_T(1)/2; 0]; % in the station frame
            zTHAT = ToolHeight(thetaHAT(1,i), thetaHAT(2,i), thetaHAT(3,i), thetaHAT(4,i));
            zHAT=zTHAT+ToolCdm2EdgeHAT(3);

            if zHAT<0 % reaction of the terrain, (we consider the scoop empty untill z>0)

                [f5HAT, n5HAT] = Reaction(P_T, thetaHAT(1,i), thetaHAT(2,i), thetaHAT(3,i), thetaHAT(4,i),...
                    thetadHAT(1,i), thetadHAT(2,i), thetadHAT(3,i), thetadHAT(4,i),...
                    thetaddHAT(1,i), thetaddHAT(2,i), thetaddHAT(3,i), thetaddHAT(4,i));
                MHAT=MassMatrix(thetaHAT(1,i), thetaHAT(2,i), thetaHAT(3,i), thetaHAT(4,i));
                VHAT=Coriolis(thetaHAT(1,i), thetaHAT(2,i), thetaHAT(3,i), thetaHAT(4,i), thetadHAT(1,i), thetadHAT(2,i), thetadHAT(3,i), thetadHAT(4,i));
                GHAT=Gravity(thetaHAT(1,i), thetaHAT(2,i), thetaHAT(3,i), thetaHAT(4,i));
                FHAT=Friction(thetadHAT(1,i), thetadHAT(2,i), thetadHAT(3,i), thetadHAT(4,i), Tcoul1HAT, Tcoul2HAT, Tcoul3HAT, Tcoul4HAT);
                [ReactHAT] = TauREACTION(thetaHAT(1,i), thetaHAT(2,i), thetaHAT(3,i), thetaHAT(4,i), f5HAT, n5HAT);

            else

                MHAT=MassMatrix(thetaHAT(1,i), thetaHAT(2,i), thetaHAT(3,i), thetaHAT(4,i));
                VHAT=Coriolis(thetaHAT(1,i), thetaHAT(2,i), thetaHAT(3,i), thetaHAT(4,i), thetadHAT(1,i), thetadHAT(2,i), thetadHAT(3,i), thetadHAT(4,i));
                GHAT=Gravity(thetaHAT(1,i), thetaHAT(2,i), thetaHAT(3,i), thetaHAT(4,i));
                FHAT=Friction(thetadHAT(1,i), thetadHAT(2,i), thetadHAT(3,i), thetadHAT(4,i), Tcoul1HAT, Tcoul2HAT, Tcoul3HAT, Tcoul4HAT);
                ReactHAT=zeros(4,1);
            end

        elseif typeOfTraj=="Ret2Trans"

            ToolCdm2Edge=(R3(theta(4,i))*R3(theta(3,i))*R3(theta(2,i))*...
                [1 0 0; 0 0 1; 0 -1 0]*R3(theta(1,i)))'*[P_T(1); P_T(1)/2; 0]; % in the station frame
            zT = ToolHeight(theta(1,i), theta(2,i), theta(3,i), theta(4,i));
            z=zT+ToolCdm2Edge(3);

            if z<0 % reaction of the terrain, (we consider the scoop empty untill z>0)

                [f5, n5] = Reaction(P_T, theta(1,i), theta(2,i), theta(3,i), theta(4,i),...
                    thetad(1,i), thetad(2,i), thetad(3,i), thetad(4,i),...
                    thetadd(1,i), thetadd(2,i), thetadd(3,i), thetadd(4,i));
                M=MassMatrix(theta(1,i), theta(2,i), theta(3,i), theta(4,i));
                V=Coriolis(theta(1,i), theta(2,i), theta(3,i), theta(4,i), thetad(1,i), thetad(2,i), thetad(3,i), thetad(4,i));
                G=Gravity(theta(1,i), theta(2,i), theta(3,i), theta(4,i));
                F=Friction(thetad(1,i), thetad(2,i), thetad(3,i), thetad(4,i), Tcoul1, Tcoul2, Tcoul3, Tcoul4);
                [React] = TauREACTION(theta(1,i), theta(2,i), theta(3,i), theta(4,i), f5, n5);


            else % augmented mass of tool (because it is full), there is not reaction

                M=MassMatrixRet2Trans(theta(1,i), theta(2,i), theta(3,i), theta(4,i));
                V=CoriolisRet2Trans(theta(1,i), theta(2,i), theta(3,i), theta(4,i), thetad(1,i), thetad(2,i), thetad(3,i), thetad(4,i));
                G=GravityRet2Trans(theta(1,i), theta(2,i), theta(3,i), theta(4,i));
                F=FrictionRet2Trans(thetad(1,i), thetad(2,i), thetad(3,i), thetad(4,i), Tcoul1, Tcoul2, Tcoul3, Tcoul4);
                React=zeros(4,1);
            end

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            ToolCdm2EdgeHAT=(R3(thetaHAT(4,i))*R3(thetaHAT(3,i))*R3(thetaHAT(2,i))*...
                [1 0 0; 0 0 1; 0 -1 0]*R3(thetaHAT(1,i)))'*[P_T(1); P_T(1)/2; 0]; % in the station frame
            zTHAT = ToolHeight(thetaHAT(1,i), thetaHAT(2,i), thetaHAT(3,i), thetaHAT(4,i));
            zHAT=zTHAT+ToolCdm2EdgeHAT(3);

            if zHAT<0 % reaction of the terrain, (we consider the scoop empty untill z>0)

                [f5HAT, n5HAT] = Reaction(P_T, thetaHAT(1,i), thetaHAT(2,i), thetaHAT(3,i), thetaHAT(4,i),...
                    thetadHAT(1,i), thetadHAT(2,i), thetadHAT(3,i), thetadHAT(4,i),...
                    thetaddHAT(1,i), thetaddHAT(2,i), thetaddHAT(3,i), thetaddHAT(4,i));
                MHAT=MassMatrix(thetaHAT(1,i), thetaHAT(2,i), thetaHAT(3,i), thetaHAT(4,i));
                VHAT=Coriolis(thetaHAT(1,i), thetaHAT(2,i), thetaHAT(3,i), thetaHAT(4,i), thetadHAT(1,i), thetadHAT(2,i), thetadHAT(3,i), thetadHAT(4,i));
                GHAT=Gravity(thetaHAT(1,i), thetaHAT(2,i), thetaHAT(3,i), thetaHAT(4,i));
                FHAT=Friction(thetadHAT(1,i), thetadHAT(2,i), thetadHAT(3,i), thetadHAT(4,i), Tcoul1HAT, Tcoul2HAT, Tcoul3HAT, Tcoul4HAT);
                [ReactHAT] = TauREACTION(thetaHAT(1,i), thetaHAT(2,i), thetaHAT(3,i), thetaHAT(4,i), f5HAT, n5HAT);


            else % augmented mass of tool (because it is full), there is not reaction

                MHAT=MassMatrixRet2Trans(thetaHAT(1,i), thetaHAT(2,i), thetaHAT(3,i), thetaHAT(4,i));
                VHAT=CoriolisRet2Trans(thetaHAT(1,i), thetaHAT(2,i), thetaHAT(3,i), thetaHAT(4,i), thetadHAT(1,i), thetadHAT(2,i), thetadHAT(3,i), thetadHAT(4,i));
                GHAT=GravityRet2Trans(thetaHAT(1,i), thetaHAT(2,i), thetaHAT(3,i), thetaHAT(4,i));
                FHAT=FrictionRet2Trans(thetadHAT(1,i), thetadHAT(2,i), thetadHAT(3,i), thetadHAT(4,i), Tcoul1HAT, Tcoul2HAT, Tcoul3HAT, Tcoul4HAT);
                ReactHAT=zeros(4,1);

            end
        end

        alfa=MHAT;
        beta=VHAT+GHAT-FHAT+ReactHAT;

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

        thetadd(:,i)=M\(tau(:,i)-V-G+F-React);
        Edotdot(:,i)=-Kv*Edot(:,i)-Kp*E(:,i)+(MHAT\((M-MHAT)*thetadd(:,i)+(V-VHAT)+(G-GHAT)+(-F+FHAT)+(React-ReactHAT)));

    end

end

tauMotor=[tau(1,:)/Joint_1.Gear_Ratio; tau(2,:)/Joint_2.Gear_Ratio;...
    tau(3,:)/Joint_3.Gear_Ratio; tau(4,:)/Joint_4.Gear_Ratio];
iMotor=tauMotor/km;
end