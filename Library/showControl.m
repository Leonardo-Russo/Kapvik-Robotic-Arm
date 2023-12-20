function showControl(tc, theta, thetad, thetadd, qDes,qdDes,...
                  qddDes, E, tau, tauMotor, iMotor, typeOfControl, tauMotorMax, iMotorMax)

if typeOfControl=="Sto2Nav"
    figure('name', 'Control Stowage to Navigation (Angles, velocities, acceleration)')
    sgtitle('Control Stowage to Navigation (Angles, velocities, acceleration)', 'Interpreter','latex','FontSize',20,'FontWeight','bold');
elseif typeOfControl=="Sto2Ret"
    figure('name', 'Control Stowage to Retrieval (Angles, velocities, acceleration)')
    sgtitle('Control Stowage to Retrieval (Angles, velocities, acceleration)', 'Interpreter','latex','FontSize',20,'FontWeight','bold');
elseif typeOfControl=="Ret2Trans"
    figure('name', 'Control Retrieval to Transfer (Angles, velocities, acceleration)')
    sgtitle('Control Retrieval to Transfer (Angles, velocities, acceleration)', 'Interpreter','latex','FontSize',20,'FontWeight','bold');
else
    error('Specify type of controller')
end
subplot(3,4,1)
plot(tc,rad2deg(theta(1,:)),'r',lineWidth=1.5)
hold on
plot(tc,rad2deg(qDes(1,:)),'c',lineWidth=1.5)
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\theta_1(t)$$ $$[^\circ]$$','Interpreter','latex')
legend('Real trajectory', 'Desidered trajectory','Interpreter','latex','Location','best')
title('Joint angles 1','Interpreter','latex')
set(gca,'FontSize',10)
subplot(3,4,2)
plot(tc,rad2deg(theta(2,:)),'b',lineWidth=1.5)
hold on
plot(tc,rad2deg(qDes(2,:)),'c',lineWidth=1.5)
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\theta_2(t)$$ $$[^\circ]$$','Interpreter','latex')
legend('Real trajectory', 'Desidered trajectory','Interpreter','latex','Location','best')
title('Joint angles 2','Interpreter','latex')
set(gca,'FontSize',10)
subplot(3,4,3)
plot(tc,rad2deg(theta(3,:)),'g',lineWidth=1.5)
hold on
plot(tc,rad2deg(qDes(3,:)),'c',lineWidth=1.5)
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\theta_3(t)$$ $$[^\circ]$$','Interpreter','latex')
legend('Real trajectory', 'Desidered trajectory','Interpreter','latex','Location','best')
title('Joint angles 3','Interpreter','latex')
set(gca,'FontSize',10)
subplot(3,4,4)
plot(tc,rad2deg(theta(4,:)),'m',lineWidth=1.5)
hold on
plot(tc,rad2deg(qDes(4,:)),'c',lineWidth=1.5)
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\theta_4(t)$$ $$[^\circ]$$','Interpreter','latex')
legend('Real trajectory', 'Desidered trajectory','Interpreter','latex','Location','best')
title('Joint angles 4','Interpreter','latex')
set(gca,'FontSize',10)

subplot(3,4,5)
plot(tc,rad2deg(thetad(1,:)),'r',lineWidth=1.5)
hold on
plot(tc,rad2deg(qdDes(1,:)),'c',lineWidth=1.5)
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\dot\theta_1(t)$$ $$[^\circ/s]$$','Interpreter','latex')
legend('Real trajectory', 'Desidered trajectory','Interpreter','latex','Location','best')
title('Joint velocities 1','Interpreter','latex')
set(gca,'FontSize',10)
subplot(3,4,6)
plot(tc,rad2deg(thetad(2,:)),'b',lineWidth=1.5)
hold on
plot(tc,rad2deg(qdDes(2,:)),'c',lineWidth=1.5)
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\dot\theta_2(t)$$ $$[^\circ/s]$$','Interpreter','latex')
legend('Real trajectory', 'Desidered trajectory','Interpreter','latex','Location','best')
title('Joint velocities 2','Interpreter','latex')
set(gca,'FontSize',10)
subplot(3,4,7)
plot(tc,rad2deg(thetad(3,:)),'g',lineWidth=1.5)
hold on
plot(tc,rad2deg(qdDes(3,:)),'c',lineWidth=1.5)
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\dot\theta_3(t)$$ $$[^\circ/s]$$','Interpreter','latex')
legend('Real trajectory', 'Desidered trajectory','Interpreter','latex','Location','best')
title('Joint velocities 3','Interpreter','latex')
set(gca,'FontSize',10)
subplot(3,4,8)
plot(tc,rad2deg(thetad(4,:)),'m',lineWidth=1.5)
hold on
plot(tc,rad2deg(qdDes(4,:)),'c',lineWidth=1.5)
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\dot\theta_4(t)$$ $$[^\circ/s]$$','Interpreter','latex')
legend('Real trajectory', 'Desidered trajectory','Interpreter','latex','Location','best')
title('Joint velocities 4','Interpreter','latex')
set(gca,'FontSize',10)

subplot(3,4,9)
plot(tc,rad2deg(thetadd(1,:)),'r',lineWidth=1.5)
hold on
plot(tc,rad2deg(qddDes(1,:)),'c',lineWidth=1.5)
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\ddot\theta_1(t)$$ $$[^\circ/s^2]$$','Interpreter','latex')
legend('Real trajectory', 'Desidered trajectory','Interpreter','latex','Location','best')
title('Joint acceleration 1','Interpreter','latex')
set(gca,'FontSize',10)
subplot(3,4,10)
plot(tc,rad2deg(thetadd(2,:)),'b',lineWidth=1.5)
hold on
plot(tc,rad2deg(qddDes(2,:)),'c',lineWidth=1.5)
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\ddot\theta_2(t)$$ $$[^\circ/s^2]$$','Interpreter','latex')
legend('Real trajectory', 'Desidered trajectory','Interpreter','latex','Location','best')
title('Joint acceleration 2','Interpreter','latex')
set(gca,'FontSize',10)
subplot(3,4,11)
plot(tc,rad2deg(thetadd(3,:)),'g',lineWidth=1.5)
hold on
plot(tc,rad2deg(qddDes(3,:)),'c',lineWidth=1.5)
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\ddot\theta_3(t)$$ $$[^\circ/s^2]$$','Interpreter','latex')
legend('Real trajectory', 'Desidered trajectory','Interpreter','latex','Location','best')
title('Joint acceleration 3','Interpreter','latex')
set(gca,'FontSize',10)
subplot(3,4,12)
plot(tc,rad2deg(thetadd(4,:)),'m',lineWidth=1.5)
hold on
plot(tc,rad2deg(qddDes(4,:)),'c',lineWidth=1.5)
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\ddot\theta_4(t)$$ $$[^\circ/s^2]$$','Interpreter','latex')
legend('Real trajectory', 'Desidered trajectory','Interpreter','latex','Location','best')
title('Joint acceleration 4','Interpreter','latex')
set(gca,'FontSize',10)

if typeOfControl=="Sto2Nav"
    figure('name', 'Control Stowage to Navigation (Error and Control Torque load side)')
    sgtitle('Control Stowage to Navigation (Error and Control Torque load side)', 'Interpreter','latex','FontSize',20,'FontWeight','bold');
elseif typeOfControl=="Sto2Ret"
    figure('name', 'Control Stowage to Retrieval (Error and Control Torque load side)')
    sgtitle('Control Stowage to Retrieval (Error and Control Torque load side)', 'Interpreter','latex','FontSize',20,'FontWeight','bold');
elseif typeOfControl=="Ret2Trans"
    figure('name', 'Control Retrieval to Transfer (Error and Control Torque load side)')
    sgtitle('Control Retrieval to Transfer (Error and Control Torque load side)', 'Interpreter','latex','FontSize',20,'FontWeight','bold');
else
    error('Specify type of controller')
end
subplot(2,4,1)
plot(tc,rad2deg(E(1,:)),'r',lineWidth=1.5)
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\Delta e_1=\theta_{1,des}-\theta_1$$ $$[^\circ]$$','Interpreter','latex')
title('Error angle Joint 1','Interpreter','latex')
set(gca,'FontSize',10)
subplot(2,4,2)
plot(tc,rad2deg(E(2,:)),'r',lineWidth=1.5)
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\Delta e_2=\theta_{2,des}-\theta_2$$ $$[^\circ]$$','Interpreter','latex')
title('Error angle Joint 2','Interpreter','latex')
set(gca,'FontSize',10)
subplot(2,4,3)
plot(tc,rad2deg(E(3,:)),'r',lineWidth=1.5)
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\Delta e_3=\theta_{3,des}-\theta_3$$ $$[^\circ]$$','Interpreter','latex')
title('Error angle Joint 3','Interpreter','latex')
set(gca,'FontSize',10)
subplot(2,4,4)
plot(tc,rad2deg(E(4,:)),'r',lineWidth=1.5)
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\Delta e_4=\theta_{4,des}-\theta_4$$ $$[^\circ]$$','Interpreter','latex')
title('Error angle Joint 4','Interpreter','latex')
set(gca,'FontSize',10)

subplot(2,4,5)
plot(tc,tau(1,:),'r',lineWidth=1.5)
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\tau_1(t)$$ [Nm]','Interpreter','latex')
title('Control torque Joint 1 (load side)','Interpreter','latex')
set(gca,'FontSize',10)
subplot(2,4,6)
plot(tc,tau(2,:),'r',lineWidth=1.5)
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\tau_2(t)$$ [Nm]','Interpreter','latex')
title('Control torque Joint 2 (load side)','Interpreter','latex')
set(gca,'FontSize',10)
subplot(2,4,7)
plot(tc,tau(3,:),'r',lineWidth=1.5)
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\tau_3(t)$$ [Nm]','Interpreter','latex')
title('Control torque Joint 3 (load side)','Interpreter','latex')
set(gca,'FontSize',10)
subplot(2,4,8)
plot(tc,tau(4,:),'r',lineWidth=1.5)
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\tau_4(t)$$ [Nm]','Interpreter','latex')
title('Control torque Joint 4 (load side)','Interpreter','latex')
set(gca,'FontSize',10)

if typeOfControl=="Sto2Nav"
    figure('name', 'Control Stowage to Navigation (Control Torque and Control Current motor side)')
    sgtitle('Control Stowage to Navigation (Control Torque and Control Current motor side)', 'Interpreter','latex','FontSize',20,'FontWeight','bold');
elseif typeOfControl=="Sto2Ret"
    figure('name', 'Control Stowage to Retrieval (Control Torque and Control Current motor side)')
    sgtitle('Control Stowage to Retrieval (Control Torque and Control Current motor side)', 'Interpreter','latex','FontSize',20,'FontWeight','bold');
elseif typeOfControl=="Ret2Trans"
    figure('name', 'Control Retrieval to Transfer (Control Torque and Control Current motor side)')
    sgtitle('Control Retrieval to Transfer (Control Torque and Control Current motor side)', 'Interpreter','latex','FontSize',20,'FontWeight','bold');
else
    error('Specify type of controller')
end
subplot(2,4,1)
plot(tc,tauMotor(1,:)*10^(3),'b',lineWidth=1.5)
hold on
% plot(tcSto2Nav,(tauMotorMax*10^(3))*ones(1,length(tcSto2Nav)),'r',lineWidth=1.5)
% plot(tcSto2Nav,-(tauMotorMax*10^(3))*ones(1,length(tcSto2Nav)),'r',lineWidth=1.5)
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\tau_{1,m}(t)$$ [mNm]','Interpreter','latex')
title('Control torque Joint 1 (motor side)','Interpreter','latex')
set(gca,'FontSize',10)
subplot(2,4,2)
plot(tc,tauMotor(2,:)*10^(3),'b',lineWidth=1.5)
hold on
% plot(tcSto2Nav,(tauMotorMax*10^(3))*ones(1,length(tcSto2Nav)),'r',lineWidth=1.5)
% plot(tcSto2Nav,-(tauMotorMax*10^(3))*ones(1,length(tcSto2Nav)),'r',lineWidth=1.5)
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\tau_{2,m}(t)$$ [mNm]','Interpreter','latex')
title('Control torque Joint 2 (motor side)','Interpreter','latex')
set(gca,'FontSize',10)
subplot(2,4,3)
plot(tc,tauMotor(3,:)*10^(3),'b',lineWidth=1.5)
hold on
% plot(tcSto2Nav,(tauMotorMax*10^(3))*ones(1,length(tcSto2Nav)),'r',lineWidth=1.5)
% plot(tcSto2Nav,-(tauMotorMax*10^(3))*ones(1,length(tcSto2Nav)),'r',lineWidth=1.5)
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\tau_{3,m}(t)$$ [mNm]','Interpreter','latex')
title('Control torque Joint 3 (motor side)','Interpreter','latex')
set(gca,'FontSize',10)
subplot(2,4,4)
plot(tc,tauMotor(4,:)*10^(3),'b',lineWidth=1.5)
hold on
% plot(tcSto2Nav,(tauMotorMax*10^(3))*ones(1,length(tcSto2Nav)),'r',lineWidth=1.5)
% plot(tcSto2Nav,-(tauMotorMax*10^(3))*ones(1,length(tcSto2Nav)),'r',lineWidth=1.5)
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\tau_{4,m}(t)$$ [mNm]','Interpreter','latex')
title('Control torque Joint 4 (motor side)','Interpreter','latex')
set(gca,'FontSize',10)

subplot(2,4,5)
plot(tc,iMotor(1,:),'b',lineWidth=1.5)
hold on
% plot(tcSto2Nav,iMotorMax*ones(1,length(tcSto2Nav)),'r',lineWidth=1.5)
% plot(tcSto2Nav,-iMotorMax*ones(1,length(tcSto2Nav)),'r',lineWidth=1.5)
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$i{1,m}(t)$$ [A]','Interpreter','latex')
title('Control current Joint 1','Interpreter','latex')
set(gca,'FontSize',10)
subplot(2,4,6)
plot(tc,iMotor(2,:),'b',lineWidth=1.5)
hold on
% plot(tcSto2Nav,iMotorMax*ones(1,length(tcSto2Nav)),'r',lineWidth=1.5)
% plot(tcSto2Nav,-iMotorMax*ones(1,length(tcSto2Nav)),'r',lineWidth=1.5)
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$i{2,m}(t)$$ [A]','Interpreter','latex')
title('Control current Joint 2','Interpreter','latex')
set(gca,'FontSize',10)
subplot(2,4,7)
plot(tc,iMotor(3,:),'b',lineWidth=1.5)
hold on
% plot(tcSto2Nav,iMotorMax*ones(1,length(tcSto2Nav)),'r',lineWidth=1.5)
% plot(tcSto2Nav,-iMotorMax*ones(1,length(tcSto2Nav)),'r',lineWidth=1.5)
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$i{3,m}(t)$$ [A]','Interpreter','latex')
title('Control current Joint 3','Interpreter','latex')
set(gca,'FontSize',10)
subplot(2,4,8)
plot(tc,iMotor(4,:),'b',lineWidth=1.5)
hold on
% plot(tcSto2Nav,iMotorMax*ones(1,length(tcSto2Nav)),'r',lineWidth=1.5)
% plot(tcSto2Nav,-iMotorMax*ones(1,length(tcSto2Nav)),'r',lineWidth=1.5)
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$i{4,m}(t)$$ [A]','Interpreter','latex')
title('Control current Joint 4','Interpreter','latex')
set(gca,'FontSize',10)

end