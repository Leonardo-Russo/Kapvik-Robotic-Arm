function showTraj(t, q, qd, qdd, omegaMax1, omegaMax2, omegaMax3, omegaMax4, thetaddMax, typOfTraj)

if typOfTraj=="Sto2Nav"
    xlimSup=25;
    nameTraj='Stowage to Navigation';
elseif typOfTraj=="Sto2Ret"
    xlimSup=45;
    nameTraj='Stowage to Retrieval';
    elseif typOfTraj=="Ret2Trans"
    xlimSup=40;
    nameTraj='Retrieval to Transfer';
end
figure('name', nameTraj, 'WindowState', 'maximized')
subplot(3,4,1)
plot(t,rad2deg(q(1,:)),'b',lineWidth=1.5)
hold on
plot(t,150*ones(1,length(t)),'--r',lineWidth=1.5)
plot(t,-160*ones(1,length(t)),'--r',lineWidth=1.5)
xlim([0 xlimSup]);
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$q_1(t)$$ $$[^\circ]$$','Interpreter','latex')
title('Joint angle 1','Interpreter','latex')
set(gca,'FontSize',15)
subplot(3,4,2)
plot(t,rad2deg(q(2,:)),'b',lineWidth=1.5)
hold on
plot(t,90*ones(1,length(t)),'--r',lineWidth=1.5)
plot(t,-90*ones(1,length(t)),'--r',lineWidth=1.5)
xlim([0 xlimSup]);
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$q_2(t)$$ $$[^\circ]$$','Interpreter','latex')
title('Joint angle 2','Interpreter','latex')
set(gca,'FontSize',15)
subplot(3,4,3)
plot(t,rad2deg(q(3,:)),'b',lineWidth=1.5)
hold on
plot(t,115*ones(1,length(t)),'--r',lineWidth=1.5)
plot(t,-150*ones(1,length(t)),'--r',lineWidth=1.5)
xlim([0 xlimSup]);
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$q_3(t)$$ $$[^\circ]$$','Interpreter','latex')
title('Joint angle 3','Interpreter','latex')
set(gca,'FontSize',15)
subplot(3,4,4)
plot(t,rad2deg(q(4,:)),'b',lineWidth=1.5)
hold on
plot(t,5*ones(1,length(t)),'--r',lineWidth=1.5)
plot(t,-90*ones(1,length(t)),'--r',lineWidth=1.5)
xlim([0 xlimSup]);
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$q_4(t)$$ $$[^\circ]$$','Interpreter','latex')
title('Joint angle 4','Interpreter','latex')
set(gca,'FontSize',15)

subplot(3,4,5)
plot(t,rad2deg(qd(1,:)),'b',lineWidth=1.5)
hold on
plot(t,rad2deg(omegaMax1)*ones(1,length(t)),'--r',lineWidth=1.5)
plot(t,-rad2deg(omegaMax1)*ones(1,length(t)),'--r',lineWidth=1.5)
xlim([0 xlimSup]);
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\dot q_1(t)$$ $$[^\circ/s]$$','Interpreter','latex')
title('Joint velocity 1','Interpreter','latex')
set(gca,'FontSize',15)
subplot(3,4,6)
plot(t,rad2deg(qd(2,:)),'b',lineWidth=1.5)
hold on
plot(t,rad2deg(omegaMax2)*ones(1,length(t)),'--r',lineWidth=1.5)
plot(t,-rad2deg(omegaMax2)*ones(1,length(t)),'--r',lineWidth=1.5)
xlim([0 xlimSup]);
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\dot q_2(t)$$ $$[^\circ/s]$$','Interpreter','latex')
title('Joint velocity 2','Interpreter','latex')
set(gca,'FontSize',15)
subplot(3,4,7)
plot(t,rad2deg(qd(3,:)),'b',lineWidth=1.5)
hold on
plot(t,rad2deg(omegaMax3)*ones(1,length(t)),'--r',lineWidth=1.5)
plot(t,-rad2deg(omegaMax3)*ones(1,length(t)),'--r',lineWidth=1.5)
xlim([0 xlimSup]);
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\dot q_3(t)$$ $$[^\circ/s]$$','Interpreter','latex')
title('Joint velocity 3','Interpreter','latex')
set(gca,'FontSize',15)
subplot(3,4,8)
plot(t,rad2deg(qd(4,:)),'b',lineWidth=1.5)
hold on
plot(t,rad2deg(omegaMax4)*ones(1,length(t)),'--r',lineWidth=1.5)
plot(t,-rad2deg(omegaMax4)*ones(1,length(t)),'--r',lineWidth=1.5)
xlim([0 xlimSup]);
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\dot q_4(t)$$ $$[^\circ/s]$$','Interpreter','latex')
title('Joint velocity 4','Interpreter','latex')
set(gca,'FontSize',15)

subplot(3,4,9)
plot(t,rad2deg(qdd(1,:)),'b',lineWidth=1.5)
hold on
plot(t,rad2deg(thetaddMax(1))*ones(1,length(t)),'--r',lineWidth=1.5)
plot(t,-rad2deg(thetaddMax(1))*ones(1,length(t)),'--r',lineWidth=1.5)
xlim([0 xlimSup]);
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\ddot q_1(t)$$ $$[^\circ/s^2]$$','Interpreter','latex')
title('Joint acceleration 1','Interpreter','latex')
set(gca,'FontSize',15)
subplot(3,4,10)
plot(t,rad2deg(qdd(2,:)),'b',lineWidth=1.5)
hold on
plot(t,rad2deg(thetaddMax(2))*ones(1,length(t)),'--r',lineWidth=1.5)
plot(t,-rad2deg(thetaddMax(2))*ones(1,length(t)),'--r',lineWidth=1.5)
xlim([0 xlimSup]);
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\ddot q_2(t)$$ $$[^\circ/s^2]$$','Interpreter','latex')
title('Joint acceleration 2','Interpreter','latex')
set(gca,'FontSize',15)
subplot(3,4,11)
plot(t,rad2deg(qdd(3,:)),'b',lineWidth=1.5)
hold on
plot(t,rad2deg(thetaddMax(3))*ones(1,length(t)),'--r',lineWidth=1.5)
plot(t,-rad2deg(thetaddMax(3))*ones(1,length(t)),'--r',lineWidth=1.5)
xlim([0 xlimSup]);
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\ddot q_3(t)$$ $$[^\circ/s^2]$$','Interpreter','latex')
title('Joint acceleration 3','Interpreter','latex')
set(gca,'FontSize',15)
subplot(3,4,12)
plot(t,rad2deg(qdd(4,:)),'b',lineWidth=1.5)
hold on
plot(t,rad2deg(thetaddMax(4))*ones(1,length(t)),'--r',lineWidth=1.5)
plot(t,-rad2deg(thetaddMax(4))*ones(1,length(t)),'--r',lineWidth=1.5)
xlim([0 xlimSup]);
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\ddot q_4(t)$$ $$[^\circ/s^2]$$','Interpreter','latex')
title('Joint acceleration 4','Interpreter','latex')
set(gca,'FontSize',15)
end