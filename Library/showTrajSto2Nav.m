function showTrajSto2Nav(tSto2Nav, qSto2Nav, qdSto2Nav, qddSto2Nav, omegaMax1, omegaMax2, omegaMax3, omegaMax4)

figure('name', 'Stowage to Navigation', 'WindowState', 'maximized')
subplot(3,4,1)
plot(tSto2Nav,rad2deg(qSto2Nav(1,:)),'r',lineWidth=1.5)
xlim([0 25]);
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\theta_1(t)$$ $$[^\circ]$$','Interpreter','latex')
title('Joint angles 1','Interpreter','latex')
set(gca,'FontSize',15)
subplot(3,4,2)
plot(tSto2Nav,rad2deg(qSto2Nav(2,:)),'b',lineWidth=1.5)
xlim([0 25]);
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\theta_2(t)$$ $$[^\circ]$$','Interpreter','latex')
title('Joint angles 2','Interpreter','latex')
set(gca,'FontSize',15)
subplot(3,4,3)
plot(tSto2Nav,rad2deg(qSto2Nav(3,:)),'g',lineWidth=1.5)
xlim([0 25]);
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\theta_3(t)$$ $$[^\circ]$$','Interpreter','latex')
title('Joint angles 3','Interpreter','latex')
set(gca,'FontSize',15)
subplot(3,4,4)
plot(tSto2Nav,rad2deg(qSto2Nav(4,:)),'m',lineWidth=1.5)
xlim([0 25]);
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\theta_4(t)$$ $$[^\circ]$$','Interpreter','latex')
title('Joint angles 4','Interpreter','latex')
set(gca,'FontSize',15)

subplot(3,4,5)
plot(tSto2Nav,rad2deg(qdSto2Nav(1,:)),'r',lineWidth=1.5)
hold on
plot(tSto2Nav,rad2deg(omegaMax1)*ones(1,length(tSto2Nav)),'k',lineWidth=1.5)
plot(tSto2Nav,-rad2deg(omegaMax1)*ones(1,length(tSto2Nav)),'k',lineWidth=1.5)
xlim([0 25]);
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\dot\theta_1(t)$$ $$[^\circ/s]$$','Interpreter','latex')
title('Joint velocities 1','Interpreter','latex')
set(gca,'FontSize',15)
subplot(3,4,6)
plot(tSto2Nav,rad2deg(qdSto2Nav(2,:)),'b',lineWidth=1.5)
hold on
plot(tSto2Nav,rad2deg(omegaMax2)*ones(1,length(tSto2Nav)),'k',lineWidth=1.5)
plot(tSto2Nav,-rad2deg(omegaMax2)*ones(1,length(tSto2Nav)),'k',lineWidth=1.5)
xlim([0 25]);
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\dot\theta_2(t)$$ $$[^\circ/s]$$','Interpreter','latex')
title('Joint velocities 2','Interpreter','latex')
set(gca,'FontSize',15)
subplot(3,4,7)
plot(tSto2Nav,rad2deg(qdSto2Nav(3,:)),'g',lineWidth=1.5)
hold on
plot(tSto2Nav,rad2deg(omegaMax3)*ones(1,length(tSto2Nav)),'k',lineWidth=1.5)
plot(tSto2Nav,-rad2deg(omegaMax3)*ones(1,length(tSto2Nav)),'k',lineWidth=1.5)
xlim([0 25]);
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\dot\theta_3(t)$$ $$[^\circ/s]$$','Interpreter','latex')
title('Joint velocities 3','Interpreter','latex')
set(gca,'FontSize',15)
subplot(3,4,8)
plot(tSto2Nav,rad2deg(qdSto2Nav(4,:)),'m',lineWidth=1.5)
hold on
plot(tSto2Nav,rad2deg(omegaMax4)*ones(1,length(tSto2Nav)),'k',lineWidth=1.5)
plot(tSto2Nav,-rad2deg(omegaMax4)*ones(1,length(tSto2Nav)),'k',lineWidth=1.5)
xlim([0 25]);
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\dot\theta_4(t)$$ $$[^\circ/s]$$','Interpreter','latex')
title('Joint velocities 4','Interpreter','latex')
set(gca,'FontSize',15)

subplot(3,4,9)
plot(tSto2Nav,rad2deg(qddSto2Nav(1,:)),'r',lineWidth=1.5)
xlim([0 25]);
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\ddot\theta_1(t)$$ $$[^\circ/s^2]$$','Interpreter','latex')
title('Joint acceleration 1','Interpreter','latex')
set(gca,'FontSize',15)
subplot(3,4,10)
plot(tSto2Nav,rad2deg(qddSto2Nav(2,:)),'b',lineWidth=1.5)
xlim([0 25]);
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\ddot\theta_2(t)$$ $$[^\circ/s^2]$$','Interpreter','latex')
title('Joint acceleration 2','Interpreter','latex')
set(gca,'FontSize',15)
subplot(3,4,11)
plot(tSto2Nav,rad2deg(qddSto2Nav(3,:)),'g',lineWidth=1.5)
xlim([0 25]);
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\ddot\theta_3(t)$$ $$[^\circ/s^2]$$','Interpreter','latex')
title('Joint acceleration 3','Interpreter','latex')
set(gca,'FontSize',15)
subplot(3,4,12)
plot(tSto2Nav,rad2deg(qddSto2Nav(4,:)),'m',lineWidth=1.5)
xlim([0 25]);
xlabel('$$t$$ [s]','Interpreter','latex')
ylabel('$$\ddot\theta_4(t)$$ $$[^\circ/s^2]$$','Interpreter','latex')
title('Joint acceleration 4','Interpreter','latex')
set(gca,'FontSize',15)
sgtitle('Stowage to Navigation', 'Interpreter','latex','FontSize',20,'FontWeight','bold');
end