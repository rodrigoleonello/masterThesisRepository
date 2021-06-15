close all;
%% IDENTIFICACAO
% cd C:\Users\maril\Desktop\identificacao
load('identwork.mat')
figure(1)
subplot(2,2,1)
plot([5:0.03:30],roll_*3.14/180,[5:0.03:30],g1c*3.14/180,'LineWidth',1.5);
ylabel({'$\phi$ [rad]'},'Interpreter','latex')
xlabel('Tempo [s]')
legend('real','modelo')
%title('a')
%xlim
grid on
%ylim([-1 1])
subplot(2,2,3)
plot([5:0.03:30],u_roll_,'LineWidth',1.5);
ylabel({'$u_{\phi}$'},'Interpreter','latex')
xlabel('Tempo [s]')
grid on
ylim([-0.3 0.3])
subplot(2,2,2)
plot([5:0.03:30],x_,[5:0.03:30],g5a,'LineWidth',1.5); 
ylabel('y[m]','Interpreter','latex')
xlabel('Tempo [s]')
legend('real','modelo')
grid on
subplot(2,2,4)
plot([5:0.03:30],roll_*3.14/180,'LineWidth',1.5);
ylabel({'$\phi$ [rad]'},'Interpreter','latex')
xlabel('Tempo [s]')
grid on

figure(2)
subplot(2,2,1)
plot([5:0.02775:30],pitch_*3.14/180,[5:0.02775:30],g2a*3.14/180,'LineWidth',1.5);
ylabel({'$\theta$ [rad]'},'Interpreter','latex')
xlabel('Tempo [s]')
legend('real','modelo')
grid on
subplot(2,2,3)
plot([5:0.02775:30],u_pitch_,'LineWidth',1.5);
ylabel({'$u_{\theta}$'},'Interpreter','latex')
xlabel('Tempo [s]')
ylim([-0.3 0.3])
grid on
subplot(2,2,2)
plot([5:0.02775:30],y_,[5:0.02775:30],g6a,'LineWidth',1.5);
legend('real','modelo')
ylabel('x[m]','Interpreter','latex')
xlabel('Tempo [s]')
grid on
subplot(2,2,4)
plot([5:0.02775:30],pitch_*3.14/180,'LineWidth',1.5);
ylabel({'$\theta$ [rad]'},'Interpreter','latex')
xlabel('Tempo [s]')
grid on

figure(3)
subplot(2,2,1)
plot([5:0.03:20],yaw_*3.14/180,[5:0.03:20],g4*3.14/180,'LineWidth',1.5);
ylabel({'$\psi$ [rad]'},'Interpreter','latex')
xlabel('Tempo [s]')
legend('real','modelo')
grid on
subplot(2,2,3)
plot([5:0.03:20],u_yaw_,'LineWidth',1.5);
ylabel({'$u_{\dot{\psi}}$'},'Interpreter','latex')
xlabel('Tempo [s]')
ylim([-0.35 0.35])
grid on
subplot(2,2,2)
plot([10:0.03:20],z_,[10:0.03:20],g3b,'LineWidth',1.5);
ylabel('z[m]','Interpreter','latex')
xlabel('Tempo [s]')
legend('real','modelo')
grid on
subplot(2,2,4)
plot([10:0.03:20],u_z_,'LineWidth',1.5);
ylabel({'$u_{\dot{z}}$'},'Interpreter','latex')
xlabel('Tempo [s]')
ylim([-0.35 0.35])
grid on

%% VALIDACAO
load('validwork.mat')
figure(4)
subplot(2,2,1)
plot([3:0.03:22],roll_*3.14/180,[3:0.03:22],g1c*3.14/180,'LineWidth',1.5);
ylabel({'$\phi$ [rad]'},'Interpreter','latex')
xlabel('Tempo [s]')
legend('real','modelo')
xlim([4 21])
grid on
subplot(2,2,3)
plot([3:0.03:22],u_roll_,'LineWidth',1.5);
ylabel({'$u_{\phi}$'},'Interpreter','latex')
xlabel('Tempo [s]')
xlim([4 21])
grid on
subplot(2,2,2)
plot([3:0.03:22],x_,[3:0.03:22],g5a,'LineWidth',1.5); 
ylabel('y[m]','Interpreter','latex')
xlabel('Tempo [s]')
legend('real','modelo')
xlim([4 21])
grid on
subplot(2,2,4)
plot([3:0.03:22],roll_*3.14/180,'LineWidth',1.5);
ylabel({'$\phi$ [rad]'},'Interpreter','latex')
xlabel('Tempo [s]')
xlim([4 21])
grid on

figure(5)
subplot(2,3,1)
plot([2:0.03:19],pitch_*3.14/180,[2:0.03:19],g2a*3.14/180,'LineWidth',1.5);
ylabel({'$\theta$ [rad]'},'Interpreter','latex')
xlabel('Tempo [s]')
legend('real','modelo')
xlim([4 19])
grid on
subplot(2,2,3)
plot([2:0.03:19],u_pitch_,'LineWidth',1.5);
ylabel({'$u_{\theta}$'},'Interpreter','latex')
xlabel('Tempo [s]')
xlim([4 19])
grid on
subplot(2,2,2)
plot([2:0.03:19],y_,[2:0.03:19],g6a,'LineWidth',1.5);
legend('real','modelo')
ylabel('x[m]','Interpreter','latex')
xlabel('Tempo [s]')
xlim([4 19])
grid on
subplot(2,2,4)
plot([2:0.03:19],pitch_*3.14/180,'LineWidth',1.5);
ylabel({'$\theta$ [rad]'},'Interpreter','latex')
xlabel('Tempo [s]')
xlim([4 19])
grid on

figure(6)
subplot(2,2,1)
plot([0:0.03:10],yaw_*3.14/180,[0:0.03:10],g4*3.14/180,'LineWidth',1.5);
ylabel({'$\psi$ [rad]'},'Interpreter','latex')
xlabel('Tempo [s]')
legend('real','modelo')
grid on
subplot(2,2,3)
plot([0:0.03:10],u_yaw_,'LineWidth',1.5);
ylabel({'$u_{\dot{\psi}}$'},'Interpreter','latex')
xlabel('Tempo [s]')
grid on
subplot(2,2,2)
plot([3:0.03:21],z_,[3:0.03:21],g3b,'LineWidth',1.5);
ylabel('z[m]','Interpreter','latex')
xlabel('Tempo [s]')
legend('real','modelo')
xlim([4 19])
grid on
subplot(2,2,4)
plot([3:0.03:21],u_z_,'LineWidth',1.5);
ylabel({'$u_{\dot{z}}$'},'Interpreter','latex')
xlabel('Tempo [s]')
xlim([4 19])
grid on