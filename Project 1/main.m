clear; clc; close all;

T = 10;
y0 = [deg2rad(30),deg2rad(45), 0, 0];
%y0 = [0, 0, 0, 0];

[t,y] = ode45(@ode,[0,T],y0);

plot(t,y);

figure;
subplot(2,2,1);
plot(t,y(:,1),'b','linewidth',2);
xlabel('t','FontSize',14)
ylabel('theta1','FontSize',14)
subplot(2,2,2);
plot(t,y(:,2),'r','linewidth',2);
xlabel('t','FontSize',14)
ylabel('theta2','FontSize',14)
subplot(2,2,3);
plot(t,y(:,3),'b','linewidth',2);
xlabel('t','FontSize',14)
ylabel('theta1_dot','FontSize',14)
subplot(2,2,4);
plot(t,y(:,4),'r','linewidth',2);
xlabel('t','FontSize',14)
ylabel('theta2_dot','FontSize',14)