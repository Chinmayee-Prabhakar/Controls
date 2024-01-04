clear; clc; close all;

T = 10;
y0 = [deg2rad(10),deg2rad(10), 0, 0];
[t,y] = ode45(@ode,[0,T],y0);

K = [ 24.7907,   12.7142,    5.7498,    6.0241 ; 6.3156,    7.4114,    1.7584,    2.1889];

sz =  size(y);
U = [];
for d = 1: sz(1)
    
    U(d,:)= -y(d,:)*(K');
    
end
plot(t,y);
% Plotting the output
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

figure;
plot(t,U(:,1));
xlabel('time (sec)');
ylabel('u1 (N.m)');

figure
plot(t,U(:,2));
xlabel('time (sec)');
ylabel('u2 (N.m)');








