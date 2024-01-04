clear; clc; close all;

 T = 10;
 y0 = [deg2rad(200),deg2rad(125), 0, 0];
 
 
 [t,y] = ode45(@ode,[0,T],y0);
 
 K = [ 2.0000         0    3.0000         0; 0    2.0000         0    3.0000];
 
 U = [];
 for i = 1:length(t)
     [x,ui] = ode(t(i,1),y(i,:));
     U(i,:)= ui';
 end
 
 theta1_des = (pi*t.^3)/500 - (3*pi*t.^2)/100 + pi;
 theta2_des = (pi*t.^3)/1000 - (3*pi*t.^2)/200 + pi/2;
 theta1_dot_des = (3*pi*t.^2)/500 - (3*pi*t)/50;
 theta2_dot_des = (3*pi*t.^2)/1000 - (3*pi*t)/100;
 
 %plot(t,y);
 % Plotting the output
 figure;
 subplot(2,2,1);
 plot(t,y(:,1),'b','linewidth',2);
 hold on;
 plot(t,theta1_des);
 xlabel('time (sec)');
 ylabel('th1 (radian)')
 subplot(2,2,2);
 plot(t,y(:,2),'r','linewidth',2);
 hold on;
 plot(t,theta2_des);
 xlabel('time (sec)');
 ylabel('th2 (radian)');
 subplot(2,2,3);
 plot(t,y(:,3),'b','linewidth',2);
 hold on;
 plot(t,theta1_dot_des);
 xlabel('time (sec)');
 ylabel('theta1 dot (radian/sec)');
 subplot(2,2,4);
 plot(t,y(:,4),'r','linewidth',2);
 hold on;
 plot(t,theta2_dot_des);
 xlabel('time (sec)');
 ylabel('theta2 dot (radian/sec)');
 
 figure;
 plot(t,U(:,1));
 xlabel('time (sec)');
 ylabel('u1 (N.m)');
 
 figure
 plot(t,U(:,2));
 xlabel('time (sec)');
 ylabel('u2 (N.m)');

