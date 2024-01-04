clear; clc; close all;

m1=1; m2=1; l1=1; l2=1; r1=0.45; r2=0.45; I1=0.084; I2=0.084; g=9.81;
alpha = [m2*l1^2 + m1*r1^2 + m2*r2^2 + I1 + I2; m2*l1*r2; m2*r2^2 + I2; m1*r1 + m2*l1; m2*r2];
alpha_hat = 0.75 * alpha;


 T = 10;

 y0 = [deg2rad(200),deg2rad(125), 0, 0, alpha_hat(1), alpha_hat(2), alpha_hat(3), alpha_hat(4), alpha_hat(5)];


  P =  [11.1667         0              0.8333         0;
          0             11.1667        0           0.8333;
        0.8333          0              1.1667         0;
          0             0.8333         0           1.1667];

%P = 0;

  K = [  6.0000         0    5.0000         0; 0    6.0000         0    5.0000];
  
  gamma = eye(5);

 [t,y] = ode45(@(t,X) ode(t,X,K,P,gamma), [0,T], y0);
 
 
  U = [];
  for i = 1:length(t)
      [x,ui] = ode(t(i,1),y(i,:),K,P,gamma);
      U(i,:)= ui';
  end
  
  theta1_des = (pi*t.^3)/500 - (3*pi*t.^2)/100 + pi;
  theta2_des = (pi*t.^3)/1000 - (3*pi*t.^2)/200 + pi/2;
  theta1_dot_des = (3*pi*t.^2)/500 - (3*pi*t)/50;
  theta2_dot_des = (3*pi*t.^2)/1000 - (3*pi*t)/100;
  
 %plot(t,y);
 %Plotting the output
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

  figure
  plot(t,y(:,5:9));
  xlabel('time (sec)');
  ylabel('alpha_hat');
  legend('alphahat1','alphahat2','alphahat3','alphahat4','alphahat5')
