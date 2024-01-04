  clear; close; clc;

syms t;
t0 = 0; 
tf = 10;
theta1_t0 = deg2rad(180);
theta1_tf = 0;
theta2_t0 = deg2rad(90); 
theta2_tf = 0;
theta1_dot_t0 = 0; 
theta1_dot_tf = 0;
theta2_dot_t0 = 0; 
theta2_dot_tf = 0;

A = [1 t0 t0^2 t0^3;
    0 1 2*t0 3*t0^2;
    1 tf tf^2 tf^3;
    0 1 2*tf 3*tf^2];

B_th1 = [theta1_t0; theta1_dot_t0; theta1_tf; theta1_dot_tf];
B_th2 = [theta2_t0; theta2_dot_t0; theta2_tf; theta2_dot_tf];

a_th1 = inv(A)*B_th1;
a_th2 = inv(A)*B_th2;
%display(a_th1)
% display(a_th2)
 theta1 = a_th1(1,1) + a_th1(2,1)*t + a_th1(3,1)*t^2 + a_th1(4,1)*t^3;
 theta2 = a_th2(1,1) + a_th2(2,1)*t + a_th2(3,1)*t^2 + a_th2(4,1)*t^3;
 theta1_dot = a_th1(2,1) + (2*a_th1(3,1)*t) + (3*a_th1(4,1)*t^2);
 theta2_dot = a_th2(2,1) + (2*a_th2(3,1)*t) + (3*a_th2(4,1)*t^2);
 theta1_ddot = (2*a_th1(3,1)) + (6*a_th1(4,1)*t);
 theta2_ddot = (2*a_th2(3,1)) + (6*a_th2(4,1)*t);
%  display(theta1)
%  display(theta2)
%  display(theta1_dot)
%  display(theta2_dot)
%  display(theta1_ddot)
%  display(theta2_ddot)

