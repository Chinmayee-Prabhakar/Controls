clear; clc; close all;

syms theta1 theta2 theta1_dot theta2_dot theta1_ddot theta2_ddot Tau1 Tau2 m1 m2 r1 r2 l1 l2 I1 I2 T1 T2 g;


q = [theta1; theta2];
q_dot = [theta1_dot; theta2_dot];
q_ddot = [theta1_ddot; theta2_ddot];
Tau = [Tau1; Tau2];

X = [theta1; theta2; theta1_dot; theta2_dot];
X_dot = [theta1_dot; theta2_dot; theta1_ddot; theta2_ddot];

% Manipulator form

M = [(m1*r1^2 + m2*r2^2 + 2*m2*cos(theta2)*r2*l1 + m2*l1^2 + I1 + I2), (m2*r2^2 + l1*m2*cos(theta2)*r2 + I2);
    (m2*r2^2 + l1*m2*cos(theta2)*r2 + I2), (m2*r2^2 + I2)];

C = [-(2*r2*theta2_dot*l1*m2*sin(theta2)), -(r2*theta2_dot*l1*m2*sin(theta2));
    (r2*l1*m2*sin(theta2)*theta1_dot), 0];

G = [(- sin(theta1)*(r1*g*m1 + g*l1*m2) - r2*g*m2*sin(theta1 + theta2));
    (- r2*g*m2*sin(theta1 + theta2))];

Tau = M*q_ddot+C*q_dot+G;

%We have the A and B matrices as follows

 
 A = [0 0 1 0;
     0 0 0 1;
     0 0 0 0;
     0 0 0 0];
 
 B = [0 0;
     0 0;
     1 0;
     0 1];
 
 lambda = [-2, -2, -3, -3];

 
 k = place(A,B,lambda);
 
 Acl = (A - B*k);
 Q = eye(4).*10;
 P = lyap(Acl',Q);
 



