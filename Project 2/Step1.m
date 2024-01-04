clear; clc; close all;

syms X1 X2 X3 X4 u lambda
syms m1 m2 l1 l2 r1 r2 I1 I2 g 'real'
syms theta1 theta2 theta1_dot theta2_dot theta1_ddot theta2_ddot Tau1 Tau2 'real'
L = ((0.5)*(m1 * r1^2 * theta1_dot^2)) + ((0.5)*(I1 * theta1_dot^2)) + ((0.5)*m2*((l1^2 * theta1_dot^2) + ((r2^2)*(theta1_dot + theta2_dot)^2) + (2 * l1 * theta1_dot * r2 * (theta1_dot + theta2_dot) * cos(theta2)))) + ((0.5)*I2*((theta1_dot + theta2_dot)^2)) - (m1 * g * r1 * cos(theta1) + (m2 * g * (l1 * cos(theta1) + (r2 * cos(theta1 + theta2)))));                             
DL_dq = jacobian(L,[theta1,theta2,theta1_dot,theta2_dot]);

dDL_dtDdq_theta1 = jacobian(DL_dq(1,3), [theta1; theta1_dot])*[theta1_dot; theta1_ddot] + jacobian(DL_dq(1,3), [theta2; theta2_dot])*[theta2_dot; theta2_ddot];
dDL_dtDdq_theta2 = jacobian(DL_dq(1,4), [theta1; theta1_dot])*[theta1_dot; theta1_ddot] + jacobian(DL_dq(1,4), [theta2; theta2_dot])*[theta2_dot; theta2_ddot];

eq1 = dDL_dtDdq_theta1 - DL_dq(1,1) - Tau1;
eq2 = dDL_dtDdq_theta2 - DL_dq(1,2) - Tau2;

eq1 = subs(eq1,[theta1_dot,theta2_dot,theta1_ddot,theta2_ddot, Tau1,Tau2,m1,m2,l1, l2, I1, I2, r1, r2, g],[0,0,0,0,0,0,1,1,1,1,0.084,0.084,0.45,0.45,9.81]);
eq2 = subs(eq2,[theta1_dot,theta2_dot,theta1_ddot,theta2_ddot, Tau1,Tau2,m1,m2,l1, l2, I1, I2, r1, r2, g],[0,0,0,0,0,0,1,1,1,1,0.084,0.084,0.45,0.45,9.81]);

X = sym ('X', [4,1]);
X(1) = theta1;
X(3) = theta1_dot;
X(2) = theta2;
X(4) = theta2_dot;

sol = solve([eq1 == 0, eq2 == 0],[theta1,theta2]);


%Jacobian Linearization

u = [Tau1 Tau2];

theta1_ddot = (I2*Tau1 - I2*Tau2 + Tau1*m2*r2^2 - Tau2*m2*r2^2 + l1*m2^2*r2^3*theta1_dot^2*sin(theta2) + l1*m2^2*r2^3*theta2_dot^2*sin(theta2) + g*l1*m2^2*r2^2*sin(theta1) + I2*g*l1*m2*sin(theta1) + I2*g*m1*r1*sin(theta1) - Tau2*l1*m2*r2*cos(theta2) + 2*l1*m2^2*r2^3*theta1_dot*theta2_dot*sin(theta2) + l1^2*m2^2*r2^2*theta1_dot^2*cos(theta2)*sin(theta2) - g*l1*m2^2*r2^2*sin(theta1 + theta2)*cos(theta2) + I2*l1*m2*r2*theta1_dot^2*sin(theta2) + I2*l1*m2*r2*theta2_dot^2*sin(theta2) + g*m1*m2*r1*r2^2*sin(theta1) + 2*I2*l1*m2*r2*theta1_dot*theta2_dot*sin(theta2))/(- l1^2*m2^2*r2^2*cos(theta2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);
theta2_ddot = -(I2*Tau1 - I1*Tau2 - I2*Tau2 - Tau2*l1^2*m2 - Tau2*m1*r1^2 + Tau1*m2*r2^2 - Tau2*m2*r2^2 + l1*m2^2*r2^3*theta1_dot^2*sin(theta2) + l1^3*m2^2*r2*theta1_dot^2*sin(theta2) + l1*m2^2*r2^3*theta2_dot^2*sin(theta2) - g*l1^2*m2^2*r2*sin(theta1 + theta2) - I1*g*m2*r2*sin(theta1 + theta2) + g*l1*m2^2*r2^2*sin(theta1) + I2*g*l1*m2*sin(theta1) + I2*g*m1*r1*sin(theta1) + Tau1*l1*m2*r2*cos(theta2) - 2*Tau2*l1*m2*r2*cos(theta2) + 2*l1*m2^2*r2^3*theta1_dot*theta2_dot*sin(theta2) + 2*l1^2*m2^2*r2^2*theta1_dot^2*cos(theta2)*sin(theta2) + l1^2*m2^2*r2^2*theta2_dot^2*cos(theta2)*sin(theta2) - g*l1*m2^2*r2^2*sin(theta1 + theta2)*cos(theta2) + g*l1^2*m2^2*r2*cos(theta2)*sin(theta1) - g*m1*m2*r1^2*r2*sin(theta1 + theta2) + I1*l1*m2*r2*theta1_dot^2*sin(theta2) + I2*l1*m2*r2*theta1_dot^2*sin(theta2) + I2*l1*m2*r2*theta2_dot^2*sin(theta2) + g*m1*m2*r1*r2^2*sin(theta1) + 2*l1^2*m2^2*r2^2*theta1_dot*theta2_dot*cos(theta2)*sin(theta2) + l1*m1*m2*r1^2*r2*theta1_dot^2*sin(theta2) + 2*I2*l1*m2*r2*theta1_dot*theta2_dot*sin(theta2) + g*l1*m1*m2*r1*r2*cos(theta2)*sin(theta1))/(- l1^2*m2^2*r2^2*cos(theta2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);

theta1_ddot = subs(theta1_ddot,[m1,m2,l1, l2, I1, I2, r1, r2, g],[1,1,1,1,0.084,0.084,0.45,0.45,9.81]);
theta2_ddot = subs(theta2_ddot,[m1,m2,l1, l2, I1, I2, r1, r2, g],[1,1,1,1,0.084,0.084,0.45,0.45,9.81]);
X_dot = [theta1_dot; theta2_dot; theta1_ddot; theta2_ddot];

A = jacobian(X_dot, X);
B = jacobian(X_dot, u);

%Substituting for the different equilibrium points

A1 = subs(A,[theta1, theta2, theta1_dot, theta2_dot], [0,0,0,0]);
B1 = subs(B,[theta1, theta2, theta1_dot, theta2_dot], [0,0,0,0]);
A1 = double(A1);
B1 = double(B1);
eigen_1 = eig(A1);
%display(eigen_1);

A2 = subs(A,[theta1, theta2, theta1_dot, theta2_dot], [0,pi,0,0]);
B2 = subs(B,[theta1, theta2, theta1_dot, theta2_dot], [0,pi,0,0]);
A2 = double(A2);
B2 = double(B2);
eigen_2 = eig(A2);
%display(eigen_2);

A3 = subs(A,[theta1, theta2, theta1_dot, theta2_dot], [pi,0,0,0]);
B3 = subs(B,[theta1, theta2, theta1_dot, theta2_dot], [pi,0,0,0]);
A3 = double(A3);
B3 = double(B3);
eigen_3 = eig(A3);
%display(eigen_3);

A4 = subs(A,[theta1, theta2, theta1_dot, theta2_dot], [pi,pi,0,0]);
B4 = subs(B,[theta1, theta2, theta1_dot, theta2_dot], [pi,pi,0,0]);
A4 = double(A4);
B4 = double(B4);
eigen_4 = eig(A4);
%display(eigen_4);

%Controllability 
r = rank(ctrb(A1,B1));
display(r)

%state feedback design for eigenvalues -2, -2, -1+1i, -1-1i

lambda = [-4 -2 -1+1i -1-1i];
k = place(A1,B1,lambda);
