clear; close; clc;

syms m1 m2 l1 l2 r1 r2 I1 I2 g 'real'
syms theta1 theta2 theta1_dot theta2_dot theta1_ddot theta2_ddot Tau1 Tau2 'real'
L = ((0.5)*(m1 * r1^2 * theta1_dot^2)) + ((0.5)*(I1 * theta1_dot^2)) + ((0.5)*m2*((l1^2 * theta1_dot^2) + ((r2^2)*(theta1_dot + theta2_dot)^2) + (2 * l1 * theta1_dot * r2 * (theta1_dot + theta2_dot) * cos(theta2)))) + ((0.5)*I2*((theta1_dot + theta2_dot)^2)) - (m1 * g * r1 * cos(theta1) + (m2 * g * (l1 * cos(theta1) + (r2 * cos(theta1 + theta2)))));                             
DL_dq = jacobian(L,[theta1,theta2,theta1_dot,theta2_dot]);

dDL_dtDdq_theta1 = jacobian(DL_dq(1,3), [theta1; theta1_dot])*[theta1_dot; theta1_ddot] + jacobian(DL_dq(1,3), [theta2; theta2_dot])*[theta2_dot; theta2_ddot];
dDL_dtDdq_theta2 = jacobian(DL_dq(1,4), [theta1; theta1_dot])*[theta1_dot; theta1_ddot] + jacobian(DL_dq(1,4), [theta2; theta2_dot])*[theta2_dot; theta2_ddot];

eq1 = dDL_dtDdq_theta1 - DL_dq(1,1) - Tau1;
eq2 = dDL_dtDdq_theta2 - DL_dq(1,2) - Tau2;

display(eq1+Tau1)
display(eq2+Tau2)

X = sym ('X', [4,1]);
X(1) = theta1;
X(2) = theta1_dot;
X(3) = theta2;
X(4) = theta2_dot;

sol = solve([eq1 == 0, eq2 == 0],[theta1_ddot,theta2_ddot]);

%display(sol.theta1_ddot);
%display(sol.theta2_ddot);

EoM = [eq1; eq2];
EoM_t = subs(EoM, [theta1_dot, theta1_ddot, theta2_dot, theta2_ddot, Tau1, Tau2], [0, 0, 0, 0, 0, 0]);

sol = solve(EoM_t == 0, [theta1, theta2]);

%display(sol.theta1);
%display(sol.theta2);

