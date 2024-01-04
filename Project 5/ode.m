function [dX, T] = ode(t, X, K, P, Gamma)

m1=1; m2=1; l1=1; l2=1; r1=0.45; r2=0.45; I1=0.084; I2=0.084; g=9.81;


B = [0 0;
    0 0;
    1 0;
    0 1];

dX = zeros (9,1);
X = num2cell(X);

[theta1, theta2, theta1_dot, theta2_dot, a_1, a_2, a_3, a_4, a_5] = deal(X{:});


state = [theta1; theta2; theta1_dot; theta2_dot];

% Desired States obtained from trajectory

des_state = [(pi*t^3)/500 - (3*pi*t^2)/100 + pi;
            (pi*t^3)/1000 - (3*pi*t^2)/200 + pi/2;
            (3*pi*t^2)/500 - (3*pi*t)/50;
            (3*pi*t^2)/1000 - (3*pi*t)/100];
        
vd = [(3*pi*t)/250 - (3*pi)/50;
    (3*pi*t)/500 - (3*pi)/100];

e = state - des_state;


k = K;


v = -k*(state - des_state) + vd;

Y = [v(1), cos(theta2)*(2*v(1) + v(2)) - 2*sin(theta2)*theta1_dot*theta2_dot - sin(theta2)*theta2_dot^2,v(2),-sin(theta1)*g,-sin(theta1 + theta2)*g;
    0, sin(theta2)*theta1_dot^2 + cos(theta2)*v(1), v(1) + v(2),0,-sin(theta1+theta2)*g];

alpha_hat = [a_1; a_2; a_3; a_4; a_5];

T = Y * alpha_hat;

Tau1=T(1,1);
Tau2=T(2,1);
dX(1)=theta1_dot;
dX(2)=theta2_dot;
dX(3)=(I2*Tau1 - I2*Tau2 + Tau1*r2^2*m2 - Tau2*r2^2*m2 + r2^3*theta1_dot^2*l1*m2^2*sin(theta2) + r2^3*theta2_dot^2*l1*m2^2*sin(theta2) + r2^2*g*l1*m2^2*sin(theta1) + I2*r1*g*m1*sin(theta1) - Tau2*r2*l1*m2*cos(theta2) + I2*g*l1*m2*sin(theta1) + 2*r2^3*theta1_dot*theta2_dot*l1*m2^2*sin(theta2) + r2^2*theta1_dot^2*l1^2*m2^2*cos(theta2)*sin(theta2) - r2^2*g*l1*m2^2*sin(theta1 + theta2)*cos(theta2) + I2*r2*theta1_dot^2*l1*m2*sin(theta2) + I2*r2*theta2_dot^2*l1*m2*sin(theta2) + r1*r2^2*g*m1*m2*sin(theta1) + 2*I2*r2*theta1_dot*theta2_dot*l1*m2*sin(theta2))/(I1*I2 + r2^2*l1^2*m2^2 + I2*r1^2*m1 + I1*r2^2*m2 + I2*l1^2*m2 + r1^2*r2^2*m1*m2 - r2^2*l1^2*m2^2*cos(theta2)^2);
dX(4)=-(I2*Tau1 - I1*Tau2 - I2*Tau2 - Tau2*r1^2*m1 + Tau1*r2^2*m2 - Tau2*r2^2*m2 - Tau2*l1^2*m2 + r2*theta1_dot^2*l1^3*m2^2*sin(theta2) + r2^3*theta1_dot^2*l1*m2^2*sin(theta2) + r2^3*theta2_dot^2*l1*m2^2*sin(theta2) - r2*g*l1^2*m2^2*sin(theta1 + theta2) - I1*r2*g*m2*sin(theta1 + theta2) + r2^2*g*l1*m2^2*sin(theta1) + I2*r1*g*m1*sin(theta1) + Tau1*r2*l1*m2*cos(theta2) - 2*Tau2*r2*l1*m2*cos(theta2) + I2*g*l1*m2*sin(theta1) + 2*r2^3*theta1_dot*theta2_dot*l1*m2^2*sin(theta2) + 2*r2^2*theta1_dot^2*l1^2*m2^2*cos(theta2)*sin(theta2) + r2^2*theta2_dot^2*l1^2*m2^2*cos(theta2)*sin(theta2) - r2^2*g*l1*m2^2*sin(theta1 + theta2)*cos(theta2) + r2*g*l1^2*m2^2*cos(theta2)*sin(theta1) - r1^2*r2*g*m1*m2*sin(theta1 + theta2) + I1*r2*theta1_dot^2*l1*m2*sin(theta2) + I2*r2*theta1_dot^2*l1*m2*sin(theta2) + I2*r2*theta2_dot^2*l1*m2*sin(theta2) + r1*r2^2*g*m1*m2*sin(theta1) + 2*r2^2*theta1_dot*theta2_dot*l1^2*m2^2*cos(theta2)*sin(theta2) + r1^2*r2*theta1_dot^2*l1*m1*m2*sin(theta2) + 2*I2*r2*theta1_dot*theta2_dot*l1*m2*sin(theta2) + r1*r2*g*l1*m1*m2*cos(theta2)*sin(theta1))/(I1*I2 + r2^2*l1^2*m2^2 + I2*r1^2*m1 + I1*r2^2*m2 + I2*l1^2*m2 + r1^2*r2^2*m1*m2 - r2^2*l1^2*m2^2*cos(theta2)^2);


M_hat = [a_1 + (2 * a_2 * cos(theta2)),  a_3 + (a_2 * cos(theta2));
        a_3 + (a_2 * cos(theta2)),  a_3];


phi = M_hat\Y;

alphat_hat_dot = -Gamma \ (phi' * B' * P * e);

dX(5) = alphat_hat_dot(1);
dX(6) = alphat_hat_dot(2);
dX(7) = alphat_hat_dot(3);
dX(8) = alphat_hat_dot(4);
dX(9) = alphat_hat_dot(5);

end
