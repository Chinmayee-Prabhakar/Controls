
rosshutdown; clear; close; clc;

% ROS Setup
rosinit;
JointStates = rossubscriber('/rrbot/joint_states');
j1_effort = rospublisher('/rrbot/joint1_effort_controller/command');
j2_effort = rospublisher('/rrbot/joint2_effort_controller/command');
tau1 = rosmessage(j1_effort);
tau2 = rosmessage(j2_effort);
tau1.Data = 0;
tau2.Data = 0;
send(j1_effort,tau1);
send(j2_effort,tau2);
client = rossvcclient('/gazebo/set_model_configuration');
req = rosmessage(client);
req.ModelName = 'rrbot';
req.UrdfParamName = 'robot_description';
req.JointNames = {'joint1','joint2'};
req.JointPositions = [deg2rad(200), deg2rad(125)];
resp = call(client,req,'Timeout',3);


phi = 0.9;
rho = [10, 0;0, 10];

B = [0 0;
    0 0;
    1 0;
    0 1];
P =  [6.2500         0    1.2500         0;
         0    6.2500         0    1.2500;
    1.2500         0    1.2500         0;
         0    1.2500         0    1.2500];

Pos = [];
Vel = [];
Effort = [];
time = [];
i = 1;

tic;
t = 0;

k = [ 0.5000         0    2.5000         0; 0    0.5000         0    2.5000];

m1=1; m2=1; l1=1; l2=1; r1=0.45; r2=0.45; I1=0.084; I2=0.084; g=9.81;
m1_hat=0.75 ; m2_hat=0.75; I1_hat=0.063; I2_hat=0.063;

while(t < 10)
  t = toc;

  des_state = [(pi*t^3)/500 - (3*pi*t^2)/100 + pi;
            (pi*t^3)/1000 - (3*pi*t^2)/200 + pi/2;
            (3*pi*t^2)/500 - (3*pi*t)/50;
            (3*pi*t^2)/1000 - (3*pi*t)/100];
  vd = [(3*pi*t)/250 - (3*pi)/50;
        (3*pi*t)/500 - (3*pi)/100];


% read the joint states
  jointData = receive(JointStates);
% inspect the "jointData" variable in MATLAB to get familiar with its structure
  theta1 = jointData.Position(1);
  theta2 = jointData.Position(2);
  theta1_dot = jointData.Velocity(1);
  theta2_dot = jointData.Velocity(2);
  state = [jointData.Position;jointData.Velocity];
  e = state - des_state;

  if phi > 0
     if norm(B'*P*e)>phi
         vr = -rho*(B'*P*e)/norm(B'*P*e);
     else
         vr = -rho*(B'*P*e)/phi;
     end
  else
    if norm(B'*P*e) ~= 0
        vr = -rho*(B'*P*e)/norm(B'*P*e);
    else
        vr = 0;
    end
  end


  v = -k*(state - des_state) + vd + vr;

  a = I1_hat + I2_hat + m1_hat*r1^2 + m2_hat*(l1^2 + r2^2);
  b = m2_hat*l1*r2;
  d = I2_hat + m2_hat*r2^2;

  Mmat= [a+2*b*cos(theta2), d+b*cos(theta2); d+b*cos(theta2), d];
  Cmat= [-b*sin(theta2)*theta2_dot, -b*sin(theta2)*(theta1_dot+theta2_dot); b*sin(theta2)*theta1_dot,0];
  Gmat= [-m1*g*r1*sin(theta1)-m2*g*(l1*sin(theta1)+r2*sin(theta1+theta2)); -m2*g*r2*sin(theta1+theta2)];
  T = Mmat*v+Cmat*[theta1_dot; theta2_dot]+Gmat;

  tau1.Data = T(1);
  tau2.Data = T(2);

  send(j1_effort,tau1);
  send(j2_effort,tau2);

  Pos(:, i) = jointData.Position;
  Vel(:,i) = jointData.Velocity;
  Effort(:,i) = jointData.Effort;
  time(:,i) = t;
  i = i+1;
end

tau1.Data = 0;
tau2.Data = 0;

send(j1_effort,tau1);
send(j2_effort,tau2);

theta1 = (pi*time.^3)/500 - (3*pi*time.^2)/100 + pi;
theta2 = (pi*time.^3)/1000 - (3*pi*time.^2)/200 + pi/2;
theta1_dot = (3*pi*time.^2)/500 - (3*pi*time)/50;
theta2 = (3*pi*time.^2)/1000 - (3*pi*time)/100;


% disconnect from roscore
rosshutdown;
%Plotting results
figure;
plot(time,Pos(1,:),'b','linewidth',2);
hold on;
plot(time,theta1);
xlabel('Time(sec)','FontSize',14)
ylabel('Theta1(radian)','FontSize',14)

figure;
plot(time,Pos(2,:),'b','linewidth',2);
hold on;
plot(time,theta2);
xlabel('Time(sec)','FontSize',14)
ylabel('Theta2(radian)','FontSize',14)

figure;
plot(time,Vel(1,:),'r','linewidth',2);
hold on;
plot(time,theta1_dot);
xlabel('Time(sec)','FontSize',14)
ylabel('Theta1 Dot(radian/s)','FontSize',14)

figure;
plot(time,Vel(2,:),'r','linewidth',2);
hold on;
plot(time,theta2);
xlabel('Time(sec)','FontSize',14)
ylabel('Theta2 Dot(radian/sec)','FontSize',14)

figure;
plot(time,Effort(1,:),'g','linewidth',2);
xlabel('Time','FontSize',14)
ylabel('Effort 1','FontSize',14)

figure;
plot(time,Effort(2,:),'g','linewidth',2);
xlabel('Time','FontSize',14)
ylabel('Effort 2','FontSize',14)

