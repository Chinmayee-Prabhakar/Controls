
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

Pos = [];
Vel = [];
Effort = [];
time = [];
i = 1;

tic;
t = 0;

%k = [17.4887   -9.4994   10.2765   -1.9021;-2.6777   38.6213    0.0720   12.7235];
%k = [3.8534   -3.0500    3.9116   -1.5240; -0.5896    4.2295    0.1147 4.0884];
%k = [5.0000         0    2.0000         0; 0    4.6200         0    4.3000];
k = [5.0000         0    2.0000         0; 0    5.5200         0    4.7000];

m1=1; m2=1; l1=1; l2=1; r1=0.45; r2=0.45; I1=0.084; I2=0.084; g=9.81;

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
  theta1_des = jointData.Position(1);
  theta2_dot_des = jointData.Position(2);
  theta1_dot_des = jointData.Velocity(1);
  theta2_dot = jointData.Velocity(2);
  state = [jointData.Position;jointData.Velocity];
  v = -k*(state - des_state) + vd;

  M = [(m1*r1^2 + m2*r2^2 + 2*m2*cos(theta2_dot_des)*r2*l1 + m2*l1^2 + I1 + I2), (m2*r2^2 + l1*m2*cos(theta2_dot_des)*r2 + I2);(m2*r2^2 + l1*m2*cos(theta2_dot_des)*r2 + I2), (m2*r2^2 + I2)];

  C = [-(2*r2*theta2_dot*l1*m2*sin(theta2_dot_des)), -(r2*theta2_dot*l1*m2*sin(theta2_dot_des));(r2*l1*m2*sin(theta2_dot_des)*theta1_dot_des), 0];

  G = [(- sin(theta1_des)*(r1*g*m1 + g*l1*m2) - r2*g*m2*sin(theta1_des + theta2_dot_des));(- r2*g*m2*sin(theta1_des + theta2_dot_des))];

  T = M*v+C*[theta1_dot_des; theta2_dot]+G;
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

theta1_des = (pi*time.^3)/500 - (3*pi*time.^2)/100 + pi;
theta2_des = (pi*time.^3)/1000 - (3*pi*time.^2)/200 + pi/2;
theta1_dot_des = (3*pi*time.^2)/500 - (3*pi*time)/50;
theta2_dot_des = (3*pi*time.^2)/1000 - (3*pi*time)/100;


% disconnect from roscore
rosshutdown;
%Plotting results
figure;
plot(time,Pos(1,:),'b','linewidth',2);
hold on;
plot(time,theta1_des);
xlabel('Time(sec)','FontSize',14)
ylabel('Theta1(radian)','FontSize',14)

figure;
plot(time,Pos(2,:),'b','linewidth',2);
hold on;
plot(time,theta2_des);
xlabel('Time(sec)','FontSize',14)
ylabel('Theta2(radian)','FontSize',14)

figure;
plot(time,Vel(1,:),'r','linewidth',2);
hold on;
plot(time,theta1_dot_des);
xlabel('Time(sec)','FontSize',14)
ylabel('Theta1 Dot(radian/s)','FontSize',14)

figure;
plot(time,Vel(2,:),'r','linewidth',2);
hold on;
plot(time,theta2_dot_des);
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

