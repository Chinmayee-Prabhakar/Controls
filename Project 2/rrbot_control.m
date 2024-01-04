rosshutdown; clear; close; clc;
% ROS Setup
rosinit;
j1_effort = rospublisher('/rrbot/joint1_effort_controller/command');
j2_effort = rospublisher('/rrbot/joint2_effort_controller/command');
JointStates = rossubscriber('/rrbot/joint_states');
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
req.JointPositions = [deg2rad(30), deg2rad(45)];
resp = call(client,req,'Timeout',3);

K = [ 24.7907,   12.7142,    5.7498,    6.0241 ; 6.3156,    7.4114,    1.7584,    2.1889];
Pos = [];
Vel = [];
Effort = [];
time = [];
i = 1;
tic;
t = 0;
while(t < 10)
t = toc;
% read the joint states
jointData = receive(JointStates);
% inspect the "jointData" variable in MATLAB to get familiar with its structure
theta1 = jointData.Position(1);
theta2 = jointData.Position(2);
theta1_dot = jointData.Velocity(1);
theta2_dot = jointData.Velocity(2);
% implement your state feedback controller below
U = -K*[theta1; theta2; theta1_dot; theta2_dot];
tau1.Data = U(1);
tau2.Data = U(2);
Pos(:, i) = jointData.Position;
Vel(:,i) = jointData.Velocity;
Effort(:,i) = jointData.Effort;
time(:,i) = t;
i = i+1;

send(j1_effort,tau1);
send(j2_effort,tau2);
% sample the time, joint state values, and calculated torques here to be plotted at the end
end
tau1.Data = 0;
tau2.Data = 0;
send(j1_effort,tau1);
send(j2_effort,tau2);
% disconnect from roscore
rosshutdown;
% plot the trajectories

figure;
plot(time,Pos(1,:),'b','linewidth',2);
xlabel('Time','FontSize',14)
ylabel('Theta1','FontSize',14)

figure;
plot(time,Pos(2,:),'b','linewidth',2);
xlabel('Time','FontSize',14)
ylabel('Theta2','FontSize',14)

figure;
plot(time,Vel(1,:),'r','linewidth',2);
xlabel('Time','FontSize',14)
ylabel('Theta1_dot','FontSize',14)

figure;
plot(time,Vel(2,:),'r','linewidth',2);
xlabel('Time','FontSize',14)
ylabel('Theta2_dot','FontSize',14)

figure;
plot(time,Effort(1,:),'g','linewidth',2);
xlabel('Time','FontSize',14)
ylabel('Effort 1','FontSize',14)

figure;
plot(time,Effort(2,:),'g','linewidth',2);
xlabel('Time','FontSize',14)
ylabel('Effort 2','FontSize',14)