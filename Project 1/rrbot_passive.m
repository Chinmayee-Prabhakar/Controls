rosshutdown; clear; close; clc;
% ROS Setup
rosinit;
JointStates = rossubscriber('/rrbot/joint_states');
client = rossvcclient('/gazebo/set_model_configuration');
req = rosmessage(client);
req.ModelName = 'rrbot';
req.UrdfParamName = 'robot_description';
req.JointNames = {'joint1','joint2'};
req.JointPositions = [deg2rad(30), deg2rad(45)];
resp = call(client,req,'Timeout',3);
Pos = [];
Vel = [];
Effort = [];
time = [];
tic;
t = 0;
while(t < 10)
t = toc;
time = [time, t];
%display(t)
% read the joint states
jointData = receive(JointStates);
Pos = [Pos, jointData.Position];
Vel = [Vel, jointData.Velocity];
Effort = [Effort, jointData.Effort];
%showdetails(jointData);
% inspect the "jointData" variable in MATLAB to get familiar with its structure
% sample the time and joint state values here to be plotted at the end
end
%display(Effort);
% disconnect from roscore
rosshutdown;
% plot the trajectories
%Pos(2,:) = 0;
subplot(2,2,1);
plot(time,Pos,'b','linewidth',2);
xlabel('Time','FontSize',14)
ylabel('Position','FontSize',14)
subplot(2,2,2);
plot(time,Vel,'r','linewidth',2);
xlabel('Time','FontSize',14)
ylabel('Velocity','FontSize',14)
subplot(2,2,3);
plot(time,Effort,'g','linewidth',2);
xlabel('Time','FontSize',14)
ylabel('Effort','FontSize',14)
