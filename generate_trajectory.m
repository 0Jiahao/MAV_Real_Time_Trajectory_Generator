% generate_trajectory
clear; clc; close all;
waypoints = cell(2,1);
pts.position = [0,0,30];   % xyz axis
pts.heading = 0;            % yaw angle
pts.time = 0;               % time 
waypoints{1} = pts;
pts.position = [1,0,25];
pts.heading = 0;
pts.time = 0.2;               
waypoints{2} = pts;
pts.position = [1,2,30];
pts.heading = 0;
pts.time = 0.8;               
waypoints{3} = pts;
pts.position = [0,2,30];
pts.heading = 0;
pts.time = 1;               
waypoints{4} = pts;
tic;
%% construct the QP problem and searching for optimal x = [c1;c2;c3;c4]
% The QP problem: 1/2 * x' * H * x + f' * x
r_xyz = 1; r_yaw = 1;   % regularization 
H = zeros(18,18);    % 5 * 3 + 3 (4th order for xyz & 2nd order for yaw)
H(1,1) = r_xyz;
H(6,6) = r_xyz;
H(11,11) = r_xyz;
H(16,16) = r_yaw;
f = zeros(1,18);
Aeq = [];
beq = [];
% start & final constraints (equality)
for i = 1:numel(waypoints)
   Aeq = [Aeq;[waypoints{i}.time^4,waypoints{i}.time^3,waypoints{i}.time^2,waypoints{i}.time,1,zeros(1,13)]];
   Aeq = [Aeq;[zeros(1,5),waypoints{i}.time^4,waypoints{i}.time^3,waypoints{i}.time^2,waypoints{i}.time,1,zeros(1,8)]];
   Aeq = [Aeq;[zeros(1,10),waypoints{i}.time^4,waypoints{i}.time^3,waypoints{i}.time^2,waypoints{i}.time,1,zeros(1,3)]];
   Aeq = [Aeq;[zeros(1,15),waypoints{i}.time^2,waypoints{i}.time,1]];
   beq = [beq;waypoints{i}.position(1);waypoints{i}.position(2);waypoints{i}.position(3);waypoints{i}.heading];
end
% corridor constraints (inequality)
nc = 5;
epsilon = 0.05;
corridor.start = waypoints{2};
corridor.final = waypoints{3};
A = [];
b = [];
for i = 1:nc
    ri_position = corridor.start.position + i/(1+nc) * (corridor.final.position - corridor.start.position);
    ti = corridor.start.time + i/(1+nc) * (corridor.final.time - corridor.start.time)
    A = [A;-[ti^4,ti^3,ti^2,ti,1,zeros(1,13)]];          
    A = [A;[ti^4,ti^3,ti^2,ti,1,zeros(1,13)]];
    A = [A;-[zeros(1,5),ti^4,ti^3,ti^2,ti,1,zeros(1,8)]];
    A = [A;[zeros(1,5),ti^4,ti^3,ti^2,ti,1,zeros(1,8)]];
    A = [A;-[zeros(1,10),ti^4,ti^3,ti^2,ti,1,zeros(1,3)]];
    A = [A;[zeros(1,10),ti^4,ti^3,ti^2,ti,1,zeros(1,3)]];
    b = [b;epsilon - ri_position(1);ri_position(1) + epsilon;epsilon - ri_position(2);ri_position(2) + epsilon;epsilon - ri_position(3);ri_position(3) + epsilon];
end
%% solving quadratic problem
c = quadprog(H,f,A,b,Aeq,beq);
toc;
%% visualization
subplot(1,2,1);
for i = 1:numel(waypoints)
    scatter3(waypoints{i}.position(1),waypoints{i}.position(2),waypoints{i}.position(3),'filled'); hold on;
end
t = 0:0.001:1;
x = c(1) * t.^4 + c(2) * t.^3 + c(3) * t.^2 + c(4) * t + c(5);
y = c(6) * t.^4 + c(7) * t.^3 + c(8) * t.^2 + c(9) * t + c(10); 
z = c(11) * t.^4 + c(12) * t.^3 + c(13) * t.^2 + c(14) * t + c(15);
yaw = c(16) * t.^2 + c(17) * t + c(18);
plot3(x,y,z); axis equal;
xlabel('x');
ylabel('y');
zlabel('z');
subplot(1,2,2);
for i = 1:numel(waypoints)
    scatter(waypoints{i}.position(1),waypoints{i}.position(2),'filled'); hold on;
end
plot(x,y); axis equal;
xlabel('x');
ylabel('y');
grid on;
set(gcf,'position',[250 0 600 250]);