clear; clc; close all;
    mav.position = [-1,-1,25];    % xyz-axis
    mav.angle = [0,0,0];
    mav.speed = [0,0,0];            % body frame
    tgt.position = [1,1,23];      % xyz-axis
    tgt.angle = [0,0,0];
    tgt.speed = [0,0,0];            % body frame
% sampling time
ts = 0.2; 
g = 9.80665;
% control input u = (T,yaw,pitch,roll)
% yaw anti-clockwise; pitch forward; roll right
% definition of yaw, pitch, row see https://en.wikipedia.org/wiki/Aircraft_principal_axes
u = [g,pi/10,pi/60,0];                      % initialization of control input
t = 0;
speed_set = [];
while 1
%     u = simple_path_planer(mav,tgt); % compute control action
    mav = dynamic_mav(mav,u,ts);
    speed_set = [speed_set;mav.speed];
    [waypoints,path_c] = path_planner(mav,tgt);
    subplot(1,2,1);
    plot_3d_obj(mav); hold on;
    plot_3d_obj(tgt); 
    plot_path(waypoints,path_c); hold off;
    title(['t=',num2str(t),'s']);
    subplot(1,2,2);
    plot_2d_obj(mav);
    title(['z=',num2str(mav.position(3)),'m']);
    pause(0.0);
    t = t + ts;
end

