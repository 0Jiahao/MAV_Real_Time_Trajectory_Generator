clear; clc; close all;
    mav.position = [-5,-5,7];         % xyz-axis
    mav.angle = [pi/2,0,0];
    mav.speed = [-3,-5,0];            % body frame
    tgt.position = [0,0,5];           % xyz-axis
    tgt.angle = [pi/6,0,0];
% sampling time
ts = 0.05; 
g = 9.80665;
m = 0.53263; % kg
% control input u = (T,yaw,pitch,roll)
% yaw anti-clockwise; pitch forward; roll right
% definition of yaw, pitch, row see https://en.wikipedia.org/wiki/Aircraft_principal_axes
u = [m*g,0,0,0];                      % initialization of control input
t = 0;
[waypoints,path_c] = path_planner(mav,tgt);
while norm(tgt.position - mav.position) > 0.05
%     u = follow_traj(mav,path_c,ts,t);
    u = mav_controller(mav,tgt,path_c,ts);
    mav = dynamic_mav(mav,u,ts);
    [waypoints,path_c] = path_planner(mav,tgt);
    subplot(1,2,1);
    plot_3d_obj(mav); hold on;
    plot_3d_obj(tgt);
    plot_path(waypoints,path_c); hold off;
    title(['t=',num2str(t),'s']);
    subplot(1,2,2);
    plot_2d_obj(mav);
    title(['z=',num2str(mav.position(3)),'m']);
    set(gcf,'position',[250 0 1200 500]);
    pause(0);
    t = t + ts;
end

