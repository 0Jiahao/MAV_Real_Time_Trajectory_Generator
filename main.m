clear; clc; close all;
    obj.position = [0,0,10];    % xyz-axis
    obj.angle = [0,0,0];
    obj.speed = [0,0,0];        % body frame
% sampling time
ts = 0.1; 
% control input u = (T,yaw,pitch,roll)
% https://en.wikipedia.org/wiki/Aircraft_principal_axes
u = [11,pi/2,pi/6,0];
t = 0;
while 1
    obj = dynamic_mav(obj,u,ts);
    subplot(1,2,1);
    plot_3d_obj(obj);
    title(['t=',num2str(t),'s']);
    subplot(1,2,2);
    plot_2d_obj(obj);
    pause(0.01);
    t = t + ts;
end

