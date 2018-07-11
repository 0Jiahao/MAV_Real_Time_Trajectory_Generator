clear; clc; close all;
% control input u = (T,yaw,pitch,roll)
% https://en.wikipedia.org/wiki/Aircraft_principal_axes
obj.position = [0,0,10];
obj.angle = [pi/2,pi/6,0];
obj.speed = [0,0,0];        % body frame

plot_3d_obj(obj);