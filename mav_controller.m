function u = mav_controller(obj,tgt,path_c,ts)
    u = zeros(1,4);
    m = 0.53263; % kg
    g = 9.80665;
	t = ts;
    % desired acceleration
    x_pp = 12 * path_c(1) * t^2 + 6 * path_c(2) * t + 2 * path_c(3);
    y_pp = 12 * path_c(6) * t^2 + 6 * path_c(7) * t + 2 * path_c(8);
    z_pp = 12 * path_c(11) * t^2 + 6 * path_c(12) * t + 2 * path_c(13);
    % xb,yb,zb
    t = [x_pp;y_pp;z_pp+g];
    zb = t/norm(t);
    toward = atan2(tgt.position(2)-obj.position(2),tgt.position(1)-obj.position(1));
    d = toward - obj.angle(1);
    d = mod(d+pi, 2*pi) - pi;
    if abs(d) >= 1/2 * pi / (1/ts)
        d = abs(d)/d * 1/2 * pi / (1/ts);
    end
    yaw = obj.angle(1) + d;
    xc = [cos(yaw);sin(yaw);0];
    yb = cross(zb,xc)/norm(cross(zb,xc));
    xb = cross(yb,zb);
    rotm = [xb,yb,zb];
    u(2:4) = rotm2eul(rotm);
    u(1) = m * norm(t); % discrete time signal fault the performance
end