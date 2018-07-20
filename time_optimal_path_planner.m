function [waypoints,path_c,opt_time] = time_optimal_path_planner(mav,tgt,ts) 
    g = 9.80665;
    time = exp(-1.7:0.1:2.5);
    for i = 1:length(time)
        [waypoints,path_c] = path_planner(mav,tgt,time(i));
        t = 0:ts:time(i);
        x = path_c(1) * t.^4 + path_c(2) * t.^3 + path_c(3) * t.^2 + path_c(4) * t + path_c(5);
        y = path_c(6) * t.^4 + path_c(7) * t.^3 + path_c(8) * t.^2 + path_c(9) * t + path_c(10); 
        z = path_c(11) * t.^4 + path_c(12) * t.^3 + path_c(13) * t.^2 + path_c(14) * t + path_c(15);
        x_p = 4 * path_c(1) * t.^3 + 3 * path_c(2) * t.^2 + 2 * path_c(3) * t + path_c(4);
        y_p = 4 * path_c(6) * t.^3 + 3 * path_c(7) * t.^2 + 2 * path_c(8) * t + path_c(9); 
        z_p = 4 * path_c(11) * t.^3 + 3 * path_c(12) * t.^2 + 2 * path_c(13) * t + path_c(14);
        x_pp = 12 * path_c(1) * t.^2 + 6 * path_c(2) * t + 2 * path_c(3);
        y_pp = 12 * path_c(6) * t.^2 + 6 * path_c(7) * t + 2 * path_c(8);
        % acceleration on xy-plane
        xy_pp = sqrt(x_pp.^2 + y_pp.^2);
        % acceleration on z-axis
        z_pp = 12 * path_c(11) * t.^2 + 6 * path_c(12) * t + 2 * path_c(13) + g;
        % desired pitch (toward (x_pp,y_pp))
        % emprically the pitch at the first frame is the maximum need to be
        % proved
        pitch = atan2(xy_pp(1),z_pp(1));
        opt_time = time(i);
        if abs(pitch) <= pi/12
            break;
        end
    end
