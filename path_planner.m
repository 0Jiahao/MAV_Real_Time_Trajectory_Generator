function [waypoints,path_c] = path_planner(obj,tgt)
    waypoints = cell(3,1);
    waypoints{1}.position = obj.position;                                   % start point
    waypoints{1}.heading = obj.angle(1);
    waypoints{1}.speed = (eul2rotm(obj.angle) * obj.speed')';               % world frame
    waypoints{1}.time = 0;
    waypoints{2}.position = tgt.position + (eul2rotm(tgt.angle)*[-0.25;0;0])'; % simple corridor start point
    waypoints{2}.time = 0.9;                                                % tunable parameter(larger->)
    waypoints{2}.heading = tgt.angle(1);
    waypoints{3}.position = tgt.position;                                   % final point
    waypoints{3}.heading = tgt.angle(1);
    waypoints{3}.speed = tgt.speed;
    waypoints{3}.time = 1;
    % construct QP
    r_xyz = 1; r_yaw = 1;                                                   % regularization 
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
       Aeq = [Aeq;[waypoints{i}.time^4,waypoints{i}.time^3,waypoints{i}.time^2,waypoints{i}.time,1,zeros(1,13)]];            % x
       Aeq = [Aeq;[zeros(1,5),waypoints{i}.time^4,waypoints{i}.time^3,waypoints{i}.time^2,waypoints{i}.time,1,zeros(1,8)]];  % y
       Aeq = [Aeq;[zeros(1,10),waypoints{i}.time^4,waypoints{i}.time^3,waypoints{i}.time^2,waypoints{i}.time,1,zeros(1,3)]]; % z
       Aeq = [Aeq;[zeros(1,15),waypoints{i}.time^2,waypoints{i}.time,1]];                                                    % yaw
       beq = [beq;waypoints{i}.position(1);waypoints{i}.position(2);waypoints{i}.position(3);waypoints{i}.heading];
    end
    Aeq = [Aeq;[waypoints{1}.time^3,waypoints{1}.time^2,waypoints{1}.time,1,0,zeros(1,13)]];                              % x'
    Aeq = [Aeq;[zeros(1,5),waypoints{1}.time^3,waypoints{1}.time^2,waypoints{1}.time,1,0,zeros(1,8)]];                    % y'
    Aeq = [Aeq;[zeros(1,10),waypoints{1}.time^3,waypoints{1}.time^2,waypoints{1}.time,1,0,zeros(1,3)]];                   % z'
    beq = [beq;waypoints{1}.speed(1);waypoints{1}.speed(2);waypoints{1}.speed(3)];
    % corridor constraints (inequality)
    nc = 2;
    epsilon = 0.05;
    corridor.start = waypoints{2};
    corridor.final = waypoints{3};
    A = [];
    b = [];
    for i = 1:nc
        ri_position = corridor.start.position + i/(1+nc) * (corridor.final.position - corridor.start.position);
        ti = corridor.start.time + i/(1+nc) * (corridor.final.time - corridor.start.time);
        A = [A;-[zeros(1,5),ti^4,ti^3,ti^2,ti,1,zeros(1,8)]];
        A = [A;[zeros(1,5),ti^4,ti^3,ti^2,ti,1,zeros(1,8)]];
        A = [A;-[zeros(1,10),ti^4,ti^3,ti^2,ti,1,zeros(1,3)]];
        A = [A;[zeros(1,10),ti^4,ti^3,ti^2,ti,1,zeros(1,3)]];
        b = [b;epsilon - ri_position(2);ri_position(2) + epsilon;epsilon - ri_position(3);ri_position(3) + epsilon];
    end
    %% solving quadratic problem
    options = optimoptions('quadprog','Display','off');
    path_c = quadprog(H,f,A,b,Aeq,beq,[],[],[],options);
end