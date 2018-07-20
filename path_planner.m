function [waypoints,path_c] = path_planner(obj,tgt)
    waypoints = cell(2,1);
    waypoints{1}.position = obj.position;                                   % start point
    waypoints{1}.heading = obj.angle(1);
    waypoints{1}.speed = (eul2rotm(obj.angle) * obj.speed')';               % world frame
    waypoints{1}.time = 0;
    waypoints{2}.position = tgt.position;                                   % final point
    waypoints{2}.heading = tgt.angle(1);
    waypoints{2}.time = 2;
    % construct QP
    r_xyz = 1; r_yaw = 1;                                                   % regularization 
    H = zeros(18,18);    % 5 * 3 + 3 (4th order for xyz & 2nd order for yaw)
    H(1,1) = r_xyz;
    H(6,6) = r_xyz;
    H(11,11) = 10000*r_xyz;
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
    % velocity of the start point
    Aeq = [Aeq;[4*waypoints{1}.time^3,3*waypoints{1}.time^2,2*waypoints{1}.time,1,0,zeros(1,13)]];                              % x'
    Aeq = [Aeq;[zeros(1,5),4*waypoints{1}.time^3,3*waypoints{1}.time^2,2*waypoints{1}.time,1,0,zeros(1,8)]];                    % y'
    Aeq = [Aeq;[zeros(1,10),4*waypoints{1}.time^3,3*waypoints{1}.time^2,2*waypoints{1}.time,1,0,zeros(1,3)]];                   % z'
    beq = [beq;waypoints{1}.speed(1);waypoints{1}.speed(2);waypoints{1}.speed(3)];
    % velocity of the final point 
    Aeq = [Aeq;[sin(waypoints{2}.heading)*[4*waypoints{2}.time^3,3*waypoints{2}.time^2,2*waypoints{2}.time,1,0],-cos(waypoints{2}.heading)*[4*waypoints{2}.time^3,3*waypoints{2}.time^2,2*waypoints{2}.time,1,0],zeros(1,8)]];                    % y'
    Aeq = [Aeq;[zeros(1,10),4*waypoints{2}.time^3,3*waypoints{2}.time^2,2*waypoints{2}.time,1,0,zeros(1,3)]];                   % z'
    beq = [beq;0;0];
    %% solving quadratic problem
   options = optimoptions('quadprog','Display','off');
   x0 = zeros(18,1); 
   path_c = quadprog(H,f,[],[],Aeq,beq,[],[],x0,options);
end