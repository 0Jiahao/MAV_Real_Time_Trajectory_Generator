function obj = dynamic_mav(obj,u,ts)
    g = 9.80665;
    obj.angle = u(2:4); 
    % kx = 0.5 and ky = 0.6
    Kf = [0.5,0,0;0,0.6,0;0,0,0];
%     A = [zeros(6,3),[eye(3);-eul2rotm(obj.angle)*Kf]];
%     A = [zeros(6,3),[eye(3);zeros(3,3)]];
    A = [zeros(6,3),[eye(3);-eul2rotm(obj.angle)*Kf*inv(eul2rotm(obj.angle))]];
    B = [zeros(3,2);[eul2rotm(obj.angle)*[0;0;1],[0;0;-1]]];
    C = [eye(3),zeros(3,3)];
    D = zeros(3,2);
    sys = ss(A,B,C,D);
    sys_d = c2d(sys,ts);
    states = [obj.position';eul2rotm(obj.angle) * obj.speed'];
    states = sys_d.A * states + sys_d.B * [u(1);g];
    obj.position = states(1:3)';
    obj.speed = (inv(eul2rotm(obj.angle)) * states(4:6))';    % store body frame speed
%     if obj.position(3) < 0
%        obj.position(3) = 0;
%        obj.angle = [0,0,0];
%        obj.speed = [0,0,0];        % body frame
%     end
end