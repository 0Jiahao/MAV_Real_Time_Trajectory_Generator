function obj = dynamic_mav(obj,u,ts)
    g = 9.80665;
    obj.angle(2:3) = u(3:4);
    obj.angle(1) = obj.angle(1) + u(2) * ts;
%     obj.angle = obj.angle + randn(1,3)/50;     
    % kx = 0.5 and ky = 0.6 data given by Shuo Li
    Kf = 0.5 * eye(3);
%     A = [zeros(6,3),[eye(3);-eul2rotm(obj.angle)*Kf]];
%     A = [zeros(6,3),[eye(3);zeros(3,3)]];
    A = [zeros(6,3),[eye(3);-eul2rotm(obj.angle)*Kf*inv(eul2rotm(obj.angle))]];
    B = [zeros(3,2);[eul2rotm(obj.angle)*[0;0;1],[0;0;-1]]];
    C = [eye(3),zeros(3,3)];
    D = zeros(3,2);
    A_d = eye(6) + A * ts;
    B_d = B * ts;
    states = [obj.position';eul2rotm(obj.angle) * obj.speed'];
    states = A_d * states + B_d * [u(1);g];
    obj.position = states(1:3)';
%     obj.position = obj.position + randn(1,3)/30; % added randome noise
    obj.speed = (inv(eul2rotm(obj.angle)) * states(4:6))';    % store body frame speed
    if obj.position(3) < 0
       obj.position(3) = 0;
       obj.angle(2:3) = [0,0];
       obj.speed = [0,0,0];        % body frame
    end
end