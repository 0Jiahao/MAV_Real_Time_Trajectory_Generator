function obj = dynamic_mav(obj,u,ts)
    g = 9.80665;
    m = 0.53263; % kg
    rotm = eul2rotm(obj.angle);
    obj.angle(1:3) = u(2:4);
    Kf = 0.5 * eye(3);
%     A = [zeros(6,3),[eye(3);zeros(3,3)]];  % ideal model
    A = [zeros(6,3),[eye(3);-rotm*Kf*rotm']];
    B = [zeros(3,2);[rotm*[0;0;1],[0;0;-1]]];
    C = [eye(3),zeros(3,3)];
    D = zeros(3,2);
    A_d = eye(6) + A * ts;
    B_d = B * ts;
    w = B_d * [u(1)/m;g];
%     w(end)
%     w(end)
    states = [obj.position';rotm * obj.speed'];
    states = A_d * states + B_d * [u(1)/m;g];
    obj.position = states(1:3)';
    obj.speed = (eul2rotm(obj.angle)' * states(4:6))';    % store body frame speed
    if obj.position(3) < 0
       obj.position(3) = 0;
       obj.angle(2:3) = [0,0];
       obj.speed = [0,0,0];        % body frame
    end
end