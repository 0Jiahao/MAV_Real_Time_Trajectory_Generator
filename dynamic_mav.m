function obj = dynamic_mav(obj,u,ts)
    g = 9.80665;
    m = 0.53263; % kg
    for i = 1:50
        rotm = eul2rotm(obj.angle);
        % limit the change rate (assume 90 degree in 1 second)
        d = u(2) - obj.angle(1);
        d = mod(d+pi, 2*pi) - pi;
        if abs(d) >= 1/2 * pi / (1/ts*50)
            d = abs(d)/d * 1/2 * pi / (1/ts*50);
        end
        obj.angle(1) = obj.angle(1) + d;
        d = u(3) - obj.angle(2);
        d = mod(d+pi, 2*pi) - pi;
        if abs(d) >= 1/2 * pi / (1/ts*50)
            d = abs(d)/d * 1/2 * pi / (1/ts*50);
        end
        obj.angle(2) = obj.angle(2) + d;
        d = u(4) - obj.angle(3);
        d = mod(d+pi, 2*pi) - pi;
        if abs(d) >= 1/2 * pi / (1/ts*50)
            d = abs(d)/d * 1/2 * pi / (1/ts*50);
        end
        obj.angle(3) = obj.angle(3) + d;
        Kf = 0.5 * eye(3);
    %     A = [zeros(6,3),[eye(3);zeros(3,3)]];  % ideal model
        A = [zeros(6,3),[eye(3);-rotm*Kf*rotm']];
        B = [zeros(3,2);[rotm*[0;0;1],[0;0;-1]]];
        C = [eye(3),zeros(3,3)];
        D = zeros(3,2);
        A_d = eye(6) + A * ts / 50;
        B_d = B * ts / 50;
        states = [obj.position';rotm * obj.speed'];
        states = A_d * states + B_d * [u(1)/m;g];
    %     states(4:6) = states(4:6) + normrnd(0,1.5,3,1) * ts;
        obj.position = states(1:3)';
        obj.speed = (eul2rotm(obj.angle)' * states(4:6))';    % store body frame speed
        if obj.position(3) < 0
           obj.position(3) = 0;
           obj.angle(2:3) = [0,0];
           obj.speed = [0,0,0];        % body frame
        end
    end
end