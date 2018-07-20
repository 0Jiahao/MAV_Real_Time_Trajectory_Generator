function plot_path(waypoints,path_c)
    hold on; 
    for i = 1:numel(waypoints)   
        scatter3(waypoints{i}.position(1),waypoints{i}.position(2),waypoints{i}.position(3),5,'filled','MarkerEdgeColor',[0 0 0],'MarkerFaceColor',[0 0 0]);
    end
    t = 0:0.01:2;
    x = path_c(1) * t.^4 + path_c(2) * t.^3 + path_c(3) * t.^2 + path_c(4) * t + path_c(5);
    y = path_c(6) * t.^4 + path_c(7) * t.^3 + path_c(8) * t.^2 + path_c(9) * t + path_c(10); 
    z = path_c(11) * t.^4 + path_c(12) * t.^3 + path_c(13) * t.^2 + path_c(14) * t + path_c(15);
    yaw = path_c(16) * t.^2 + path_c(17) * t + path_c(18);
    plot3(x,y,z,'k'); axis equal;
    hold off;
end