function plot_2d_obj(obj)
    hold on;
    scatter(obj.position(1),obj.position(2),5,'filled','MarkerEdgeColor',[0 0 0],'MarkerFaceColor',[0 0 0]);
    hold off;
    axis equal;
    grid on;
    xlim([-100,100]); xlabel('x');
    ylim([-100,100]); ylabel('y');
end