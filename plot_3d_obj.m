function plot_3d_obj(obj)
    scatter3(obj.position(1),obj.position(2),obj.position(3),5,'filled','MarkerEdgeColor',[0 0 0],'MarkerFaceColor',[0 0 0]);
    rotm = eul2rotm(obj.angle);
    oPt = obj.position;
    xPt = (obj.position + (rotm * [5,0,0]')') ;
    yPt = (obj.position + (rotm * [0,5,0]')');
    zPt = (obj.position + (rotm * [0,0,5]')');
    line([oPt(1) xPt(1)],[oPt(2) xPt(2)],[oPt(3) xPt(3)],'Color','red');
    line([oPt(1) yPt(1)],[oPt(2) yPt(2)],[oPt(3) yPt(3)],'Color','green');
    line([oPt(1) zPt(1)],[oPt(2) zPt(2)],[oPt(3) zPt(3)],'Color','blue');
    axis equal;
    xlim([-100,100]); xlabel('x');
    ylim([-100,100]); ylabel('y');
    zlim([0,200]); zlabel('z');
end
