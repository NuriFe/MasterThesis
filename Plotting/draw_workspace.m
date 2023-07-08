function draw_workspace(w_pos,model)

    x = w_pos(1,:);
    y = w_pos(2,:);
    z = w_pos(3,:);
    
	% set plotting volume 
	xrange = [-0.3 0.6];
	yrange = [-1 0.5];
	zrange = [-0.35 0.35];
    
    cla
    draw_ellipsoid(model)
    hold on

    scatter3(x, y, z, 'filled');
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('3D Scatter Plot');
	
    % set the axes and viewpoint
    axis('equal');
    box on
    axis([zrange xrange yrange]);
    xlabel('Z');
    ylabel('X');
    zlabel('Y');
    hold off

end

