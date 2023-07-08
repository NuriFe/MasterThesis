function plot_wrist_references(wpos,model)
    % Plot 3D points and join them in order
    x = wpos(:,3);
    y = wpos(:,1);
    z = wpos(:,2);
    
    xrange = [-0.3 0.6];
	yrange = [-1 0.5];
	zrange = [-0.35 0.35];

    %figure(1);clf
    draw_ellipsoid(model)
    hold on


    plot3(x, y, z, 'b.', 'MarkerSize', 15);
   
    
    axis('equal');
    box on
    axis([zrange xrange yrange]);
    xlabel('Z');
    ylabel('X');
    zlabel('Y');
    hold off

end