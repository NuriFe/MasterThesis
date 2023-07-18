function plot_wrist_references(wpos,model,tocheck, colour)
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
    if nargin ~= 4
        colour = 'bo';
        size = 1;
    else
        size = 15;
    end
    plot3(x, y, z, colour, 'MarkerSize', size);

    if nargin ==3
        w_bad = wpos(tocheck,:);
        x_r = w_bad(:,3);
        y_r= w_bad(:,1);
        z_r = w_bad(:,2);
        plot3(x_r, y_r, z_r, 'r.', 'MarkerSize', 15);

    end

   
    
    axis('equal');
    box on
    axis([zrange xrange yrange]);
    xlabel('Z');
    ylabel('X');
    zlabel('Y');
    hold off

end