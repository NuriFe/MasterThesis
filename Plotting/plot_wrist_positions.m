function plot_wrist_positions(xout,model,w_ref)
    % Plot 3D points and join them in order
    x = [];
    y = [];
    z = [];
    
    xrange = [-0.3 0.6];
	yrange = [-1 0.5];
	zrange = [-0.45 0.45];

    for i = 1:size(xout,1)
            
            state = xout(i,:)';
            pos = wrist_position(state);
            x = [x pos(3)];
            y = [y pos(1)];
            z = [z pos(2)];
    end
    %figure(1);clf
    draw_ellipsoid(model)
    hold on

    plot3(x, y, z, 'o-', 'LineWidth', 0.5, 'MarkerSize', 1);
    % Plot the first and last point with a different color and larger size
    plot3(x(1), y(1), z(1), 'go', 'LineWidth', 1.5, 'MarkerSize', 4);  % Green marker for the first point
    plot3(x(end), y(end), z(end), 'ro', 'LineWidth', 1.5, 'MarkerSize', 4);  % Red marker for the last point
    % Add labels and title
    
    if nargin>1
        draw_arm(xout(1,:)',model);
        draw_arm(xout(end,:)',model);
        if nargin>2;
           plot3(w_ref(3), w_ref(1), w_ref(2), 'bo', 'LineWidth', 1.5, 'MarkerSize', 4);
    end
        % set the axes and viewpoint
    axis('equal');
    box on
    axis([zrange xrange yrange]);
    xlabel('Z');
    ylabel('X');
    zlabel('Y');
    hold off

end