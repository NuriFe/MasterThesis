function plot_configuration(qTHout, tout)
    % Plotting all angles in the same plot
    figure;
    qTHout = qTHout(2:end,:);
    tout = tout(2:end,:);
    % Plot shoulder elevation plane
    hold on;
    plot(tout, qTHout(:,1), 'r', 'LineWidth', 1.5);
    
    % Plot shoulder elevation
    plot(tout, qTHout(:,2), 'g', 'LineWidth', 1.5);
    
    % Plot shoulder rotation
    plot(tout, qTHout(:,3), 'b', 'LineWidth', 1.5);
    
    hold off;
    
    % Add labels and legend
    title('Shoulder Angles over Time');
    xlabel('Time');
    ylabel('Angle');
    legend('Shoulder Elevation Plane', 'Shoulder Elevation', 'Shoulder Rotation');
end