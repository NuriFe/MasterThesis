function plotting_triceps_FES(t, u_triceps, FES, total_u)
    % Check if the input vectors have the same length
    if length(t) ~= length(u_triceps) || length(t) ~= length(FES) || length(t) ~= length(total_u)
        error('All input vectors must have the same length.');
    end

    % Create a new figure
    figure;

    % Plot all data on the same plot
    plot(t, u_triceps, 'Color', [1, 0.5, 0], 'DisplayName', 'Triceps');
    hold on;
    plot(t, FES, 'r', 'DisplayName', 'FES');
    plot(t, total_u, 'b', 'DisplayName', 'Total u');
    
    % Add title, labels, and legend
    title('Muscle Activity, FES and Neural Excitation Input over Time');
    xlabel('Time (t)');
    legend('show');

    % Turn off hold
    hold off;
end

