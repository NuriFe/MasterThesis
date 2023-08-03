function plot_neurexct(t,u)
% Loop through each muscle and create subplots
        hold on

for muscle = 1:9
    [muscle_indices] = muscledict(muscle);
    
    if ~isempty(muscle_indices)
        
        subplot(3, 3, muscle);
        plot(t, u(:,muscle_indices));
        title(['Index: ', num2str(muscle)]);
        ylim([-0.1, 1.1]); % Set y-axis range to be from 0 to 1
        xlim([t(1) t(end)]);
    end
end

end