% Load Positions
clear
clc
close all
directory = 'C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis/Data\Stimulated Forces';
fileList = dir(fullfile(directory, '*.mat'));
rows = 3;
cols = 3;
[model, nstates, ndof, nmus, iLce] = initialize_model();
cell_array = {};
to_check = [];
muscle_to_check = [];
muscles = 9;
for i = 1:muscles:length(fileList)
    
    for j = 1:min(muscles, length(fileList) - i + 1)
        % Load the .mat file
        name = fileList(i + j - 1).name;
        filePath = fullfile(directory, name);
        data = load(filePath);
        xout = data.xout;
        number = regexp(name, '\d+', 'match');
        if str2double(number(2))==2
            continue
        end

        % Calculate the error
        error = wrist_position(xout(end,:)') - wrist_position(xout(1,:)');

        % Create a subplot
        %subplot(rows, cols, str2double(number(2)));
        %plot_wrist_positions(xout, model)
        %title(name);

           % Check if any error is higher than 0.05
        if any(abs(error) > 0.05)
            to_check = [to_check str2double(number(1))];
            muscle_to_check = [muscle_to_check str2double((number(2)))];
            
            plot_wrist_positions(xout, model)
        else
            
            %text(1.5, 0.5, num2str(error));
        end

    end

    %keyboard(); % Optional pause to inspect the figure
    %clf;  % Close the figure after inspection
end

matrix = [to_check; muscle_to_check]
