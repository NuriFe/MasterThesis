load('C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis\Data\Static Torques\data64.mat')

directory = 'C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis\Data\Stimulated Forces';
fileList = dir(fullfile(directory, '*.mat'));

positions = zeros(length(fileList),11);
for i = 1:length(fileList)
    % Load the .mat file
    name = fileList(i).name;
    filePath = fullfile(directory, name);
     % Extract the number using regular expressions
    number = regexp(name, '\d+', 'match');
    % Convert the extracted number to a numeric value
    number = str2double(number(1));
    data = load(filePath);
    armTorque = data.armTorque;
    plot(armTorque);

    ylim([-5 5]);

    % Calculate the index corresponding to the last 10% of the array
    last_10_percent_index = ceil(0.9 * length(armTorque)):length(armTorque);
    
    % Extract the last 10% of the array
    last_10_percent = armTorque(last_10_percent_index,:);
    
    % Calculate the mean of the last 10% of the array
    mean_last_10_percent = mean(last_10_percent)'

end