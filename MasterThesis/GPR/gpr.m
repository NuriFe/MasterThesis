%% Clear data
clc
clear
%% Specify the directory path
directory = 'C:\Users\s202421\Documents/GitHub/MasterThesis\DataCreation\modelpoints/';
fileList = dir(fullfile(directory, '*.mat'));

for i = 1:length(fileList)
    % Load the .mat file
    name = fileList(i).name;
    filePath = fullfile(directory, name);
    data = load(filePath);

    tableData=struct2table(data);
    %variableNames = {''}
    
end