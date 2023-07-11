
%% Clear data
clc
clear
%% Specify the directory path
directory = 'C:\Users\s202421\Documents/GitHub/MasterThesis\DataCreation\modelpoints';
wrefs = create_grid(0.1);
fileList = dir(fullfile(directory, '*.mat'));

for i = 1:length(fileList)
    % Load the .mat file
    filePath = fullfile(directory, fileList(i).name);
    data = load(filePath);

    % Access the data from the loaded .mat file
    hand_Fs=data.handF_total';%handF_total
    hand_F = hand_Fs(:,end);
    xout = data.xout; %xout
    
    if wrefs(i,2)==-0.15
        disp(hand_F(2,:));
    end

end
