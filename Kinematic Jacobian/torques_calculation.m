
%% Clear data
clc
clear
%% Specify the directory path
directory = 'C:\Users\s202421\Documents/GitHub/MasterThesis\DataCreation\modelpoints/forces/biceps';
% Get a list of .mat files in the directory
fileList = dir(fullfile(directory, '*.mat'));
torques = zeros(11,length(fileList));
configs = zeros(5,length(fileList));

load('C:\Users\s202421\Documents\GitHub\MasterThesis\DataCreation\modelpoints\torques\torque_static.mat');

for i = 1:length(fileList)
    % Load the .mat file
    filePath = fullfile(directory, fileList(i).name);
    data = load(filePath);
    static_torque = torques(:,i);
    % Access the data from the loaded .mat file
    hand_Fs=data.fs';%handF_total
    hand_F = hand_Fs(:,end);
    xout = data.xouts; %xout
    %config = data.config;
    qTH = data.qTHs;
    x = xout(:,end);

    config = [qTH' x(10,1) x(11,1)]';

    [dPhand_dx, ~, ~] = handpos_jacobian(x);
    torque = dPhand_dx'*hand_F;
    muscle_torque = static_torque-torque;
    torques(:,i)=muscle_torque;
    configs(:,i)=config;
end

name = append('C:\Users\s202421\Documents/Github/MasterThesis\DataCreation\modelpoints/torques/biceps',string(i) );
% Export to mat file and to opensim motion file
save(name,'torques','config');
fprintf('Data saved for torques')
