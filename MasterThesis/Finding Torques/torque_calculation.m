%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TORQUE CALCULATOR FROM FORCES 
%
% Description:
% This script processes the forces under muscle stimulation to derive the 
% corresponding torques, joint configurations, wrist positions, and wrist 
% forces during different poses. The code fetches data from a predefined 
% directory, integrates it with OpenSim models, computes dynamics, 
% and collates the results.
%
% Inputs:
% - Preprocessed '.mat' files containing force data for each pose and muscle 
%   group under stimulated conditions, located in the specified directory.
%
% Outputs:
% - torqueData: 5xN matrix of torques for N poses.
% - angleData: 5xN matrix of joint configurations for N poses.
% - wristposition: 3xN matrix showing wrist positions for N poses.
% - wristforces: 3xN matrix indicating resultant wrist forces for N poses.
%
% Notes:
% - The code assumes a structured '.mat' file format for the force data.
% - Ensure 'das3' function and required models are in the MATLAB path.
% - Modify the 'directory' variable to point to the correct data source.
% - Data will be saved to a predefined directory; adjust path if necessary.
% - The script is optimized for 64 data points; adjust loop structures for other data sizes.
%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Clear data
clc
clear
%% Specify the directory path

directory = 'C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis\Data\Stimulated Forces';
muscles = 9;
fileList = dir(fullfile(directory, '*.mat'));
n_pos = length(fileList)/muscles;
torques = zeros(5,n_pos);
configs = zeros(5,n_pos);
wps= zeros(3,n_pos);
w_forces = zeros(3,n_pos);

[model, nstates, ndof, nmus, iLce] = initialize_model();

data_for_model(muscles).torqueData = {};
data_for_model(muscles).angleData = {};
data_for_model(muscles).wristposition = {};
data_for_model(muscles).wristforces={};

for j = 1:muscles
    torques = zeros(5,n_pos);
    configs = zeros(5,n_pos);
    wps= zeros(3,n_pos);
    w_forces = zeros(3,n_pos);
    for i = j:(muscles):length(fileList)
        name = fileList(i).name;
        filePath = fullfile(directory, name );
        data = load(filePath);
    
        forces = data.forces;
        cut = round(length(forces)*0.9);
        hand_F=data.mean_force';
        x = data.x;
        [~, ~, ~, ~, ~, ~, qTH] = das3('Dynamics',x, zeros(size(x)), zeros(138,1));
        pose=[qTH;x(10:11)];
    
        J = kinematic_jacobian(x);
        torque = J'*hand_F;
    
        % Extract the number of the position
        number = regexp(name, '\d+', 'match');
        number = str2double(number(1));
    
        torques(:,number)=torque;
        configs(:,number)=pose;
        wps(:,number) = wrist_position(x);
        w_forces(:,number)=hand_F;
    
    end
    muscle = regexp(name, '\d+', 'match');
    muscle = str2double(muscle(2));
    data_for_model(muscle).torqueData = torques';
    data_for_model(muscle).angleData = configs';
    data_for_model(muscle).wristposition = wps';
    data_for_model(muscle).wristforces=w_forces';
end

% Calculate the generated torque
static_torque = data_for_model(muscles).torqueData;
for i = 1:muscles-1
    total_torque = data_for_model(i).torqueData;
    stimulated_torque = static_torque-total_torque;
    data_for_model(i).torqueData = stimulated_torque;
end

name = append('C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis\Data\Torques/data64.mat' );
save(name,'data_for_model');

fprintf('Data saved for torques')

