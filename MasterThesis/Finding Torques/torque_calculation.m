
%% Clear data
clc
clear
%% Specify the directory path

directory = 'C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis\Data\Stimulated Forces';
muscles = 10;
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
    for i = j:10:length(fileList)
        name = fileList(i).name;
        filePath = fullfile(directory, name );
        data = load(filePath);
    
        forces = data.forces;
        cut = round(length(forces)*0.9);
        hand_F=mean(forces(cut:end,:))';
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


name = append('C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis\Data\Static Torques/data64.mat' );
save(name,'data_for_model');

fprintf('Data saved for torques')

