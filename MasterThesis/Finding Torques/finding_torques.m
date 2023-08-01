
%% Clear data
clc
clear
%% Specify the directory path
directory = 'C:\Users\s202421\Documents\GitHub\MasterThesis\Data\forces\64';
% Get a list of .mat files in the directory
fileList = dir(fullfile(directory, '*.mat'));

muscles = 4;
files = length(fileList);

torques = zeros(5,length(fileList));
configs = zeros(5,length(fileList));
wps= zeros(3,length(fileList));
w_forces = zeros(3,length(fileList));

%load('C:\Users\s202421\Documents\GitHub\MasterThesis\DataCreation\modelpoints\torques\torque_static.mat');
[model, nstates, ndof, nmus, iLce] = initialize_model();
%% Get Static torques
torques4 = zeros(5,64);
configs4= zeros(5,64);
wps4 = zeros(3,64);
w_forces4 = zeros(3,64);
for i = muscles:muscles:length(fileList)
    name = fileList(i).name;
    filePath = fullfile(directory, name );
    data = load(filePath);

    forces = data.forces;
    cut = round(length(forces)*0.9);
    hand_F=mean(forces(cut:end,:))';
    x = data.x;
    [~, ~, ~, ~, ~, ~, qTH] = das3('Dynamics',x, zeros(size(x)), zeros(138,1));
    pose=[qTH;x(10:11)];

    [dPhand_dx, ~, ~] = handpos_jacobian(x);

    torque = dPhand_dx'*hand_F;
    torque_5 = [torque(1:3,:); torque(10:11,:)];

    % Extract the number of the position
    number = regexp(name, '\d+', 'match');
    number = str2double(number(1));

    torques4(:,number)=torque_5;
    configs4(:,number)=pose;
    wps4(:,number) = wrist_position(x);
    w_forces4(:,number)=hand_F;

end

j = muscles-1;
for i = 1:length(fileList)
    % Load the .mat file
    name = fileList(i).name;
    filePath = fullfile(directory, name);
    data = load(filePath);
    % Extract the number of the position
    number = regexp(name, '\d+', 'match');
    number = str2double(number(1));

    if mod(i,4)~=0
        forces = data.forces;
        cut = round(length(forces)*0.9);
        hand_F=mean(forces(cut:end,:))';
        x = data.x;
        [~, ~, ~, ~, ~, ~, qTH] = das3('Dynamics',x, zeros(size(x)), zeros(138,1));
        pose=[qTH;x(10:11)];
        wp = wrist_position(x);
    
        [dPhand_dx, ~, ~] = handpos_jacobian(x);
        torque = dPhand_dx'*hand_F;
        torque_5 = [torque(1:3,:); torque(10:11,:)];
        %muscle_torque = torques4(:,number)-torque_5;
        muscle_torque = torque_5; 
        if number == 1
            torques(:,muscles-j)=torque_5;
            configs(:,muscles-j)=pose;
            wps(:,muscles-j)=wp;
            w_forces(:,muscles-j)=hand_F;

        else
            torques(:,number*muscles-j)=torque_5;
            configs(:,number*muscles-j)=pose;
            wps(:,number*muscles-j)=wp;
            w_forces(:,number*muscles-j)=hand_F;

        end
        j = j-1;
    else
        torques(:,number*muscles-j)=torques4(:,number);
        configs(:,number*muscles-j)=configs4(:,number);
        wps(:,number*muscles-j)=wps4(:,number);
        w_forces(:,number*muscles-j)=w_forces4(:,number);

        j = muscles-1;
    end


end

data_for_model(1).torqueData = {};
data_for_model(1).angleData = {};
data_for_model(1).wristposition = {};
data_for_model(1).wristforces={};
for i=1:muscles
    indx = i:muscles:files;
    torques_i = torques(:,indx);
    configs_i = configs(:,indx);
    wps_i = wps(:,indx);
    w_forces_i = w_forces(:,indx);

    data_for_model(i).torqueData=torques_i';
    data_for_model(i).angleData=configs_i';
    data_for_model(i).wristposition = wps_i';
    data_for_model(i).wristforces = w_forces_i';

    name = append('C:\Users\s202421\Documents\GitHub\MasterThesis\Data/torques/64/',string(i) );
    save(name,'torques_i','configs_i','wps_i', 'w_forces_i');

end
name = append('C:\Users\s202421\Documents\GitHub\MasterThesis\Data/torques/data64.mat' );
save(name,'data_for_model');

fprintf('Data saved for torques')

