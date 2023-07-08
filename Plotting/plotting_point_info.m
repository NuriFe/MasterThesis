
%% Clear data
clc
clear
%% Specify the directory path
directory = 'C:\Users\s202421\Documents/GitHub/MasterThesis\DataCreation\points';
% Initialize the model
modelparams = load('model_struct'); 
model = modelparams.model;
das3('Initialize',model);

% Get a list of .mat files in the directory
fileList = dir(fullfile(directory, '*.mat'));
w_refs = create_grid(0.1);
% Loop through each .mat file
table = [zeros(length(fileList),9)];
figure(); 

for i = 1:length(fileList)
    % Load the .mat file
    filePath = fullfile(directory, fileList(i).name);
    data = load(filePath);
    
    % Access the error data from the loaded .mat file
    errors= data.error;
    error = errors(end,:);
    hand_Fs=data.handF_total';
    hand_F = hand_Fs(:,end);
    
    xout = data.xout;

    name =append('point',string(i) );
    %load('equilibrium.mat')
    %[xout, tout] = neurext_handf(name, 0 ,hand_F,x,5);
    % Call the plot_error function
    clf
    subplot(1, 3, 1);
    hold on 
    plot_error_position(errors);
    subplot(1, 3, 2);
    plot_hand_forces(hand_Fs);
    subplot(1, 3, 3);
    plot_wrist_positions(xout',model,w_refs(i,:));
    
    table(i,1:3)=w_refs(i,:);
    table(i,4:6)=error;
    table(i,7:9)=hand_F';
    keyboard
end
%%

% Print table header
fprintf('   x      y      z      x_err  y_err  z_err    Fx     Fy     Fz\n');
fprintf('----------------------------------------------------------------\n');
% Print data rows
for i = 1:size(table, 1)
    fprintf('%6.2f  %6.2f  %6.2f  %5.2f  %5.2f  %5.2f  %5.2f  %5.2f  %5.2f\n', table(i, :));
end
