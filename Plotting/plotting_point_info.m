
%% Clear data
clc
clear
%% Specify the directory path
directory = 'C:\Users\s202421\Documents/GitHub/MasterThesis\DataCreation\forces';
% Initialize the model
modelparams = load('model_struct'); 
model = modelparams.model;
das3('Initialize',model);

% Get a list of .mat files in the directory
fileList = dir(fullfile(directory, '*.mat'));
w_refs = create_grid(0.1);
% Loop through each .mat file
table = zeros(length(fileList),9);
wrists = zeros(length(fileList),3);


figure(); 

for i = 1:length(fileList)
    % Load the .mat file
    name = fileList(i).name;
    filePath = fullfile(directory, name);
    
    % Extract the number using regular expressions
    number = regexp(name, '\d+', 'match');
    % Convert the extracted number to a numeric value
    number = str2double(number);

    data = load(filePath);
    
    % Access the error data from the loaded .mat file
    errors= data.ers;
    error = errors(end,:);
    hand_Fs=data.fs';
    hand_F = hand_Fs(:,end);
    
    xout = data.xouts;
    x = xout(:,end);
    wrists(i,:)=wrist_position(x);
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
    plot_wrist_positions(xout',model,w_refs(number,:));
    title(name)
    xlim([-0.318 0.318])
    ylim([-0.259 0.559])
    zlim([-0.93 0.43])
    view([-50.55 25.43])
    hold off
    table(i,1:3)=w_refs(i,:);
    table(i,4:6)=error;
    table(i,7:9)=hand_F';
    keyboard
end
%%
figure()
plot_wrist_references(w_refs(1:i,:),model);
hold on
plot_wrist_references(wrists,model,'g.')
xlim([-0.318 0.318])
ylim([-0.259 0.559])
zlim([-0.93 0.43])
view([-50.55 25.43])
hold off

% Print table header
fprintf('   x      y      z      x_err  y_err  z_err    Fx     Fy     Fz\n');
fprintf('----------------------------------------------------------------\n');
% Print data rows
for i = 1:size(table, 1)
    fprintf('%6.2f  %6.2f  %6.2f  %5.2f  %5.2f  %5.2f  %5.2f  %5.2f  %5.2f\n', table(i, :));
end
