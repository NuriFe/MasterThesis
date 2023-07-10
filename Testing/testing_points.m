
%% Clear data
clc
clear
%% Specify the directory path
directory = 'C:\Users\s202421\Documents/GitHub/MasterThesis\DataCreation\modelpoints';
% Initialize the model
modelparams = load('model_struct'); 
model = modelparams.model;
das3('Initialize',model);

% Get a list of .mat files in the directory
fileList = dir(fullfile(directory, '*.mat'));
w_refs = create_grid(0.1);
tocheck = [];
%totest = [22 24 25 26 27 9]';
%totest = [22 24 26];
for i = 1:length(fileList)
    % Load the .mat file
    name = fileList(i).name;
    filePath = fullfile(directory, name);
    
    % Extract the number using regular expressions
    number = regexp(name, '\d+', 'match');
    % Convert the extracted number to a numeric value
    number = str2double(number);
    %if ~ismember(number,totest)
    %    continue
    %end

    data = load(filePath);
    
    % Access the error data from the loaded .mat file
    fs=data.handF_total; 
    xouts = data.xout;
    wref = w_refs(number,:)';
    [error,xout_test] = testing(xouts, fs,wref,model);
    wrist = string(wref');
    wrist_error = string(roundn(error',-3));
    
    message = sprintf('Data for w_ref %s x:%s+%s y:%s+%s z:%s+%s\n ', ...
        string(number),wrist(1,1), wrist_error(1,1), wrist(1,2), wrist_error(1,2), wrist(1,3), wrist_error(1,3));
    
    if any(abs(error)>0.04)
        %warning(message);
        %disp(fs(end,:));
        tocheck = [tocheck number];
        %figure();
        %plot_wrist_positions(xout_test,model,wref)
        %title(name);
    else
        fprintf(message);
    end
    

    plot_wrist_references(w_refs,model,tocheck);
end
l = string(length(fileList));
name = append('C:\Users\s202421\Documents/Github/MasterThesis\DataCreation\points/testing',l );
save(l,'directory','w_refs','tocheck');