
%% Clear data
clc
clear
%% Specify the directory path
directory = 'C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis\Data\Static Forces';
% Initialize the model
modelparams = load('model_struct'); 
model = modelparams.model;
das3('Initialize',model);

% Get a list of .mat files in the directory
fileList = dir(fullfile(directory, '*.mat'));
%points
w_refs = create_grid(0.1);
muscles = 0;
%forces
%w_refs = [];
name = '';

%muscles = 83:1:85; %biceps
%muscles = 109:1:115; %brachialis
%muscles = 48:1:51;
tocheck = [];
error_to_check = [];
%totest = [22 24 25 26 27 9]';
%totest = [15 17 18];
for i = 1:length(fileList)
    % Load the .mat file
    name = fileList(i).name;
    filePath = fullfile(directory, name);
    
    % Extract the number using regular expressions
    number = regexp(name, '\d+', 'match');
    % Convert the extracted number to a numeric value
    number = str2double(number(1));
    %if ~ismember(number,totest)
    %    continue
    %end

    data = load(filePath);
    
    % Access the error data from the loaded .mat file
    %points
    forces=data.forces; 
    fs = data.mean_force;
    %fs = forces(end,:);
    xouts = data.xout;
    wref = w_refs(number,:)';

    %forces
    %fs=data.forces; 
    %xouts = data.xout;
    %xref = xouts(1,:)';
    %wref = wrist_position(xref);
    %w_refs = [w_refs wref];
    %%
    tend = 3;
    tstep = 0.001;
    [error,xout_test] = testing(xouts, fs,wref,name,muscles,model, tend, tstep);
    wrist = string(wref');
    wrist_error = string(roundn(error',-3));
    
    message = sprintf('Data for w_ref %s x:%s+%s y:%s+%s z:%s+%s\n ', ...
        string(number),wrist(1,1), wrist_error(1,1), wrist(1,2), wrist_error(1,2), wrist(1,3), wrist_error(1,3));
    
    if any(abs(error)>0.04)
        warning(message);
        %disp(fs(end,:));
        tocheck = [tocheck number];
        end_wrist = wrist_position(xouts(end,:)');
        error = wref -end_wrist
        error_to_check = [error_to_check error];
        %figure();
        %plot_wrist_positions(xout_test,model,wref)
        %title(name);
    else
        fprintf(message);
    end
    
    

end
plot_wrist_references(w_refs,model,tocheck);

l = string(length(fileList));
name = append('C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis\Data\Static Forces/Testing/',l );
save(name,'directory','w_refs','tocheck');