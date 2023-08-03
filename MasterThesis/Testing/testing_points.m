
%% Clear data
clc
clear
close all
%% Specify the directory path
directory = 'C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis\Data\Stimulated Forces';

% Get a list of .mat files in the directory
fileList = dir(fullfile(directory, '*.mat'));
%points
%w_refs = create_grid(0.1);
muscle = 1;
%forces
w_refs = [];
name = '';

tocheck = [];
error_to_check = [];

for i = 1:length(fileList)
    % Load the .mat file
    name = fileList(i).name;
    filePath = fullfile(directory, name);
    
    % Extract the number using regular expressions
    number = regexp(name, '\d+', 'match');
    if str2double(number(2))~=muscle
        continue
    end
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
    xouts = data.xout;
    xref = xouts(1,:)';
    wref = wrist_position(xref);
    w_refs = [w_refs wref];

    %%
    tend = 2;
    tstep = 0.001;
    [error,xout_test] = testing(xouts, fs,wref,muscle, tend, tstep);
    wrist = string(wref');
    wrist_error = string(roundn(error',-3));
    
    message = sprintf('Data for w_ref %s x:%s+%s y:%s+%s z:%s+%s\n ', ...
        string(number),wrist(1,1), wrist_error(1,1), wrist(1,2), wrist_error(1,2), wrist(1,3), wrist_error(1,3));
    
    if any(abs(error)>0.04)
        %warning(message);
        %disp(fs(end,:));
        tocheck = [tocheck number];
        error_to_check = [error_to_check wrist_error];
        %figure();
        %plot_wrist_positions(xout_test,model,wref)
        %title(name);
    else
        fprintf(message);
    end
    

end
plot_wrist_references(w_refs',model,tocheck);

l = append(string(length(fileList)),'_', string(muscle));
name = append('C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis\Data\Static Forces/Testing/',l );
save(name,'directory','w_refs','tocheck');