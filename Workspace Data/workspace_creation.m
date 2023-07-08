clc;
clear; 
% Some model related variables
modelparams = load('model_struct'); 
model = modelparams.model;

ndof = model.nDofs;
nmus = model.nMus;
nstates = 2*ndof + 2*nmus;

% Initialize the model
das3('Initialize',model);

muscle = 0;
muscle_name ='new_state_2';
limits = das3('Limits');
limits = limits(:,5:end-1);

angles = zeros(length(limits),5);
for i=1:length(limits)
    limit = limits(:,i);
    angles(i,:) = linspace(limit(1), limit(2),5);
    
end

comb = combinations(angles(1,:),angles(2,:),angles(3,:),angles(4,:), angles(6,:));
comb = table2array(comb);
% Define indices to the state variables within the state vector x
iq = 1:ndof;
iqdot = max(iq) + (1:ndof);
iLce = max(iqdot) + (1:nmus);


load('equilibrium.mat');
x_valid = [x];
x_nonvalid = [zeros(298,1)];
w_pos = [wrist_position(x)];
coordinates = [5 6 7 8 10];
total_it = length(comb);
progressBar = waitbar(0, 'Progress; 0%');
for i= 1:total_it
    progress = i/total_it;
    waitbar(progress, progressBar, sprintf('Progress: %.1f%%', progress * 100));
    x(coordinates) = comb(i,:);

    x(iLce) = 1.0;
    lengths = das3('Musclelengths',x);
    LCEopt = das3('LCEopt');
    SEEslack = das3('SEEslack');
    x(iLce) = (lengths - SEEslack)./LCEopt;

    valid = config_valid(x,model);
    wrist = wrist_position(x);
    if valid == true && (wrist(1) > -0.1 || wrist(3)> 0.1 || wrist(3)<-0.1)
        valid = false;
    end
    if valid == true
        w_pos = [w_pos wrist_position(x)];
        x_valid = [x_valid x];
        
    else
        x_nonvalid = [x_nonvalid x];
    end 
    
end
close(progressBar);
cla;
draw_workspace(w_pos, model);
    
disp('Write dbcont if you want this result saved. Dbquit to quit');
keyboard

name = append('C:\Users\s202421\Desktop\DataCreation\w_pos_1');
% Export to mat file and to opensim motion file
save(name,'w_pos','x_valid');
    


