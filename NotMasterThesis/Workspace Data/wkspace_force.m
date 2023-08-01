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
limit_x = [-3,3];
limit_y = [5,10];
limit_z = [-10,-5];
limits = [limit_x; limit_y; limit_z];
   
number = 6;
forces = zeros(length(limits),number);
for i=1:length(limits)
    limit = limits(i,:);
    forces(i,:) = linspace(limit(1), limit(2),number);
    
end


comb = combinations(forces(1,:),forces(2,:),forces(3,:));
comb = table2array(comb);
% Define indices to the state variables within the state vector x
iq = 1:ndof;
iqdot = max(iq) + (1:ndof);
iLce = max(iqdot) + (1:nmus);


load('equilibrium.mat');
x_valid = [x];
x_nonvalid = [zeros(298,1)];
w_pos = [wrist_position(x)];
total_it = length(comb);
progressBar = waitbar(0, 'Progress; 0%');

for i= 1:total_it
    progress = i/total_it;
    waitbar(progress, progressBar, sprintf('Progress: %.1f%%', progress * 100))
    
    handF = comb(i,:)';
    [xout, tout] = simulate_force(0,x,model,handF);

    x = xout(end,:)';
    valid = config_valid(x,model);

    wrist = wrist_position(x);
    if valid == true && (wrist(1) > -0.1)
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

name = append('C:\Users\s202421\Desktop\DataCreation\w_pos_forces');
% Export to mat file and to opensim motion file
save(name,'w_pos','x_valid');
    


