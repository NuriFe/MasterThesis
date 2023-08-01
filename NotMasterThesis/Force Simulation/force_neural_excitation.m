%% Initialise variables and model
% Some model related variables
clc;
clear;
modelparams = load('model_struct'); 
model = modelparams.model;

ndof = model.nDofs;
nmus = model.nMus;
nstates = 2*ndof + 2*nmus;

% Initialize the model
das3('Initialize',model);
% Define indices to the state variables within the state vector x
iq = 1:ndof;
iqdot = max(iq) + (1:ndof);
iLce = max(iqdot) + (1:nmus);

%% Insert time start, end and time step
tstep = .003;
tend = 1;
t = 0;
nsteps = round((tend-t)/tstep);
%% Create space for variables

muscle_name= 'triceps';
%muscle_name= 'deltascap';
muscles = 37:1:47;
%muscle_name= 'brachialis'
%muscles = 109:1:115;
directory = 'C:\Users\s202421\Documents/GitHub/MasterThesis\DataCreation\modelpoints';
% Get a list of .mat files in the directory
fileList = dir(fullfile(directory, '*.mat'));

tic;
for i = 1:length(fileList)
    % Load the .mat file
    filePath = fullfile(directory, fileList(i).name);
    data = load(filePath);
    
    xout = data.xout;
    x = xout(:,end);
    handF_total = data.handF_total;
    handF = handF_total(end,:)';
    % Input Function
    u_total = stim_fun(muscles, tstep, tend);
    [fs,ers,xouts,qTHs] = simulate_force_PID(nmus,nstates,x, handF, u_total,tstep,nsteps);
    %keyboard
    filename = strrep(fileList(i).name, '.mat', '');
    name = append('C:\Users\s202421\Documents/Github/MasterThesis\DataCreation\modelpoints/forces/triceps/',muscle_name,filename);
    save(name,'fs','ers','xouts','qTHs');
end
toc
