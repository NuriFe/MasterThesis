%% Initialise variables and model
clear;
clc;
% Some model related variables
load('C:\Users\s202421\Documents\GitHub\MasterThesis\DataCreation\forces\brachialispoint14.mat')
modelparams = load('model_struct'); 
model = modelparams.model;
ndof = model.nDofs;

dofnames = cell(ndof,1);
for idof=1:ndof
    dofnames{idof} = model.dofs{idof}.osim_name;
end


%% Insert time start, end and time step
t = 0;
tstep = .003;
tend = 1;
nsteps = round((tend-t)/tstep);
tout = tstep*(0:nsteps)';
xout = xouts';
file_name='brachialispoint14';
name = append('C:\Users\s202421\Documents/Github/MasterThesis\DataCreation\sto/',file_name );
make_osimm(name, dofnames, xout(:,1:ndof), tout);
fprintf('Simulation result for %s has been saved.\n ',file_name)
