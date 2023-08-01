%% Initialise variables and model
clear;
clc;
% Some model related variables
directory = 'C:\Users\s202421\Documents/GitHub/MasterThesis\DataCreation\modelpoints/';
fileList = dir(fullfile(directory, '*.mat'));
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
tend = 3;
nsteps = round((tend-t)/tstep);
tout = tstep*(0:nsteps)';


for i = 1:length(fileList)
    file_name=fileList(i).name;
    filePath = fullfile(directory, file_name);
    data = load(filePath);

    xout=data.xout;
    xout = xout';
    name = append('C:\Users\s202421\Documents/GitHub/MasterThesis\DataCreation\sto/',file_name );
    make_osimm(name, dofnames, xout(:,1:ndof), tout);
    fprintf('Simulation result for %s has been saved.\n ',file_name)
    
end



