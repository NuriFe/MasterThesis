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
tend = 3;
t = 0;
nsteps = round((tend-t)/tstep);
%% Create space for variables

muscle_name= 'brachialis';
muscles = 109:1:115;
filepath = 'C:\Users\s202421\Documents/GitHub/MasterThesis\DataCreation\points/point14.mat';

data = load(filepath);

xout = data.xout;
x = xout(:,end);
handF_total = data.handF_total;
handF = handF_total(end,:)';

% Input Function
u_total = stim_fun(muscles, tstep, tend);
[fs,ers,xouts,qTHs] = simulate_force_PID(nmus,nstates,x, handF, u_total,tstep,nsteps);
keyboard
name = append('C:\Users\s202421\Documents/Github/MasterThesis\DataCreation\forces/',muscle_name,string(i) );
save(name,'fs','ers','xouts','qTHs');