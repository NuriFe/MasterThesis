%semiStruct.gpHyperparameters    = hypSIMPLET1;
%@infExact >> gp

% mean function
% 
clear
clc
close all
global modeldata

sampling = 1; % only use 1/5 of the data to fit models
muscles = 4;
% load model filename
load('C:\Users\s202421\Documents\GitHub\MasterThesis\Data\model\model64.mat')

load('C:\Users\s202421\Documents\GitHub\MasterThesis\Data\torques\data64.mat')

q= data_for_model(1).angleData(1,:);
t = zeros(1,4);

%covGP = {@covSEardNoDer};
covGP = {@covSEard};
torque = zeros(4,3);
static_torque = zeros(4,1);
for i = 1:muscles
    covSIMPLE1 = {@cov1};
    covfunc1 = {'covSum',{covGP,covSIMPLE1}};
    t(1) = gp(modeldata.muscle(i).elevationplane.semiparametric.gpHyperparameters, @infExact, @mean1, covfunc1, @likGauss, modeldata.muscle(i).elevationplane.semiparametric.trainingInputs, modeldata.muscle(i).elevationplane.semiparametric.trainingOutputs, q);
    
    covSIMPLE2 = {@cov2};
    covfunc2 = {'covSum',{covGP,covSIMPLE2}};
    t(2) = gp(modeldata.muscle(i).shoulderelevation.semiparametric.gpHyperparameters, @infExact, @mean2, covfunc2, @likGauss, modeldata.muscle(i).shoulderelevation.semiparametric.trainingInputs, modeldata.muscle(i).shoulderelevation.semiparametric.trainingOutputs, q);
    
    covSIMPLE3 = {@cov3};
    covfunc3 = {'covSum',{covGP,covSIMPLE3}};
    t(3) = gp(modeldata.muscle(i).shoulderrotation.semiparametric.gpHyperparameters, @infExact, @mean3, covfunc3, @likGauss, modeldata.muscle(i).shoulderrotation.semiparametric.trainingInputs, modeldata.muscle(i).shoulderrotation.semiparametric.trainingOutputs, q);
    
    covSIMPLE4 = {@cov4};
    covfunc4 = {'covSum',{covGP,covSIMPLE4}}; 
    t(4) = gp(modeldata.muscle(i).elbowflexion.semiparametric.gpHyperparameters, @infExact, @mean4, covfunc4, @likGauss, modeldata.muscle(i).elbowflexion.semiparametric.trainingInputs, modeldata.muscle(i).elbowflexion.semiparametric.trainingOutputs, q);
    
    torqueMag = norm(t);
    if i == 4
        static_torque = t';
    else
        torque(:,i)=t';
    end
end
alpha0=zeros(3,1);

R = static_torque - torque;
tauDes = static_torque;
alpha0=computeActivations(R,tauDes,alpha0);

tstep = .003;
tend = 1;

load('equilibrium.mat')

%% Initialise variables and model
% Some model related variables
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
t = 0;
nsteps = round((tend-t)/tstep);

%% Create space for variables
tout = tstep*(0:nsteps)';
xout = zeros(nsteps+1, nstates);
uout = zeros(nsteps+1, 138);
mfout = zeros(nsteps+1, 138);
qTHout = zeros(nsteps+1,3);

% Set equilibrium state as initial state
x(iLce)=1;
lengths = das3('Musclelengths',x);
LCEopt = das3('LCEopt');
SEEslack = das3('SEEslack');
x(iLce) = (lengths - SEEslack)./LCEopt;

xout(1,:) = x';

% Initialize variables
step_u = zeros(nmus,1);
step_xdot = zeros(nstates,1); 

%%
warning('error', 'MATLAB:nearlysingularMatrix');
M = zeros(5,1);
exF = [ 0 0]';
handF = [0 0 0]';
u=zeros(nmus,1);

for j=1:3
    mus=whichMuscle(j);
    u(mus)=alpha0(j)*1;
end

for i=1:(nsteps)
    try
        % Advance simulation by a step
        [x, step_xdot, step_u,~,qTH] = das3step(x, u, tstep, step_xdot, step_u, M, exF, handF);
        mf = das3('Muscleforces',x);
        xout(i+1,:) = x';   % store result
        uout(i+1,:) = u';
        mfout(i+1,:) = mf';
        qTHout(i+1,:)=qTH;
    
        t = t + tstep;
    catch exception
         if strcmp(exception.identifier, 'MATLAB:nearlySingularMatrix')
             fprintf('Singularity found. Simulation stopped after %.3f seconds \n', t);
             break
         end
    end


end

create_osim(ndof, model, '0',x, tout, xout, mfout, qTHout);
plot_wrist_positions(xout,model);


%==========================================================
function [] = create_osim(ndof, model, muscle_name, x, tout, xout, mfout, qTHout)
    % Find DOF names
    dofnames = cell(ndof,1);
    for idof=1:ndof
        dofnames{idof} = model.dofs{idof}.osim_name;
    end
    
    name = append('C:\Users\s202421\Documents/Github/MasterThesis\DataCreation\sto/', muscle_name);
    % Export to mat file and to opensim motion file
    save(name,'x','tout','xout', 'mfout', 'qTHout');
    make_osimm(name, dofnames, xout(:,1:ndof), tout);
    fprintf('Simulation result for %s has been saved.\n ',muscle_name)
end
