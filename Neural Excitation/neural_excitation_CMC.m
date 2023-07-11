clear;
clc;

% Read STO data
path =  'C:/Users/s202421/Desktop/point1/';
file = 'point1_states.sto';
[data,C,tableData] =  readMOTSTOTRCfiles(path,file);

% Get time data and activation data
time = 1;
act = 24:2:299;

activations = C(:,act);
data_act = data(:,act);
data_time = data(:,time);

%% Prepare for Simulation
%Calculate time variables
total_time = data_time(end,:)-data_time(1,:);
[time_simple, data_simple] = clean_time_data(data_time, data_act);
[time_fill, data_fill] = fill_in_data(time_simple, data_simple);
data_fill = data_fill';
tstep = time_fill(2,:)-time_fill(1,:);
tend = total_time;
t = 0;
nsteps = round((tend-t)/tstep);

% Model Variables
% Some model related variables
modelparams = load('model_struct'); 
model = modelparams.model;
ndof = model.nDofs;
nmus = model.nMus;
nstates = 2*ndof + 2*nmus;
das3('Initialize',model);

% Define indices to the state variables within the state vector x
iq = 1:ndof;
iqdot = max(iq) + (1:ndof);
iLce = max(iqdot) + (1:nmus);

% Set equilibrium state as initial state
load('equilibrium.mat');
lengths = das3('Musclelengths',x);
LCEopt = das3('LCEopt');
SEEslack = das3('SEEslack');
x(iLce) = (lengths - SEEslack)./LCEopt;

% Initialize variables
step_u = zeros(nmus,1);
step_xdot = zeros(nstates,1); 
M = zeros(5,1);
exF = zeros(2,1);
handF = zeros(3,1);
% Create space for variables
tout = tstep*(0:nsteps)';
xout = zeros(nsteps+1, nstates);
uout = zeros(nsteps+1, 138);
xout(1,:) = x';

for i=1:(nsteps)
    u = data_fill(:,i);
    
    % Advance simulation by a step
    [x, step_xdot, step_u,~,qTH] = das3step(x, u, tstep, step_xdot, step_u,M,exF,handF);
    mf = das3('Muscleforces',x);
    xout(i+1,:) = x';   % store result
    uout(i+1,:) = u';
    mfout(i+1,:) = mf';
    qTHout(i+1,:)=qTH;

    t = t + tstep;
end

figure();
plot_wrist_positions(xout,model);