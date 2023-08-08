function [model, nstates, ndof, nmus, iLce] = initialize_model()
% Initialize the model
load model_struct
model.dofs{1}.range=[-0.419 -0.419];
model.dofs{2}.range=[0.0874 0.0874];
model.dofs{3}.range=[12 12]*pi/180;
model.dofs{4}.range=[34 34]*pi/180;
model.dofs{5}.range=[-20 -20]*pi/180;
model.dofs{6}.range=[-16 -16]*pi/180;
das3('Initialize',model);
%disp('Done.');

% Initialize the model
ndof=11;
nmus=138;
nstates=2*ndof+2*nmus;

% indices to the state variables within the state vector
iq = 1:ndof;
iqdot = max(iq) + (1:ndof);
iLce = max(iqdot) + (1:nmus);
iAct = max(iLce) + (1:nmus);

end