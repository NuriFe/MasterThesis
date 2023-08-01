load('equilibrium.mat')
modelparams = load('model_struct'); 
model = modelparams.model;
% Initialize the model
das3('Initialize',model);

moments = das3('Jointmoments',x);

sum(moments) %Should be 0 if its a state of equilibrium, shouldn't be?