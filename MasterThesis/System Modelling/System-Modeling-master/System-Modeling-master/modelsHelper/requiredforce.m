%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% REQUIREDforce.M
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Descritpion: This function computes the torque required to hold a desired
% arm configuration.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eric Schearer
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Created: 18 Feb 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Updated: 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function torqueMag = requiredforce(q)

global modeldata

t = zeros(1,3);


meanfunc=[];
likfunc = {@likGauss}; 
%covfunc={@covSE_rigid_multHyp};
%covfunc={@covSE_rigid};
covfunc={@covSEard};


t(:,1) = gp(modeldata.muscle(4).Fx.hyp, @infExact, meanfunc, covfunc, likfunc, modeldata.muscle(4).Fx.trainingInputs(1:end,:), modeldata.muscle(4).Fx.trainingOutputs(1:end), q);
t(:,2) = gp(modeldata.muscle(4).Fy.hyp, @infExact, meanfunc, covfunc, likfunc, modeldata.muscle(4).Fy.trainingInputs(1:end,:), modeldata.muscle(4).Fy.trainingOutputs(1:end), q);
t(:,3) = gp(modeldata.muscle(4).Fz.hyp, @infExact, meanfunc, covfunc, likfunc, modeldata.muscle(4).Fz.trainingInputs(1:end,:), modeldata.muscle(4).Fz.trainingOutputs(1:end), q);


torqueMag = norm(t);