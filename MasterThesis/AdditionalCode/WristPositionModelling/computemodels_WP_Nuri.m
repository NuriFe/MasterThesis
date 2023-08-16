%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% computeModels_wrist.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
clear
clc

load('C:\Users\s202421\Documents\GitHub\MasterThesis\Data\torques\data64.mat')

params = [1 1 1 1]*log(10);

muscles = 4;
targets = 64;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% build data structure to save model data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
modeldata.muscle(1).label  = 'Radial Nerve - Triceps';
modeldata.muscle(2).label  = 'Axillary Nerve - Deltoids';
modeldata.muscle(3).label  = 'Musculocutaneous Nerve - Biceps/Bracialis';
modeldata.muscle(4).label = 'Passive';


for i = 1:muscles
    i
    wristData = data_for_model.wristposition;
    forceData= data_for_model.wristforces;

    testOutputs = forceData;
    testInputs = wristData;
    
    modeldata.muscle(i).Fx = TrainWristGP_position('Fx',i,wristData,forceData(:,1),params);
    modeldata.muscle(i).Fy = TrainWristGP_position('Fy',i,wristData,forceData(:,2),params);
    modeldata.muscle(i).Fz = TrainWristGP_position('Fz',i,wristData,forceData(:,3),params);
    
    
end
filename = 'model_WP';
save(['C:\Users\s202421\Documents\GitHub\MasterThesis\Data\model\',filename],'modeldata');
