%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMPUTEMODELS.M
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Descritpion: This script loads the 'parseddata.mat' file and does GPR for
% each muscle where the inputs are joint angles and the outputs are joint
% torques.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Eric Schearer
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Created: 15 February 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Updated:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all

%load('sampleData')
load('C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis\Data\Torques/data64.mat')
sampleData = data_for_model;
% prior parameters and covariance
params1 = zeros(1,2);
params2 = zeros(1,2);
params3 = zeros(1,2);
params4 = zeros(1,2);
covariance1 = diag(10000*ones(length(params1),1));
covariance2 = diag(10000*ones(length(params2),1));
covariance3 = diag(10000*ones(length(params3),1));
covariance4 = diag(10000*ones(length(params4),1));

muscles = 9;
targets = 64;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% build data structure to save model data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
modeldata.muscle(1).label  = 'Radial Nerve - Triceps';
modeldata.muscle(2).label  = 'Thoracodorsal Nerve - Latissimus Dorsi';
modeldata.muscle(3).label  = 'Long Thoracic - Serratus Anterior';
modeldata.muscle(4).label  = 'Musculocutaneous Nerve - Biceps/Bracialis';
modeldata.muscle(5).label  = 'Suprascapular Nerve - Supraspinatus/Infraspinatus';
modeldata.muscle(6).label  = 'Rhomboids';
modeldata.muscle(7).label  = 'Lower Pectoralis';
modeldata.muscle(8).label  = 'Upper Pectoralis';
modeldata.muscle(9).label = 'Passive';

for i = 1:muscles
    i
%%%%%% CHANGED TO USE SAMPLE DATA
    angleData   = [];
    torqueData  = [];
    angleData = sampleData(i).angleData;
    torqueData=sampleData(i).torqueData;
    
%%%%% REMOVED FOR DATA
%     for j = 1:targets
%         if size(stimdata.muscle(i).target(j).allJointTorques,1) > 0
%             torqueData  = [torqueData;stimdata.muscle(i).target(j).allJointTorques];
%             angleData   = [angleData;ones(size(stimdata.muscle(i).target(j).allJointTorques,1),1)*[stimdata.muscle(i).target(j).meanFilteredJointAngles(1:4) stimdata.muscle(i).target(j).meanFilteredJointAngles(6)]];
%         end
%     end
    muscle = num2str(i);
    modeldata.muscle(i).elevationplane.parametric           = parametricfunctionReach(muscle,'ElevationPlane',   angleData,torqueData(:,1),angleData,torqueData(:,1),covariance1,params1);
    modeldata.muscle(i).elevationplane.semiparametric       = semiparametricfunctionReach(muscle,'ElevationPlane',   angleData(1:2:length(angleData),:),torqueData(1:2:length(angleData),1),angleData,torqueData(:,1),modeldata.muscle(i).elevationplane.parametric.parameterCov,   modeldata.muscle(i).elevationplane.parametric.parameters);
    %
    display('a');
    modeldata.muscle(i).shoulderelevation.parametric        = parametricfunctionReach(muscle,'ShoulderElevation',angleData,torqueData(:,2),angleData,torqueData(:,2),covariance1,params1);
    modeldata.muscle(i).shoulderelevation.semiparametric    = semiparametricfunctionReach(muscle,'ShoulderElevation',angleData(1:2:length(angleData),:),torqueData(1:2:length(angleData),2),angleData,torqueData(:,2),modeldata.muscle(i).shoulderelevation.parametric.parameterCov,modeldata.muscle(i).shoulderelevation.parametric.parameters);
    %
    display('b');
    modeldata.muscle(i).shoulderrotation.parametric         = parametricfunctionReach(muscle,'ShoulderRotation', angleData,torqueData(:,3),angleData,torqueData(:,3),covariance1,params1);
    modeldata.muscle(i).shoulderrotation.semiparametric     = semiparametricfunctionReach(muscle,'ShoulderRotation', angleData(1:2:length(angleData),:),torqueData(1:2:length(angleData),3),angleData,torqueData(:,3),modeldata.muscle(i).shoulderrotation.parametric.parameterCov, modeldata.muscle(i).shoulderrotation.parametric.parameters);
    %
    display('c');
    modeldata.muscle(i).elbowflexion.parametric             = parametricfunctionReach(muscle,'ElbowFlexion',     angleData,torqueData(:,4),angleData,torqueData(:,4),covariance1,params1);
    modeldata.muscle(i).elbowflexion.semiparametric         = semiparametricfunctionReach(muscle,'ElbowFlexion',     angleData(1:2:length(angleData),:),torqueData(1:2:length(angleData),4),angleData,torqueData(:,4),modeldata.muscle(i).elbowflexion.parametric.parameterCov,     modeldata.muscle(i).elbowflexion.parametric.parameters);
    %
    display('d');
    modeldata.muscle(i).elbowpronation.parametric           = parametricfunctionReach(muscle,'ElbowPronation',   angleData,torqueData(:,5),angleData,torqueData(:,5),covariance1,params1);
    modeldata.muscle(i).elbowpronation.semiparametric       = semiparametricfunctionReach(muscle,'ElbowPronation',   angleData(1:2:length(angleData),:),torqueData(1:2:length(angleData),5),angleData,torqueData(:,5),modeldata.muscle(i).elbowpronation.parametric.parameterCov,   modeldata.muscle(i).elbowpronation.parametric.parameters);
    %
end
        
save('model64.mat','modeldata')

