function [R] = predict_activation_torque(q,model)
    muscles = 8;
    % load model filename
    if nargin ==1
        load('C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis\Data\model64.mat')
    end
    covGP = {@covSEard};
    R = zeros(4,muscles);
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
        

        R(:,i)=t';
        
    end
    
    R;
end