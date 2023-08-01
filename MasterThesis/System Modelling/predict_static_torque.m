function [static_torque, R] = predict_static_torque(q)
    % load model filename
    load('C:\Users\s202421\Documents\GitHub\MasterThesis\Data\model\model64.mat')
    t = zeros(1,4);
    
    %covGP = {@covSEardNoDer};
    covGP = {@covSEard};

    covSIMPLE1 = {@cov1};
    covfunc1 = {'covSum',{covGP,covSIMPLE1}};
    t(1) = gp(modeldata.muscle(4).elevationplane.semiparametric.gpHyperparameters, @infExact, @mean1, covfunc1, @likGauss, modeldata.muscle(4).elevationplane.semiparametric.trainingInputs, modeldata.muscle(4).elevationplane.semiparametric.trainingOutputs, q);
    
    covSIMPLE2 = {@cov2};
    covfunc2 = {'covSum',{covGP,covSIMPLE2}};
    t(2) = gp(modeldata.muscle(4).shoulderelevation.semiparametric.gpHyperparameters, @infExact, @mean2, covfunc2, @likGauss, modeldata.muscle(4).shoulderelevation.semiparametric.trainingInputs, modeldata.muscle(4).shoulderelevation.semiparametric.trainingOutputs, q);
    
    covSIMPLE3 = {@cov3};
    covfunc3 = {'covSum',{covGP,covSIMPLE3}};
    t(3) = gp(modeldata.muscle(4).shoulderrotation.semiparametric.gpHyperparameters, @infExact, @mean3, covfunc3, @likGauss, modeldata.muscle(4).shoulderrotation.semiparametric.trainingInputs, modeldata.muscle(4).shoulderrotation.semiparametric.trainingOutputs, q);
    
    covSIMPLE4 = {@cov4};
    covfunc4 = {'covSum',{covGP,covSIMPLE4}}; 
    t(4) = gp(modeldata.muscle(4).elbowflexion.semiparametric.gpHyperparameters, @infExact, @mean4, covfunc4, @likGauss, modeldata.muscle(4).elbowflexion.semiparametric.trainingInputs, modeldata.muscle(4).elbowflexion.semiparametric.trainingOutputs, q);
    

    static_torque = t;
end