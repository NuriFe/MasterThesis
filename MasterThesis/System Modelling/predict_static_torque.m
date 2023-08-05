function [static_torque, R] = predict_static_torque(q)
    % load model filename
    if nargin ==1
        load('C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis\Data\model64.mat')
    end
    t = zeros(1,4);
    
    %covGP = {@covSEardNoDer};
    covGP = {@covSEard};

    covSIMPLE1 = {@cov1};
    covfunc1 = {'covSum',{covGP,covSIMPLE1}};
    t(1) = gp(modeldata.muscle(9).elevationplane.semiparametric.gpHyperparameters, @infExact, @mean1, covfunc1, @likGauss, modeldata.muscle(9).elevationplane.semiparametric.trainingInputs, modeldata.muscle(9).elevationplane.semiparametric.trainingOutputs, q);
    
    covSIMPLE2 = {@cov2};
    covfunc2 = {'covSum',{covGP,covSIMPLE2}};
    t(2) = gp(modeldata.muscle(9).shoulderelevation.semiparametric.gpHyperparameters, @infExact, @mean2, covfunc2, @likGauss, modeldata.muscle(9).shoulderelevation.semiparametric.trainingInputs, modeldata.muscle(9).shoulderelevation.semiparametric.trainingOutputs, q);
    
    covSIMPLE3 = {@cov3};
    covfunc3 = {'covSum',{covGP,covSIMPLE3}};
    t(3) = gp(modeldata.muscle(9).shoulderrotation.semiparametric.gpHyperparameters, @infExact, @mean3, covfunc3, @likGauss, modeldata.muscle(9).shoulderrotation.semiparametric.trainingInputs, modeldata.muscle(9).shoulderrotation.semiparametric.trainingOutputs, q);
    
    covSIMPLE4 = {@cov4};
    covfunc4 = {'covSum',{covGP,covSIMPLE4}}; 
    t(4) = gp(modeldata.muscle(9).elbowflexion.semiparametric.gpHyperparameters, @infExact, @mean4, covfunc4, @likGauss, modeldata.muscle(9).elbowflexion.semiparametric.trainingInputs, modeldata.muscle(9).elbowflexion.semiparametric.trainingOutputs, q);
    
    %covSIMPLE5 = {@cov5};
    %covfunc5 = {'covSum',{covGP,covSIMPLE5}}; 
    %t(5) = gp(modeldata.muscle(9).elbowpronation.semiparametric.gpHyperparameters, @infExact, @mean5, covfunc5, @likGauss, modeldata.muscle(9).elbowpronation.semiparametric.trainingInputs, modeldata.muscle(9).elbowpronation.semiparametric.trainingOutputs, q);


    static_torque = t;
end