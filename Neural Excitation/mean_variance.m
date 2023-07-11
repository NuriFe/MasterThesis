%I have a matrix with 10 rows and 3 columns. Each column is a variable. 
% I want to calculate and plot the mean and standard deviation 
% of each variable for the 10 data that I have of each.
load('C:\Users\s202421\Documents\GitHub\MasterThesis\DataCreation\modelpoints\neuralexcitations\1.mat')

%activation = act_fill(:,end-100:end)';
activation = act_fill';
data_mean = mean(activation);
data_std = std(activation);

figure()
errorbar(data_mean,data_std,"o")
