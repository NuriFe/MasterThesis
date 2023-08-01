load('C:\Users\s202421\Documents\GitHub\MasterThesis\DataCreation\excitations1.mat')

max_activation = zeros(3,1);
for i= 1:3
    indx = whichMuscle(i)+160;
    activation = xout(:,indx);
    max_activation(i) = max(max(activation));
end