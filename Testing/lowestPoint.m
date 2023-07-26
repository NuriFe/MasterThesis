%close all
clear
clc

load('C:\Users\s202421\Documents\GitHub\MasterThesis\PathFollowingControl\feasiblepoints.mat')
points = wristFeasible;
[model, nstates, ndof, nmus, iLce] = initialize_model();

% Target point
targetPoint = [0.5, -1, 0];

% Calculate the Euclidean distances from each point to the target point
distances = sqrt(sum((points - targetPoint).^2, 2));

% Find the index of the point with the smallest distance
[minDist, minDistIdx] = min(distances);

% Extract the point with the smallest distance
closestPoint = points(minDistIdx, :);

% Display the closest point and its index
disp("Closest Point:");
disp(closestPoint);
disp("Index in Array:");
disp(minDistIdx);

endWristPos = wristFeasible(minDistIdx,:);
figure()
plot_wrist_references(wristFeasible,model,[]);
hold on;

plot_wrist_references(endWristPos,model,[],'r.');

hold off;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');