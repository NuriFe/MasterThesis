clear;
close all;
clc;
visitedWristPose = create_grid(0.1);

%% create a grid of positions inside the convex hull of visited wrist positions
visitedWristPositions=visitedWristPose(:,1:3);
minPositions = min(visitedWristPositions);
maxPositions = max(visitedWristPositions);
spacing = 0.05; % grid spacing in mm
xCoords = minPositions(1):spacing:maxPositions(1);
yCoords = minPositions(2):spacing:maxPositions(2);
zCoords = minPositions(3):spacing:maxPositions(3);
[X,Y,Z] = meshgrid(xCoords,yCoords,zCoords);
X = reshape(X,length(xCoords)*length(yCoords)*length(zCoords),1);
Y = reshape(Y,length(xCoords)*length(yCoords)*length(zCoords),1);
Z = reshape(Z,length(xCoords)*length(yCoords)*length(zCoords),1);
inhullIndices = find(inhull([X Y Z],visitedWristPositions));
testWristPositions = [X(inhullIndices) Y(inhullIndices) Z(inhullIndices)];
plot3(testWristPositions(:,1),testWristPositions(:,2),-testWristPositions(:,3),'o')
hold on
plot3(visitedWristPositions(:,1),visitedWristPositions(:,2),-visitedWristPositions(:,3),'x')
axis image