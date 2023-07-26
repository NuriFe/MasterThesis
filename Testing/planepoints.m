%close all
clear
clc
close all

load('C:\Users\s202421\Documents\GitHub\MasterThesis\PathFollowingControl\feasiblepoints.mat')
points = wristFeasible;
[model, nstates, ndof, nmus, iLce] = initialize_model();

% Target point
% Draw the box using the "rectangle" function in 3D
P = [-0.17,0.085,-0.075] ;   % you center point 
L = [0.02,0.23,0.15] ;  % your cube dimensions 
O = P-L/2 ;       % Get the origin of cube so that P is at center 
           
% Transformed point
min_corner = [-0.03, -0.15, -0.18];
max_corner = [0.2, 0, -0.16];

% Find the points that lie inside the defined rectangle
is_inside_cube = all(points >= min_corner & points <= max_corner, 2);
indices_inside_cube = find(is_inside_cube);

% Extract the points that are inside the rectangle
points_inside_cube = points(is_inside_cube, :);


figure()
plot_wrist_references(points,model,[]);
hold on;
plotcube(L,O,.5,[0 1 0]);   % use function plotcube 

plot_wrist_references(points_inside_cube,model,[],'r.');


hold off;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
view(-90,90);
linspace = 1:20:length(indices_inside_cube);
to_try = indices_inside_cube(linspace,:)';