function w_refs = create_grid(distance,model)
% Define the starting coordinates
start_x = 0.05;
start_y = -0.2;
start_z = -0.3;

% Generate x, y, and z coordinates
x = start_x + distance * (0:2);
y = start_y + distance * (0:2);
z = start_z + distance * (0:2);

% Create a grid of coordinates
[X, Y, Z] = meshgrid(x, y, z);
% [z,x,y]

% Reshape the coordinate grids into column vectors
x_coords = X(:);
y_coords = Y(:);
z_coords = Z(:);

w_refs = [x_coords y_coords z_coords];

if nargin>1

    plot_wrist_references(w_refs,model)


end

