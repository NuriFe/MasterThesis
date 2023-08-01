function w_refs = create_grid(distance,model)
% Define the starting coordinates
start_y = 0;
start_z = -0.1;
start_x = 0.3;
end_y = -0.3;
end_z = -0.35;
end_x = -0.08;

% Generate x, y, and z coordinates
%x = start_x + distance * (0:2);
%y = start_y + distance * (0:1);
%z = start_z + distance * (0:2);
x = linspace(start_x, end_x, 4);
y = linspace(start_y, end_y, 4);
z = linspace(start_z, end_z, 4);

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

