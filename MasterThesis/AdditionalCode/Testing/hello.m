clear 
clc
close all
load('equilibrium.mat');
[model, nstates, ndof, nmus, iLce] = initialize_model();
%q = [x(1:5); 0.0799; 1.2470 ; 1.46 ; 1.7927; x(10:11)];
%q = [x(1:5); 0.0799; 1.2470 ; 1.46 ;-2.91; 1.34;x(11)];
q = [x(1:5); 0.0799; -0.436 ; 1.46 ;-2.91; 1.34;x(11)];

x = initialize_state(q,nstates,iLce);
[~, ~, ~, ~, ~, ~, qTH] = das3('Dynamics',x, zeros(size(x)), zeros(138,1));

% radii of thorax ellipsoid 
Ax  = model.thorax_radii(1);
Ay  = model.thorax_radii(2);
Az  = model.thorax_radii(3);

% center of thorax ellipsoid
Mx  = model.thorax_center(1);
My  = model.thorax_center(2);
Mz  = model.thorax_center(3);

% set plotting volume 
xrange = [-0.3 0.6];
yrange = [-1 0.5];
zrange = [-0.35 0.35];

% get the 3D bone position and orientation 
d = das3('Visualization',x);

R = reshape(d(1,4:12),3,3)';	% orientation matrix
[xx,yy,zz] = ellipsoid(Mx,My,Mz,Ax,Ay,Az,R);
[row,col,~] = find(zz>0);
newx = xx(row,col);
newy = yy(row,col);
newz = zz(row,col);
%mesh(newz,newx,newy);

bonepoints = cell(model.nJoints,2);
for iJnt = 2:model.nJoints
    bonepoints{iJnt-1,1} = [0,0,0;model.joints{iJnt}.location];
end

bonepoints{model.nJoints,1} = [0,0,0;model.joints{iJnt-1}.location]; % estimated endpoint hand coordinates

pglobal= [];
for j = [2,7,11,13]
	    p = d(j,1:3)';					% position vector of bone
	    R = reshape(d(j,4:12),3,3)';	% orientation matrix
	    
	    % create n x 3 matrix of the bone points local coordinates
	    plocal = bonepoints{j,1}';
	    npoints = size(plocal,2);
	    p = (repmat(p,1,npoints) + R*plocal);
	    % transform to global coordinates
	    pglobal = [pglobal p(:,1) ];

end

% get the 3D bone position and orientation 
T = pglobal(:,1);
A = pglobal(:,2);
B = pglobal(:,3);
H = pglobal(:,4);
color = 'blue';

% Create a 3D plot
figure()
das3stick(x,model);
hold on
plot3(T(3), T(1), T(2), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
hold on
plot3(A(3), A(1), A(2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot3(B(3), B(1), B(2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
plot3(H(3), H(1), H(2), 'yo', 'MarkerSize', 8, 'MarkerFaceColor', 'y');

dz = norm(B-A);
% Draw projections
line([A(3), A(3)], [A(1), A(1)], [A(2), A(2)+dz], 'Color', 'g', 'LineWidth', 2);
O = [A(3), A(1), A(2)];
P1 = [A(3), A(1), A(2)+dz];
P2 =[B(3) B(1) B(2)];
u = O-P1 ; 
v = O-P2 ;  
EA = atan2(norm(cross(u, v)), dot(u, v));

line([A(3), A(3)], [A(1), A(1)+dz], [A(2), A(2)], 'Color', 'm', 'LineWidth', 2);
O = [A(3), A(1), A(2)];
P1 = [A(3), A(1)+dz, A(2)];
P2 =[B(3) B(1) B(2)];
u = O-P1 ; 
v = O-P2 ;  
EP = atan2(norm(cross(u, v)), dot(u, v));

%%%
dz = norm(H-B);
line([B(3), B(3)], [B(1), B(1)-dz], [B(2), B(2)], 'Color', 'r', 'LineWidth', 2);
O = [B(3), B(1), B(2)];
P1 = [B(3), B(1)-dz, B(2)];
P2 =[H(3) H(1) H(2)];
u = O-P1 ; 
v = O-P2 ;  
SR = atan2(norm(cross(u, v)), dot(u, v));

(qTH*180/pi)'
[EP,EA,SR]*180/pi


%line([B(3), B(3)], [A(1), B(1)], [A(2), B(2)], 'Color', 'b', 'LineWidth', 2);
%line([A(3), B(3)], [B(1), B(1)], [A(2), B(2)], 'Color', 'b', 'LineWidth', 1);

% Draw the projected line from A to the origin (0, 0, 0)
scaling_factor = 200;
% Calculate the direction vector of the line passing through A and B
direction_vector = B - A;

% Extend the line in both directions
extended_A = A - scaling_factor * direction_vector;
extended_B = B + scaling_factor * direction_vector;
plot3([extended_A(3), extended_B(3)], ...
      [extended_A(1), extended_B(1)], ...
      [extended_A(2), extended_B(2)], ...
      'c', 'LineWidth', 1);

% % Draw the projected line from B to the origin (0, 0, 0)
% %plot3([0, xB], [0, yB], [0, zB], 'g--');
% % Define the extents of the planes
% x_extent = 10;
% y_extent = 10;
% z_extent = 10;
% 
% % Create a grid of points for the planes
% [X, Y] = meshgrid(-x_extent:x_extent, -y_extent:y_extent);
% Z = zeros(size(X));
% 
% % Plot the XY plane
% surf(X, Y, Z, 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'FaceColor', 'r');
% hold on;
% 
% % Plot the YZ plane
% surf(Z, X, Y, 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'FaceColor', 'g');
% 
% % Plot the XZ plane
% surf(X, Z, Y, 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'FaceColor', 'b');

% set plotting volume 
xrange = [-0.3 0.6];
yrange = [-1 0.5];
zrange = [-0.45 0.35];
view(-120,50);
axis([zrange xrange yrange]);
xlabel('Z');
ylabel('X');
zlabel('Y');
grid on;
hold off

% subplot(1,3,2);
% plot3(T(3), T(1), T(2), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
% hold on
% plot3(A(3), A(1), A(2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
% plot3(B(3), B(1), B(2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
% % Connect the points A and B with a line
% line([A(3), B(3)], [A(1), B(1)], [A(2), B(2)], 'Color', 'b', 'LineWidth', 2);
% plot3([extended_A(3), extended_B(3)], ...
%       [extended_A(1), extended_B(1)], ...
%       [extended_A(2), extended_B(2)], ...
%       'c', 'LineWidth', 1)
% 
% % set plotting volume 
% xrange = [-0.3 0.6];
% yrange = [-1 0.5];
% zrange = [-0.35 0.35];
% axis('equal');
% view(-90,90);
% axis([zrange xrange yrange]);
% xlabel('Z');
% ylabel('X');
% zlabel('Y');
% grid on;
% hold off
% 
% subplot(1,3,3);
% plot3(T(3), T(1), T(2), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
% hold on
% plot3(A(3), A(1), A(2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
% plot3(B(3), B(1), B(2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
% % Connect the points A and B with a line
% line([A(3), B(3)], [A(1), B(1)], [A(2), B(2)], 'Color', 'b', 'LineWidth', 2);
% plot3([extended_A(3), extended_B(3)], ...
%       [extended_A(1), extended_B(1)], ...
%       [extended_A(2), extended_B(2)], ...
%       'c', 'LineWidth', 1)
% % set plotting volume 
% xrange = [-0.3 0.6];
% yrange = [-1 0.5];
% zrange = [-0.35 0.35];
% axis('equal');
% view(0,0);
% axis([zrange xrange yrange]);
% xlabel('Z');
% ylabel('X');
% zlabel('Y');
% grid on;
% hold off
% 
% % Define the two 2D points
% y = [A(2), B(2)];
% z = [A(3), B(3)];
% x = [A(1), B(1)];
% dx = abs(A(1)-B(2));
% dy = abs(A(2)-B(2));
% dz = abs(A(3)-B(3));
% theta1 = atan(dy/dz);%*180/pi;
% theta2 = atan(dz/dy);
% 
% 
% r = 0.01;
% %yy = r*sin(th); 
% %zz = r*cos(th);
% %xx = r*ones(1,length(yy));
% 
% %plot3(z,x,[0 0])
% %plot(yy,zz)
% 
% % theta = atan(dy/dx);%*180/pi;
% % if theta < 0
% %     th = (pi/2-theta):1/360:(pi);;
% % else
% %     th = pi/2:1/360:theta;
% % end
% % 
% % xx = r*sin(th)+ B(2); 
% % yy = r*cos(th) +B(3);
% % plot(xx,yy)
% % axis('equal')
% 
% %axis([0 0.24 -0.3 0])
% hold off


function [xx,yy,zz]=ellipsoid(varargin)
%ELLIPSOID Generate ellipsoid.
%   [X,Y,Z]=ELLIPSOID(XC,YC,ZC,XR,YR,ZR,R,N) generates three
%   (N+1)-by-(N+1) matrices so that SURF(X,Y,Z) produces an
%   ellipsoid with center (XC,YC,ZC) and radii XR, YR, ZR, rotated by R.
% 
%   [X,Y,Z]=ELLIPSOID(XC,YC,ZC,XR,YR,ZR,R) uses N = 20.
%   [X,Y,Z]=ELLIPSOID(XC,YC,ZC,XR,YR,ZR) uses R = eye(3).

%   ELLIPSOID(...) and ELLIPSOID(...,N) with no output arguments
%   graph the ellipsoid as a SURFACE and do not return anything.
%
%   ELLIPSOID(AX,...) plots into AX instead of GCA.
%
%   The ellipsoidal data is generated using the equation:
%
%    (X-XC)^2     (Y-YC)^2     (Z-ZC)^2
%    --------  +  --------  +  --------  =  1
%      XR^2         YR^2         ZR^2
%
%   See also SPHERE, CYLINDER.

%   Dimitra Blana Nov 2014 based on ellipsoid.m 
%   by Laurens Schalekamp and Damian T. Packer
%   Copyright 1984-2002 The MathWorks, Inc. 

% Parse possible Axes input
narginchk(6,8);
[cax,args,nargs] = axescheck(varargin{:});

[xc,yc,zc,xr,yr,zr] = deal(args{1:6});
n  = 20;
R = eye(3);
if nargs > 6
	R = args{7}; 
end
if nargs > 7
	n = args{8}; 
end

[x,y,z] = sphere(n);

x = xr*x+xc;
y = yr*y+yc;
z = zr*z+zc;

for i=1:size(x,1)
    for j=1:size(x,2)
        rot_xyz = R*[x(i,j);y(i,j);z(i,j)];
        x(i,j) = rot_xyz(1);
        y(i,j) = rot_xyz(2);
        z(i,j) = rot_xyz(3);
    end
end

if(nargout == 0)
    cax = newplot(cax);
	surf(x,y,z,'parent',cax)
else
	xx=x;
	yy=y;
	zz=z;
end

end
