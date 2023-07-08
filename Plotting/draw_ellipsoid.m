function draw_ellipsoid(model)
    load('equilibrium.mat');
    d = das3('Visualization',x);

    % radii of thorax ellipsoid 
	Ax  = model.thorax_radii(1);
	Ay  = model.thorax_radii(2);
	Az  = model.thorax_radii(3);
    
    % center of thorax ellipsoid
	Mx  = model.thorax_center(1);
	My  = model.thorax_center(2);
	Mz  = model.thorax_center(3);

	R = reshape(d(1,4:12),3,3)';	% orientation matrix
    [xx,yy,zz] = ellipsoid(Mx,My,Mz,Ax,Ay,Az,R);
    [row,col,~] = find(zz>0);
    newx = xx(row,col);
    newy = yy(row,col);
    newz = zz(row,col);
    mesh(newz,newx,newy);
end
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