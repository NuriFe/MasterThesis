clear
clc
close all

load model_struct

p = zeros(13,3);
r11 = zeros(13,3);
r12 = zeros(13,3);
r13 = zeros(13,3);
scalingfactor = 0.15;
for i=1:13
    p(i,:)=model.joints{1,i}.location;
    try
        r11(i,:)=model.joints{1, i}.r1_axis*scalingfactor;
        r12(i,:)=model.joints{1, i}.r2_axis*scalingfactor;
        r13(i,:)=model.joints{1, i}.r3_axis*scalingfactor;
    catch exception
        msgText = getReport(exception,'basic');
        string = 'Unrecognized field name "r1_axis".';
        if all(msgText == string) && ((i==1)|| (i==13))
            r11(i,:)=[0 1 0]*scalingfactor;
            r12(i,:)=[0 0 1]*scalingfactor;
            r13(i,:)=[1 0 0]*scalingfactor;
        end
        
    end

end
modelparams = load('model_struct'); 
model = modelparams.model;
das3('Initialize',model);


options.box = 1;

options.axes = 1;

options.keepview = 1;

    
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

% Initialize the global coordinate system
global_points = zeros(size(p));
load('equilibrium.mat')
d = das3('Visualization',x);
figure()
subplot(1,2,1)
das3stick(x,model);

bonepoints = cell(model.nJoints,2);
for iJnt = 2:model.nJoints
    bonepoints{iJnt-1,1} = [0,0,0;model.joints{iJnt}.location];
end

bonepoints{model.nJoints,1} = [0,0,0;model.joints{iJnt-1}.location];

% Perform cumulative translations to transform each point to global coordinates
for i = 1:size(d, 1)
    p = d(i,1:3)';					% position vector of bone
    R = reshape(d(i,4:12),3,3)';
    plocal = bonepoints{i,1}';
    npoints = size(plocal,2);
	pglobal = repmat(p,1,npoints) + R*plocal;
    P = pglobal(:,2);
    if i > 1
        global_points(i, :) = P;
    else
        global_points(i, :) = p(i, :);
    end
end

% Plot the global points in 3D
subplot(1,2,2)
[xx,yy,zz] = ellipsoid(Mx,My,Mz,Ax,Ay,Az);
[row,col,~] = find(zz>0);
newx = xx(row,col);
newy = yy(row,col);
newz = zz(row,col);
mesh(newz,newx,newy);
grid on;
axis('equal');
axis([zrange xrange yrange]);
xlabel('Z');
ylabel('X');
zlabel('Y');
view(290, 15);


% draw each point
hold on
plot3(global_points(:, 3), global_points(:, 1), global_points(:, 2), 'o-', 'LineWidth', 2, 'MarkerSize',2);
h = zeros(3,1);
for i=1:13
    plot3(global_points(i, 3), global_points(i, 1), global_points(i, 2), 'bo-', 'LineWidth', 2, 'MarkerSize', 1);

    h(1) = quiver3(global_points(i, 3), global_points(i, 1), global_points(i, 2), r11(i, 1), r11(i, 2), r11(i, 3), 'c','LineWidth', 1);
    h(2) = quiver3(global_points(i, 3), global_points(i, 1), global_points(i, 2), r12(i, 1), r12(i, 2), r12(i, 3), 'r','LineWidth', 1);
    h(3) = quiver3(global_points(i, 3), global_points(i, 1), global_points(i, 2), r13(i, 1), r13(i, 2), r13(i, 3), 'g','LineWidth', 1);

end
legend(h,{'x axis','y axis','z axis'})
