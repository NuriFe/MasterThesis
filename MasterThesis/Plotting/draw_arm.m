function draw_arm(x,model)
    % define the points used to draw each bone as stick or polygon, and their colors
	black = [0 0 0];
	red = [0.5 0 0];
	green = [0 0.5 0];
	blue = [0 0 0.5];
	yellow = [.75 .75 0];
	magenta = [.75 0 .75];
	cyan = [0 .75 .75];
    
    bonepoints = cell(model.nJoints,2);
    for iJnt = 2:model.nJoints
        bonepoints{iJnt-1,1} = [0,0,0;model.joints{iJnt}.location];
        bonepoints{iJnt-1,2} = black;        
    end
    
    bonepoints{model.nJoints,1} = [0,0,0;model.joints{iJnt-1}.location]; % estimated endpoint hand coordinates
    bonepoints{model.nJoints,2} = black;        
        
    bonepoints{1,2} = yellow;       % IJ-SC
    bonepoints{4,2} = green;        % SC-AC
    bonepoints{7,2} = magenta;      % AC-GH
    bonepoints{10,2} = red;         % GH-UL
    bonepoints{11,2} = blue;        % UL-RD
    bonepoints{12,2} = cyan;        % RD-HD
    
    for i=1:size(x,2)
		
		% get the 3D bone position and orientation 
		d = das3('Visualization',x);
		if size(d,1) ~= size(bonepoints,1)
			error('das3stick: bonepoints structure is not consistent with number of bones');
        end

        % draw each bone
		hold on
		for j = 1:size(d,1)
			p = d(j,1:3)';					% position vector of bone
			R = reshape(d(j,4:12),3,3)';	% orientation matrix
			
			% create n x 3 matrix of the bone points local coordinates
			plocal = bonepoints{j,1}';
			npoints = size(plocal,2);
			
			% transform to global coordinates
			pglobal = repmat(p,1,npoints) + R*plocal;
			
			% plot points
			color = bonepoints{j,2};
			xx = pglobal(1,:);
			yy = pglobal(2,:);
			zz = pglobal(3,:);

			plot3(zz,xx,yy,'Color',color,'LineWidth',2)
        end
    end
end
