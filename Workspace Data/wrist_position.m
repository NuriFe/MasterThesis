function pos = wrist_position(x)
    d = das3('Visualization',x);
    j = length(d);
    Mx = 0;
    My = -0.1486;
    Mz = 0.0591;
    
    p = d(j,1:3)';					% position vector of bone
	R = reshape(d(j,4:12),3,3)';	% orientation matrix
	
	%create n x 3 matrix of the bone points local coordinates
	plocal =  [0,0,0; 0.0171, -0.0133, -0.0163]';
	
    npoints = 2;
	%transform to global coordinates
	pglobal = repmat(p,1,npoints) + R*plocal;
    x = pglobal(1,1);
    y = pglobal(2,1);
    z = pglobal(3,1);

    pos = [x y z]';



end
