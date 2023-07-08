function kinematic_jacobian(x)
    J = zeros(3,11);   % initialize the Jacobian for the xyz hand position as a function of 11 generalized coordinates
    
    q = x(1:11,:);                % the arm pose (11 generalized coordinates) at which you want to know the Jacobian
    
    x = [q ; zeros(287,1)];   % add 287 zeros to get a full system state vector, forward kinematics only depends on x1..x11
    
    h = 1e-7;   % optimal choice for finite difference, square root of the machine precision
    
    p = das3mex('Stick',x);
    
    p = p(:,end);  % 3d position of hand at pose q
    
    for i = 1:11
    
      xsave = x(i);
    
      x(i) = x(i) + h;
    
      ph = das3mex('Stick',x);
    
      ph = ph(:,end);
    
      J(:,i) = (ph-p)/h;  % this is column i of the Jacobian
    
      x(i) = xsave;
    
    end
end