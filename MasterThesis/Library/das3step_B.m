% das3step.m function
function [xnew, xdot, step_u, FGH] = das3step_B(x, u, h, xdot, step_u, M, exF, handF, K, B)
	% Ton van den Bogert
	% (c) 2010-2011 Case Western Reserve University
    %
	% For dynamic simulation.  Advances the model by one time step.
    %
	% Inputs
	%	x		(298 x 1) System state
	%	u		(138 x 1) Muscle excitations, assumed constant during the time step
	%	h		(scalar)  Time step size
    %   xdot    (298 x 1) System state derivatives (initially zero)
	%	step_u	(138 x 1) Muscle excitations from previous step (initially zero)
    % Optional inputs
    % 	M		(5 x 1)	  Moments applied to the thorax-humerus YZY axes and the elbow flexion and supination axes
	%   exF     (2 x 1)   Vertical force of amplitude exF(2) applied to the ulna at a distance of exF(1) from the elbow 
	%						(to simulate a mobile arm support)
    %   handF   (3 x 1)   Force at the CoM of the hand
    %
	% Outputs
	%	xnew	(298 x 1) The system state at the end of the time step
	%	FGH		(3 x 1)   The glenohumeral reaction force, acting on the scapula
    %
	% Method: First order Rosenbrock formula on implicit differential equation	
	
	% Evaluate dynamics in current x and xdot
    if nargin>7
        [f, dfdx, dfdxdot, dfdu, FGH] = das3h(x,xdot,step_u,M,exF,handF);
    elseif nargin>6
        [f, dfdx, dfdxdot, dfdu, FGH] = das3h(x,xdot,step_u,M,exF);
    elseif nargin>5
        [f, dfdx, dfdxdot, dfdu, FGH] = das3h(x,xdot,step_u,M);
    else
        [f, dfdx, dfdxdot, dfdu, FGH] = das3h(x,xdot,step_u);
    end
    
    % Update extra terms in the Jacobian
   %d2Phand_dx2 = das_hessian(x);
    %[dPhand_dx,~] = pos_jacobian(x);
    %[df_dhandF] = handforce_jacobian(x,xdot,step_u,M,exF,handF);
    %dfdx(:,1:11) = dfdx(:,1:11) - df_dhandF*(K*dPhand_dx - B * mult(d2Phand_dx2,x(12:22)));
    %dfdx(:,12:22) = dfdx(:,12:22) - df_dhandF*B*dPhand_dx;
    
	% Solve the change in x from the 1st order Rosenbrock formula
	du = u - step_u;
	dx = (dfdx + dfdxdot/h)\(dfdxdot*xdot - f - dfdu*du);
	xnew = x + dx;
	
	% update variables for the next simulation step
	xdot = dx/h;
	step_u = u;

end