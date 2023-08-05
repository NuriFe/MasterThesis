desired_arm_config = [0.9000    0.7600   -1.3700    2.0000    1.1000];

[model, nstates, ndof, nmus, iLce] = initialize_model();
%x_desired = initialize_state(desired_arm_config',nstates,iLce);
%desired_hand = wrist_position(x_desired);

data_eq = load('equilibrium.mat');
x_eq = data_eq.x;
x = initialize_state(x_eq(1:11),nstates,iLce);

complete = 0;
warnCount= 0;
while complete == 0
    % Initialize derivatives and muscle excitations
    xdot = zeros(nstates,1);
    step_u = zeros(nmus,1);
    
    %Set simulation parameters
    t = 0;
    tend = 5;
    tstep = 0.001;
    nsteps = round((tend-t)/tstep)+1; % +1 allows for saving of initial state
    
    % Initialize variables for saving
    Fhand=zeros(nsteps,3);
    usave=zeros(nsteps,nmus);
    xsave=zeros(nsteps,length(x));
    HandLocation=zeros(nsteps,3);
    JointAngles=zeros(nsteps,5);
    FBForce=zeros(nsteps,3);
    FBTorque=zeros(nsteps,4);
    TotalTorque=zeros(nsteps,4);
    Fkstep=zeros(nsteps,3);
    Fdstep=zeros(nsteps,3);
    Fistep=zeros(nsteps,3);
    actStep=zeros(nsteps,9);
    GoalLocation=zeros(nsteps,3);

    % Initialize parameters
    u=zeros(nmus,1);
    handF=[0;0;0];

    % Save initial points
    i=1;
    Fhand(i,:)=handF';
    usave(i,:)=u';
    xsave(i,:)=x';
    
    % Compute neural excitations
    alpha = compute_neural_excitation(desired_arm_config);

    for i=2:nsteps
        display_progress(i,nsteps);
        
        for j=1:8
            mus=muscledict(j);
            u(mus)=alpha(j)*1;
        end

        % Set moment and external force
        M = zeros(5,1);
        exF= zeros(2,1);

        % Arm Support
        [dPhand_dx, Phand] = pos_jacobian(x,model);
        supportEq=[0;0.3;-0.15];
        K=diag([0 30 30]);
        B=diag([120 120 120]);
        Vhand=dPhand_dx*x(12:22);
        hands=-K*(Phand-supportEq)-B*Vhand;

        % holdTime=300;
        % if i<holdTime
        %     handF = hands;                
        % else
        %     %handF = fs'+hands;
        % end
        forces(1,:) = handF;

       try
            % Advance simulation by a step
            [x, xdot, step_u] = das3step(x, u, tstep, xdot, step_u, M, exF, handF);
            
            % Save data
            xout(i,:)=x;
            uout(i,:)=u;
            forces(i,:) = handF;

            [warnMsg, warnId] = lastwarn;
        catch exception
             warnMsg = exception.message;
             warnId = exception.identifier;
        end
        
        if ~isempty(warnMsg)
            warnCount=warnCount+1;
            break;
        end

    end

     % if Warning Message pops then reselect the initial LCE
    if ~isempty(warnMsg)
        x=zeros(nstates,1);
        x(1:11)=position;
        x(iLce)=-3+6*rand(nmus,1); % randomly select initial Lce
    else
        complete=1;
    end


end
