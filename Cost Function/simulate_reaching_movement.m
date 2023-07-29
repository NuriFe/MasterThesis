function [PositionErrors,MuscleForces ] = simulate_reaching_movement(G,end_pos)
    switchTime=1.3;
    
    %Initialize Model
    [model, nstates, ndof, nmus, iLce] = initialize_model();
    
    %Set Parameters to calculate path
    reachdist = 30;
    d = 4;
    startPos = 13451;
    endPos = end_pos;

    load('feasiblepoints.mat')

    endPos = totry(idx);
    % Find Best Path using KNN
    paths = find_path(d,reachdist,startPos,endPos);
    paths = [paths paths(end)];

    % Set initial state
    pos = paths(1);
    x=zeros(nstates,1);
    x(1:11)=stateFeasible(:,pos);
    
    LCEopt=das3('LCEopt');
    muscle_tendon_lengths = das3('Musclelengths', x);    % only the first 11 elements of x (the joint angles) will be used
    slack_lengths = das3('SEEslack');
    Lce = muscle_tendon_lengths - slack_lengths;
    x(iLce)=Lce; % randomly select initial Lce
    
    Phand_start=wrist_position(x);

    warnCount = 0;
    complete = 0;

    while complete ==0
    warning('')

    % Set goal position
    pathIdx = 2;
    GoalPos = paths(pathIdx);
    qGoal = stateFeasible(:, GoalPos);
    xGoal = initialize_state(qGoal, nstates, iLce);
    HandGoal = wrist_position(xGoal);

    % Initialize derivatives and muscle excitations
    xdot = zeros(nstates,1);
    step_u = zeros(nmus,1);
    
    % Set simulation parameters
    time = 0;
    tend = switchTime*(length(paths)-1)*2;
    tstep = .003; % default was 0.003; decreased to make more stable numerically
    nsteps = round((tend-time)/tstep)+1; 
    
    % Initialize variables for saving
    MuscleForces = zeros(nsteps,9);
    PositionErrors = zeros(nsteps,3);

    % Initialize parameters
    u=zeros(nmus,1);
    handF=[0;0;0];
   
    % Set initial static torque and muscle force mapping
    staticTorque = torqueFeasible(GoalPos,:);
    MFM = Mfeasible(:,:,GoalPos);

    % Start Simulation
    fprintf('\nSimulating...        ')
    sumErr = 0;
    alpha0 = zeros(9,1);
    i = 0;
    timer = 0;
    trial = 0; % Counter to try at most 3 times to reach the same position
    last = 0; % Will be set to 1 when the last point of the path is reached
    while last == 0
        lastwarn('');
        i = i+1;
        timer = timer + tstep;

        % Switch the Goal Position
        if mod(i,floor(switchTime/tstep))==0 
            HandCurrent = wrist_position(x);
            distance = norm(HandGoal-HandCurrent);
            
            % If the hand has reached close to the Goal Pose change
            % position in path.
            % If not continue trying to reach the same point.
            if distance < 0.03 || trial >2 
                if pathIdx < length(paths)
                    pathIdx=pathIdx+1;
                    fprintf("New point %s \n", string(pathIdx))
                else 
                    last =1;
                end
                GoalPos=paths(pathIdx);
                HandGoal=wristFeasible(paths(pathIdx),:)';
                staticTorque=torqueFeasible(GoalPos,:);
                MFM=Mfeasible(:,:,GoalPos);
                openLoopAct=activationFeasible(GoalPos,:);
                if d~=100
                    sumErr=0;
                end
                trial = 0;
            else
                trial = trial +1;
                fprintf("Repeating Hand Goal %s \n\n",string(pathIdx))                        
            end
        end


        % Arm Support
        [dPhand_dx, Phand] = pos_jacobian(x,model);
        supportEq=[0;0.3;-0.15];
        K=diag([0 30 30]);
        B=diag([120 120 120]);
        Vhand=dPhand_dx*x(12:22);
        handF=-K*(Phand-supportEq)-B*Vhand;
        holdTime=166;
        if i<holdTime
            Khold=eye(3)*2000;
            Bhold=eye(3)*200;
            holdF=-Khold*(Phand-Phand_start)-Bhold*Vhand;
            handF=handF+holdF;
            
            
            K=K+Khold;
            B=B+Bhold;
        end


        Moments = zeros(5,1);
        exF=[0;0];
    
        % PI Feedback
        Kp = 250;
        Ki = 80;

        Phand = wrist_position(x);
        sumErr=sumErr+(HandGoal-Phand)*tstep;
        Fk=-Kp*(Phand-HandGoal);
        Fi=Ki*sumErr;
        Fdes=Fk+Fi;
        
        % Compute torque from Kinematic Jacobian
        %[dPhand_dx, ~, ~] = handpos_jacobian(x);
        %tau_feedback = dPhand_dx'*Fdes;
        %indx = [1:2 9:10];
        %tau_feedback= tau_feedback(indx,:);
        [~, ~, ~, ~, ~, ~, qTH] = das3('Dynamics',x, zeros(size(x)), zeros(138,1));
        pose=[qTH;x(10:11)];
        J=computeJacobianDAS_5angles(pose);
        J=J(:,1:4);
        tau_feedback=J'*Fdes;

        % Compute desired torque
        tau_des = tau_feedback + staticTorque';
        
        stroke = 10;
        % Time to switch activation and add the stroke function
        if mod(i,25)==0
            alpha0=computeActivations(MFM,tau_des,alpha0);
            for j=1:9
                mus=whichMuscles(j);
                if j == 5
                    alpha0(j)= alpha0(j)*(stroke); %biceps overactivity
                    if alpha0(j) >1
                        alpha0(j) = 1;
                    elseif alpha0(j) == 0 && stroke >1
                        alpha0(j)= 0.3;
                    end
                elseif (j == 1 || j == 2) && stroke >1
                    alpha0(j) = alpha0(j)/stroke/2;

                end
                u(mus)=alpha0(j)*1;

            end
        end
        
        % G calculation
        Kp = G(1);
        %Kd = G(2);
        
        u = Kp*u;
        
        try
            % Advance simulation by a step
            [x, xdot, step_u] = das3step_B(x, u, tstep, xdot, step_u, Moments, exF, handF);%;, K, B);
            force = das3('Muscleforces', x);
            forces = zeros(9,1);
            for j=1:9
                mus=whichMuscles(j);
                frc = force(mus);
                forces(j) = mean(frc);
            end
            MuscleForces(i,:)= forces';

            [~, Phand] = pos_jacobian(x,model);
            error = HandGoal-Phand;

            PositionErrors(i,:) = error;
        catch exception
             warnMsg = exception.message;
             warnId = exception.identifier;
        end

                % if Warning Message pops then reselect the initial LCE
        if ~isempty(warnMsg)
            x=zeros(nstates,1);
            x(1:11)=position;
            x(iLce)=-3+6*rand(nmus,1); % randomly select initial Lce
        else
            complete=1;
            plot_wrist_positions(xsave(1:i-1,:),model,HandGoal)
            hold on
            wrists =  wristFeasible(paths,:);
            plot_wrist_references(wrists,model);
            view(-90,90);
            hold off
        end
    end
    
    MuscleForces = MuscleForces(1:i,:);
    PositionErrors = PositionErrors(1:i,:);
   

end