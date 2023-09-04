%function [PositionErrors,MuscleForces, timer ] = simulate_reaching_movement(G)
    clear
    clc
    close all
    G = [2.99, 14.478];
    switchTime=1.3;
    
    %Initialize Model
    [model, nstates, ndof, nmus, iLce] = initialize_model();
    
    %Set Parameters to calculate path
    d = 4;

    totry = load('C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis/PathFollowingControl/totry.mat');
    totry = totry.to_try';
    not_good = [1 15 18 21];
    totry(not_good) = [];
    triceps = 0;
    triceps_stroke = 0;
    errors = zeros(length(totry),3);
    sstime = zeros(length(totry),1);
    startPos = 13451;

    figure()
    for indx =[13]%1:length(totry)
        endPos = totry(indx);
        load('feasiblepoints.mat')
        % Find Best Path using KNN
        paths = find_path(d,startPos,endPos);
        paths = [paths paths(end)];% paths(end-3)]; %remember to change it [paths paths(end)]
    
        % Set initial state
        pos = paths(1);
        x  =zeros(nstates,1);
        position = stateFeasible(:,pos);
        x = initialize_state(position,nstates,iLce);
        
        Phand_start=wrist_position(x);
    
        warnCount = 0;
        complete = 0;
        warning('error', 'MATLAB:nearlysingularMatrix');
    
        while complete ==0 && warnCount<5
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
        xsave = zeros(nsteps,nstates);
        usave = zeros(nsteps,138);
        FES = zeros(nsteps,1);
        u_triceps = zeros(nsteps,1);
        u_total = zeros(nsteps,1);

        % Initialize parameters
        u=zeros(nmus,1);
        handF=[0;0;0];
       
        % Set initial static torque and muscle force mapping
        staticTorque = torqueFeasible(GoalPos,:);
        MFM = Mfeasible(:,:,GoalPos);
    
        % Start Simulation
        fprintf('\nSimulating...%s         ',string(indx))

        sumErr = 0;
        alpha0 = zeros(9,1);
        i = 0;
        timer = 0;
        trial = 0; % Counter to try at most 3 times to reach the same position
        last = 0; % Will be set to 1 when the last point of the path is reached
        while last == 0
            i = i+1;
            timer = timer + tstep;
            warnMsg = '';
    
            % Switch the Goal Position
            if mod(i,floor(switchTime/tstep))==0 
                HandCurrent = wrist_position(x);
                distance = norm(HandGoal-HandCurrent);
                
                % If the hand has reached close to the Goal Pose change
                % position in path.
                % If not continue trying to reach the same point.
                if distance < 0.05 || trial >0 
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
            
            %stroke = 7;
            stroke = 1;
            % Time to switch activation and add the stroke function
            if mod(i,10)==0
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
                    %if (j == 1) && stroke >1
                    %    alpha0(j) = alpha0(j)/stroke/2;
    
                    end
                    u(mus)=alpha0(j)*1;
    
                end
            end
            try
                % Advance simulation by a step
                [x, xdot, step_u] = das3step_B(x, u, tstep, xdot, step_u, Moments, exF, handF);%;, K, B);
                xsave (i,:) = x';
                usave(i,:) = u;
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
                 last=1;
            end
    
        end
        
        % if Warning Message pops then reselect the initial LCE
        if ~isempty(warnMsg)
            warnCount = warnCount +1;
            x=zeros(nstates,1);
            x(1:11)=position;
            x(iLce)=-3+6*rand(nmus,1); % randomly select initial Lce
        else
            complete=1;
            errors(indx,:)=error;

            %h9 = figure(9);
            plot_wrist_positions(xsave(1:i-1,:),model,HandGoal)
            hold on
            wrists =  wristFeasible(paths,:);
            plot_wrist_references(wrists,model);
            view(-90,90);
            hold off
            %filename = sprintf('G(%.2f)_G(%.2f)_Stroke_%d_position_totry(%d)', G(1), G(2), stroke, endPos);
            %saveas(h9,['C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis/Data\stroke/',filename, '_wp.jpg'])
            %saveas(h9,['C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis\Data\stroke/',filename,'_wp.fig'])

            
            tout = tstep:tstep:timer;
            xout = xsave(1:i,:);
            h9 = figure(9);
            plot_wrist_positions_2D(tout,xout,HandGoal)
            saveas(h9,['C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis/Data/',sprintf('%.0f',indx),'.png']);

            %uout = usave(1:i,:);
            %h10 = figure(10);
            %plot_neurexct(tout,uout);
            %filename = sprintf('G(%.2f)_G(%.2f)_Stroke_%d_position_totry(%d).png', G(1), G(2), stroke, endPos);

            %saveas(h10,['C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis/Data\stroke/',filename, '_ne.jpg'])
            %saveas(h10,['C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis\Data\stroke/',filename,'_ne.fig'])
            %MuscleForces = MuscleForces(1:i,:);
            %PositionErrors = PositionErrors(1:i,:);
            
            %subplot(5, 6, indx);
            %figure()
            %plot_wrist_positions(xsave(1:i-1,:),model,HandGoal)
            %hold on
            %wrists =  wristFeasible(paths,:);
            %plot_wrist_references(wrists,model);
            %view(-90,90);
            %hold off
            
            %u_triceps = u_triceps(1:i,:);
            %FES = FES(1:i,:);
            %u_total = u_total(1:i,:);
            %plotting_triceps_FES(tout, u_triceps, FES, u_total)
        end

    
        end
    end
%end

function Kp = sigmoidGainControl(x, k1, k2, a, x0)
    % x: The input value for which you want to compute the gain.
    % k1: Gain for small values.
    % k2: Gain for large values.
    % a: Controls the steepness of the sigmoid transition.
    % x0: Center point of the sigmoid transition.
    
    % Compute the sigmoid value
    S = 1 / (1 + exp(-a * (x - x0)));
    
    % Compute the gain based on the sigmoid value
    Kp = k1 + (k2 - k1) * S;
end