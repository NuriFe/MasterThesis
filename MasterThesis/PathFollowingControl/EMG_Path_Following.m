%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% EMG_Path_Following.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
clc
close all

switchTime=1.3;

%Initialize Model
[model, nstates, ndof, nmus, iLce] = initialize_model();

%Set Parameters to calculate path
d = 4;
startPos = 13451;

totry = load('C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis/System Modelling\matlab.mat');
totry = totry.to_try';
not_good = [1 15 18 21];
totry(not_good) = [];

%Load feasible points
load('feasiblepoints.mat') %MFM feasible, state feasible, wrist feasible, torque feasible

for idx = 2:length(totry)
    endPos = totry(idx);

    % Find Best Path using KNN
    paths = find_path(d,startPos,endPos);
    paths = [paths paths(end)];

    % Set initial state
    pos = paths(1);
    x=zeros(nstates,1);
    x = initialize_state(stateFeasible(:,pos),nstates,iLce);
    
    hand_start=wrist_position(x);

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
        
        MuscleForces = zeros(nsteps,9);
        % Initialize parameters
        u=zeros(nmus,1);
        handF=[0;0;0];
    
        % Save initial position
        i=1;
        Fhand(i,:)=handF';
        usave(i,:)=u';
        xsave(i,:)=x';
        HandLocation(i,:)=hand_start';
        JointAngles(i,:)=qFeasible(pos,:);
        GoalLocation(i,:)=HandGoal;
        
        % Set initial static torque and muscle force mapping
        staticTorque = torqueFeasible(GoalPos,:);
        MFM = Mfeasible(:,:,GoalPos);%Mfeasible(:,[1 2 5],GoalPos);
        
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
                if distance < 0.03 || trial >1 
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
                holdF=-Khold*(Phand-hand_start)-Bhold*Vhand;
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
            
            stroke = 1;
            % Time to switch activation and add the stroke function
            if mod(i,33)==0
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
            
            try
                % Advance simulation by a step
                [x, xdot, step_u] = das3step_B(x, u, tstep, xdot, step_u, Moments, exF, handF);%;, K, B);
        
                Fhand(i,:)=handF;
                usave(i,:)=u;
                xsave(i,:)=x;
                HandLocation(i,:)=Phand;
                FBForce(i,:)=Fdes;
                FBTorque(i,:)=tau_feedback;
                TotalTorque(i,:)=tau_des;
                Fkstep(i,:)=Fk;
                Fistep(i,:)=Fi;
                actStep(i,:)=alpha0;
                GoalLocation(i,:)=HandGoal;
    
                force = das3('Muscleforces', x);
                forces = zeros(9,1);
                for j=1:9
                    mus=whichMuscles(j);
                    frc = force(mus);
                    forces(j) = mean(frc);
                end
                MuscleForces(i,:)= forces';
    
            catch exception
                 warnMsg = exception.message;
                 warnId = exception.identifier;
            end
            [warnMsg, warnId] = lastwarn;        
            if ~isempty(warnMsg)
                warnCount=warnCount+1;
                break;
            end 
            
            % Read Neural Excitation
            triceps_indx = whichMuscles(1);
            triceps = u(triceps_indx);
            ant_deltoids_indx = whichMuscles(11);
            ant_deltoids = u(ant_deltoids_indx);
    
            %% PD Controller
            % Hand error and derivative
            error = HandGoal - Phand;
            [dPhand_dx, Phand] = pos_jacobian(x,model);
            Vhand=dPhand_dx*x(12:22);hand=dPhand_dx*x(12:22);
            
            % Force
    
        
    
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
        error = HandGoal-Phand;
    
    
        %trim to save
        usave = usave(1:i,:);
        xsave = xsave(1:i,:);
        HandLocation = HandLocation(1:i,:);
        FBForce = FBForce(1:i,:);
        FBTorque = FBTorque(1:i,:);
        TotalTorque = TotalTorque(1:i,:);
        actStep = actStep(1:i,:);
        Fkstep = Fkstep(1:i,:);
        Fistep = Fistep(1:i,:);
        GoalLocation = GoalLocation(1:i,:);
        tsave=tstep*(0:i-1);
        
        % figure()
        % subplot(1,2,1)
        % plot(tsave(100:i),MuscleForces(100:i,[1 2 5]))
        % legend('triceps', 'deltoids', 'biceps')
        % subplot(1,2,2)
        % plot(tsave(100:i),actStep(100:i,[1 2 5]))
        % legend('triceps', 'deltoids', 'biceps')
    
    
        error = HandGoal-Phand;
        
    
    
        %filename = ['C:\Users\s202421\Documents\GitHub\MasterThesis\Data\neural_excitation/','Point_', num2str(endPos)];
        %save(filename,'usave','xsave','tsave','HandLocation','GoalLocation','Fkstep','Fistep','FBForce','FBTorque','TotalTorque','actStep')


    end
end