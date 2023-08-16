%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Point_to_Point.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
clc
close all

switchTime=1.3;

%Initialize Model
[model, nstates, ndof, nmus, iLce] = initialize_model();

%Set Parameters to calculate path
reachdist = 30;
d = 4;
startPos = 13451;
endPos = 7218;

% Find Best Path using KNN
paths = find_path(d,reachdist,startPos,endPos);
paths = [paths paths(end)];

%Load feasible points
load('feasiblepoints.mat') %MFM feasible, state feasible, wrist feasible, torque feasible

% Set initial state
pos = paths(1);
x=zeros(nstates,1);
x(1:11)=stateFeasible(:,pos);

LCEopt=das3('LCEopt');
muscle_tendon_lengths = das3('Musclelengths', x);    % only the first 11 elements of x (the joint angles) will be used
slack_lengths = das3('SEEslack');
Lce = muscle_tendon_lengths - slack_lengths;
x(iLce)=Lce; % randomly select initial Lce

Phand_start=wrist_position(x)';

count = 0;
warnCount = 0;
complete = 0;
while count == 0 && complete ==0
    warning('')

    % Set goal position
    GoalPos = paths(2);
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

    % Initialize parameters
    u=zeros(nmus,1);
    handF=[0;0;0];

    % Save initial position
    i=1;
    Fhand(i,:)=handF';
    usave(i,:)=u';
    xsave(i,:)=x';
    HandLocation(i,:)=Phand_start';
    JointAngles(i,:)=qFeasible(pos,:);
    GoalLocation(i,:)=HandGoal;
    
    % Set initial static torque and muscle force mapping
    staticTorque = torqueFeasible(GoalPos,:);
    MFM = Mfeasible(:,[1 2 5],GoalPos);
    
    % Start Simulation
    fprintf('\nSimulating...        ')
    sumErr = 0;
    alpha0 = zeros(3,1);
    pathIdx = 2;
    for i = 2:nsteps
        lastwarn('');
        display_progress(i,nsteps);

        % Switch the Goal Position
        if mod(i,floor(switchTime/tstep))==0 && i<nsteps-1.5*floor(switchTime/tstep) % makes sure last steps don't switch
            HandCurrent = wrist_position(x);

            distance = norm(HandGoal-HandCurrent);
            if distance < 0.03
               
                if pathIdx < length(paths)
                    pathIdx=pathIdx+1;
                else 
                    last =1;
                end
                GoalPos=paths(pathIdx);
                HandGoal=wristFeasible(paths(pathIdx),:)';
                staticTorque=torqueFeasible(GoalPos,:);
                M=Mfeasible(:,:,GoalPos);
                openLoopAct=activationFeasible(GoalPos,:);
                if d~=100
                    sumErr=0;
                end
            else
                nsteps = nsteps+floor(switchTime/tstep);
                fprintf("Repeating Hand Goal %s \n\n",string(pathIdx))                        
            end
        end


        holdTime=166;
        if i<holdTime
            [dPhand_dx, Phand] = pos_jacobian(x,model);
            supportEq=[0;0.3;-0.15];
            K=diag([0 30 30]);
            B=diag([120 120 120]);
            Vhand=dPhand_dx*x(12:22);
            handF=-K*(Phand-supportEq)-B*Vhand;
        
            Khold=eye(3)*2000;
            Bhold=eye(3)*200;
            holdF=-Khold*(Phand-Phand_start')-Bhold*Vhand;
            handF=(handF+holdF);
            
            
            K=K+Khold;
            B=B+Bhold;
        else
            handF=[0;0;0];
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
        [dPhand_dx, ~, ~] = handpos_jacobian(x);
        tau_feedback = dPhand_dx'*Fdes;
        indx = [1:3 10];
        tau_feedback= tau_feedback(indx,:);
        
        % Compute desired torque
        tau_des = tau_feedback + staticTorque';

        % Time to switch activation
        if mod(i,25)==0
            alpha0=computeActivation_Nuri(MFM,tau_des,alpha0);
            for j=1:3
                mus=whichMuscle(j);
                u(mus)=alpha0(j)*1;
            end
        end
        
        try
            % Advance simulation by a step
            [x, xdot, step_u] = das3step_B(x, u, tstep, xdot, step_u, Moments, exF, handF);%, K, B);
    
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
        catch exception
             warnMsg = exception.message;
             warnId = exception.identifier;
        end
        [warnMsg, warnId] = lastwarn;        
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
        plot_wrist_positions(xsave(1:i-1,:),model,HandGoal)
        hold on
        wrists =  wristFeasible(paths,:);
        plot_wrist_references(wrists,model);
        view(-90,90);
        hold off
    end               



end