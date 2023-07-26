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
startPos = 10;
endPos = 20;

% Find Best Path using KNN
%paths = find_path(d,reachdist,startPos,endPos);
%paths = [paths paths(end)];

% Set initial state
eq = load('equilibrium.mat');
x=zeros(nstates,1);
%x(1:11)=stateFeasible(:,pos);
position = eq.x(1:11);

LCEopt=das3('LCEopt');
muscle_tendon_lengths = das3('Musclelengths', x);    % only the first 11 elements of x (the joint angles) will be used
slack_lengths = das3('SEEslack');
Lce = muscle_tendon_lengths - slack_lengths;
x(iLce)=Lce; % randomly select initial Lce

Phand_start=wrist_position(x)';

count = 0;
warnCount = 0;

% Set goal position
goal = load('C:\Users\s202421\Documents\GitHub\MasterThesis\Data\forces\64\55_4.mat');
HandGoal = wrist_position(goal.x);
 % Set initial static torque and muscle force mapping
[~, ~, ~, ~, ~, ~, qTH] = das3('Dynamics',goal.x, zeros(size(goal.x)), zeros(138,1));
q=[qTH;goal.x(10:11)]';
staticTorque = predict_static_torque(q);
MFM = predict_activation_torque(q);

complete = 0;
while count == 0 && complete ==0
    warning('')

    %HandGoal = [0.3 0 -0.1];

    % Initialize derivatives and muscle excitations
    xdot = zeros(nstates,1);
    step_u = zeros(nmus,1);
    
    % Set simulation parameters
    time = 0;
    tend = 3;%switchTime*(length(paths)-1);
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
    actStep=zeros(nsteps,3);
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
    %JointAngles(i,:)=qFeasible(pos,:);
    GoalLocation(i,:)=HandGoal;
    
   
    
    % Start Simulation
    fprintf('\nSimulating...        ')
    sumErr = 0;
    alpha0 = zeros(3,1);

    for i = 2:nsteps
        lastwarn('');
        display_progress(i,nsteps);

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
            alpha0=computeActivations(MFM,tau_des,alpha0);
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
    end            



end