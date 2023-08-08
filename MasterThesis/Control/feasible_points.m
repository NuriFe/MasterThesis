
clear 
clc


[model, nstates, ndof, nmus, iLce] = initialize_model();
%x_desired = initialize_state(desired_arm_config',nstates,iLce);
%desired_hand = wrist_position(x_desired);

% Load Positions
%directory = 'C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis\Data\Crazy';
%fileList = dir(fullfile(directory, '*.mat'));

% positions = zeros(length(fileList),11);
% tocheck = [];
% error_to_check=[];
% for i = 1:length(fileList)
%     % Load the .mat file
%     name = fileList(i).name;
%     filePath = fullfile(directory, name);
%      % Extract the number using regular expressions
%     number = regexp(name, '\d+', 'match');
%     % Convert the extracted number to a numeric value
%     number = str2double(number(1));
% 
%     data = load(filePath);
%     x = data.x;
%     positions(number,:)=x(1:11,:);
% end
% %index = randperm(120, 30);
%A = 1:120;
%toDelete = [ 99, 76, 96, 116, 78, 17,84];
tocheck = [];
error_to_check=[];
% Deleting the specified numbers
%A(ismember(A, toDelete)) = [];
load('C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis\Data\960.mat')
positions = confg;
for j = 301:960
    disp(j);
    position = positions(j,:);
    complete = 0;
    warnCount= 0;
    x = initialize_state(position,nstates,iLce);
    [~, ~, ~, ~, ~, ~, qTH] = das3('Dynamics',x, zeros(size(x)), zeros(138,1));
    desired_arm_config=[qTH;x(10:11)]';
    desired_hand = wrist_position(x);
    warnCount = 0;

    while complete == 0
        % Initialize derivatives and muscle excitations
        xdot = zeros(nstates,1);
        step_u = zeros(nmus,1);
        
        %Set simulation parameters
        t = 0;
        tend = 1;
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
        
        sumErr=0;
        alpha0=zeros(8,1);
        for i=2:nsteps
            current_hand = wrist_position(x);
            HandLocation(i,:) = desired_hand-current_hand;
            %display_progress(i,nsteps);
    
    
            % Set moment and external force
            M = zeros(5,1);
            exF= zeros(2,1);

            Kp=250;
            Kd=0;
            Ki=80;
            
            sumErr=sumErr+(desired_hand-current_hand)*tstep;
            Fk=-Kp*(current_hand-desired_hand);
            Fi=Ki*sumErr;
            corrective_force=Fk+Fi;
            
            FBForce(i,:) = corrective_force;
            
            J=kinematic_jacobian(x);
            J=J(:,1:4);
            torque_feedback=J'*corrective_force; 
    
            static_torque = predict_static_torque(desired_arm_config)';
            desired_torque = torque_feedback+static_torque;
            TotalTorque(i,:)=desired_torque;
            if mod(i,10)==0
                alpha0 = compute_neural_excitation(desired_arm_config, ...
                    desired_torque,alpha0);
                for m=1:8
                    mus=whichMuscles(m);
                    u(mus)=alpha0(m)*1;
                end            
            end
            
            % Arm Support
            [dPhand_dx, Phand] = pos_jacobian(x,model);
            supportEq=[0;0.3;-0.15];
            K=diag([0 30 30]);
            B=diag([120 120 120]);
            Vhand=dPhand_dx*x(12:22);
            handF=-K*(Phand-supportEq)-B*Vhand;
            
            holdTime=100;
            if i<holdTime
                Khold=eye(3)*2000;
                Bhold=eye(3)*200;
                holdF=-Khold*(Phand-desired_hand)-Bhold*Vhand;
                handF=handF+holdF;
                
                
                K=K+Khold;
                B=B+Bhold;
                
            end

    
           try
                % Advance simulation by a step
                [x, xdot, step_u] = das3step(x, u, tstep, xdot, step_u, M, exF, handF);
                
                % Save data
                xsave(i,:)=x;
                uout(i,:)=u;
                forces(i,:) = handF;
    
                [warnMsg, warnId] = lastwarn;
            catch exception
                 warnMsg = exception.message;
                 warnId = exception.identifier;
            end
            
            if ~isempty(warnMsg)
                warnCount=warnCount+1
                break;
            end

            wrist_error = desired_hand-current_hand;
            if any(abs(wrist_error)>0.1)
                break
            end
    
        end
    
         % if Warning Message pops then reselect the initial LCE
        if ~isempty(warnMsg) && warnCount <10
            x=zeros(nstates,1);
            x(1:11)=position;
            x(iLce)=-3+6*rand(nmus,1); % randomly select initial Lce
        else
            complete=1;
        end

    
    
    end
    disp([j,wrist_error(1,1), wrist_error(2,1), wrist_error(3,1)]);

    if any(abs(wrist_error)>0.1)

        tocheck = [tocheck j];
        wrist_error = desired_hand-current_hand;
        error_to_check = [error_to_check wrist_error];
    else
        %save(['C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis\Data\Crazy',num2str(j),'.mat'],'xsave','forces','uout','TotalTorque');

    end
    % figure()
    % subplot(1,3,1)
    % plot_hand_forces(FBForce',tstep,tend)
    % subplot(1,3,2)
    % plot(TotalTorque)
    % subplot(1,3,3)
    % plot_error_position(HandLocation,tstep,tend)
    % name = [j,Kp(1,1),Kp(2,2),Kp(3,3),Kd,Ki];
    % title(name)
end
save('C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis\Data\301_960_to_check.mat','tocheck','error_to_check');
