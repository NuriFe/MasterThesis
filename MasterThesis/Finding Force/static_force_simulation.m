%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Finding Force for Static Position Without Neural Excitation
% 
% Description:
% This code simulates the dynamics of a model to determine the force 
% required for static positions. It uses an iterative process to adjust the
%  model's state and uses PI control to achieve desired hand positions. 
% Any warnings during the simulation due to issues like a near-singular 
% matrix are handled, and if necessary, the initial muscle length (Lce) 
% is reselected.
% 
% Inputs:
% 1. DAS Model Initialization - through initialize_model().
% 2. Predefined positions - created by create_grid().
% 3. 'equilibrium.mat' - loaded as initial state.
% 
% Outputs:
% 1. `confg` - array containing the configurations for which the simulation 
% was successful.
% 2. A saved .mat file (e.g., '960.mat') with the `confg` data.
% 
% Additional notes:
% - Errors in position are calculated and if they exceed a certain 
% threshold, the iteration is marked for repetition.
% - Data pertaining to the model dynamics, arm configurations, and mean
%  force are saved at a specified location.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


clear
close all
clc

% Initialize DAS model
[model, nstates, ndof, nmus, iLce] = initialize_model();

warnidx=[]; % used to record warnings from MATLAB

% Load Positions
w_refs = create_grid(0,model);
% To repeat after testing
repeat = [];
warning('error', 'MATLAB:nearlysingularMatrix');
confg = zeros(length(w_refs),11);
errors = zeros(length(w_refs),3);
sstime = zeros(length(w_refs),1);
simulation_time = zeros(length(w_refs),1);
%% Loop through all positions
for j = 1:length(w_refs)
    % Initiliaze variables
    mean_force = zeros(10,3);
    warn = 0;
    tic
    %% Lopp through muscles 
    for muscle = 9
        warnCount = 0;
        complete = 0;

        % Initiliaze state
        load('equilibrium.mat')
        position = x(1:11);
        x = initialize_state(position,nstates, iLce);

        % Exit loop if too many warning
        while warnCount<=10 && complete ==0
            warning('')
            %muscle
                       
            % Set simulation parameters
            t = 0;
            tend = 1;
            tstep = .001; %default = .003 
            nsteps = round((tend-t)/tstep)+1; % +1 allows for saving of initial state
           
            % Initialize variables
            xdot = zeros(nstates,1);
            step_u = zeros(nmus,1);

            % Create space for saving variables
            tout = tstep*(0:nsteps)';
            forces = zeros(nsteps,3);
            xout = zeros(nsteps,nstates);
            uout = zeros(nsteps,nmus);
            armTorque = zeros(nsteps,ndof);

            % Set start and goal hand positions
            hand_start = wrist_position(x)';
            hand_goal = w_refs(j,:)';

            % Set initial force hand
            handF = [0;0;0];

            % Initialize errors
            error_int = 0;

            % Save starting point
            xout(1,:)=x;
            uout(1,:)=zeros(nmus,1);
            armTorque(1,:)=das3('Jointmoments',x);

            fprintf('\nSimulating...        ')

            % Run Simulation
            for i=2:nsteps
                display_progress(i,nsteps);
                
                %Set neural excitation
                u = zeros(nmus,1);

                % Set moment and external force
                M = zeros(5,1);
                exF= zeros(2,1);

                %% PI Controller
                K = eye(3)*2000;
                I = 100;
                % PID 
                hand_current = wrist_position(x);
                error_pos = hand_goal-hand_current;
                error_int = error_int + error_pos*tstep;

                % PI calculation for time step
                handF = K*error_pos+I*error_int;
                
                % Arm Support
                [dPhand_dx, Phand] = pos_jacobian(x,model);
                supportEq=[0;0.3;-0.15];
                K=diag([0 30 30]);
                B=diag([120 120 120]);
                Vhand=dPhand_dx*x(12:22);
                handF=handF-K*(Phand-supportEq)-B*Vhand;

                % Advance simulation by a step
                try
                    [x, xdot, step_u] = das3step(x, u, tstep, xdot, step_u, M, exF, handF);
                
                    % Save data
                    forces(i,:)= handF;
                    xout(i,:)=x;
                    uout(i,:)=u;
                    armTorque(i,:)=das3('Jointmoments',x);
                    [warnMsg, warnId] = lastwarn;

                    tolerance = [0.2 0.2 0.2];
                    if i>100 && sstime(j,:)==0
                        avg_last_50 = mean(forces(i-50:i-1,:));
                        if all(abs(forces(i,:) - avg_last_50) < tolerance) 
                            sstime(j,:)=i*tstep;
                        end
                    end
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

        if warnCount>10
            warnidx=[warnidx position];
            warn=1;
        end
        
        simulation_time(j,:)=toc;
        error_pos = calculate_error(xout,hand_goal,j);
        errors(j,:)=error_pos;
        cut = round(length(forces)*0.9);
        mean_force=mean(forces(cut:end,:));
        %plot_multiple(xout,uout,forces,tstep,tend,tout(2:end,:),hand_goal,model)
        %wrist_error = test(xout,mean_force,hand_goal,model,tend,tstep);
        if any(abs(error_pos)>0.05)
            repeat = [repeat j];
        else
            confg(j,:) = x(1:11);
            %[f, ~, ~, ~, ~,~,qTH] = das3('Dynamics',x,xdot,step_u,M,exF,handF);
            %arm_config = [qTH' x(10:11)'];
            %save(['C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis\Data\Crazy/',num2str(j),'_',num2str(muscle),'.mat'],'xout','forces','x','arm_config','mean_force');
        end

        
    end
end


%save('C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis\Data\960.mat','confg');