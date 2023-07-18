%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Finding Force for Static Position with Neural Excitation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


clear
close all
clc

% Initialize DAS model
[model, nstates, ndof, nmus, iLce] = initialize_model();

warnidx=[]; % used to record warnings from MATLAB

% Load Positions
directory = 'C:\Users\s202421\Documents\GitHub\MasterThesis\Data\static_forces';
fileList = dir(fullfile(directory, '*.mat'));
positions = zeros(18,11);
for i = 1:length(fileList)
    % Load the .mat file
    name = fileList(i).name;
    filePath = fullfile(directory, name);
    data = load(filePath);
    config = data.arm_config;
    positions(i,:)=config';
end
warning('error', 'MATLAB:nearlysingularMatrix');

%% Loop through all positions
for j = 2:length(positions)
    position = positions(j,:);

    % Initiliaze variables
    mean_force = zeros(10,3);
    warn = 0;

    %% Lopp through muscles 
    for muscle = 1:4
        warnCount = 0;
        complete = 0;

        % Initiliaze state
        x = initialize_state(position,nstates, iLce);

        % Exit loop if too many warning
        while warnCount<=10 && complete ==0
            warning('')
            muscle
                       
            % Set simulation parameters
            t = 0;
            tend = 0.5;
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
            hand_goal = hand_start';

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
                muscles = whichMuscle(muscle);
                u(muscles)=1;

                % Set moment and external force
                M = zeros(5,1);
                exF= zeros(2,1);

                %% PI Controller
                K = eye(3)*2000;
                I = 100;
                %% PID 
                [dPhand_dx, Phand] = pos_jacobian(x,model); % find position and velocity of the hand
                B= 300;
                Vhand=dPhand_dx*x(12:22);

                hand_current = wrist_position(x);
                error_pos = hand_goal-hand_current;
                error_int = error_int + error_pos*tstep;

                % PI calculation for time step
                handF = K*error_pos+I*error_int;

                %PID calculation for time step
                handF= K*error_pos+I*error_int -B*Vhand;

                try
                    % Advance simulation by a step
                    [x, xdot, step_u] = das3step(x, u, tstep, xdot, step_u, M, exF, handF);
                    
                    % Save data
                    forces(i,:)= handF;
                    xout(i,:)=x;
                    uout(i,:)=u;
                    armTorque(i,:)=das3('Jointmoments',x);
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

        error_pos = hand_goal-hand_current;
        if warnCount>10
            warnidx=[warnidx position];
            warn=1;
        end
        
        %figure()
        %plot_wrist_positions(xout,model,hand_goal);
        
        error_pos = calculate_error(xout,hand_goal,j);

        cut = round(length(forces)*0.9);
        mean_force=mean(forces(cut:end,:));

        save(['C:\Users\s202421\Documents\GitHub/MasterThesis/Data/forces/PID/',num2str(j),'_',num2str(muscle),'.mat'],'xout','forces','x', 'handF', "armTorque");

    end
end

function error = calculate_error(xout, wref, pos)
    x_final = xout(end,:)';
    w_final = wrist_position(x_final);
    error = wref-w_final;
    wrist = string(wref');
    wrist_error = string(roundn(error',-3));

    message = sprintf('Data for w_ref %s x:%s+%s y:%s+%s z:%s+%s\n ', ...
    string(pos), wrist(1,1), wrist_error(1,1), wrist(1,2), wrist_error(1,2), wrist(1,3), wrist_error(1,3));
    disp(message);
end

function x = initialize_state(position, nstates, iLce)
    x=zeros(nstates,1);
    LCEopt=das3('LCEopt');
    lengths = das3('Musclelengths', x);    % only the first 11 elements of x (the joint angles) will be used
    SEEslack = das3('SEEslack');
    Lce = (lengths - SEEslack);%./LCEopt;
    x(iLce)=Lce; % randomly select initial Lce
    x(1:11)=position;

end


function display_progress(i,nsteps)
    if round(1000*i/nsteps)/10 < 10
        fprintf('\b\b\b\b\b%3.1f%%\n', round(1000*i/nsteps)/10)
    elseif round(1000*i/nsteps)/10 < 100
        fprintf('\b\b\b\b\b\b%3.1f%%\n', round(1000*i/nsteps)/10)
    elseif round(1000*i/nsteps)/10 < 1000
        fprintf('\b\b\b\b\b\b\b%3.1f%%\n', round(1000*i/nsteps)/10)
    end
end