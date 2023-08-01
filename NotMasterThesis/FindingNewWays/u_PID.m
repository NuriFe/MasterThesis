clear
close all
clc

% Initialize DAS model
[model, nstates, ndof, nmus, iLce] = initialize_model();

warnidx=[]; % used to record warnings from MATLAB

% Load Positions
directory = 'C:\Users\s202421\Documents\GitHub\MasterThesis\Data\static_forces/64';
fileList = dir(fullfile(directory, '*.mat'));

positions = zeros(length(fileList),11);
forces = zeros(length(fileList),3);
for i = 1:length(fileList)
    % Load the .mat file
    name = fileList(i).name;
    filePath = fullfile(directory, name);
     % Extract the number using regular expressions
    number = regexp(name, '\d+', 'match');
    % Convert the extracted number to a numeric value
    number = str2double(number(1));

    data = load(filePath);
    config = data.arm_config;
    positions(number,:)=config';
    forces(number,:) = data.mean_force;
end

% Load Positions
%w_refs = create_grid(0,model);

% To repeat after testing
repeat = [];
warning('error', 'MATLAB:nearlysingularMatrix');

%% Loop through all positions
warnCount = 0;
complete = 0;
for j = 1:length(positions)
    goal_position = positions(j,:);

    % Initiliaze variables
    handF = forces(i,:);
    warn = 0;
    
    % Initiliaze state
    load('equilibrium.mat')
    position = x(1:11);
    x = initialize_state(position,nstates, iLce);

    while warnCount<=10 && complete ==0
        warning('')
        % Set simulation parameters
        t = 0;
        tend = 3;
        tstep = .001; %default = .003 
        nsteps = round((tend-t)/tstep)+1; % +1 allows for saving of initial state

        % Initialize variables
        xdot = zeros(nstates,1);
        step_u = zeros(nmus,1);

        % Create space for saving variables
        tout = tstep*(0:nsteps)';
        xout = zeros(nsteps,nstates);
        uout = zeros(nsteps,nmus);


        % Set start and goal hand positions
        hand_start = wrist_position(x)';
        x_goal = initialize_state(goal_position,nstates,iLce);
        hand_goal = wrist_position(x_goal);

        % Set initial force hand
        handF = forces(i,:)';

        % Initialize errors
        error_int = 0;

        % Save starting point
        xout(1,:)=x;
        uout(1,:)=zeros(nmus,1);
        armTorque(1,:)=das3('Jointmoments',x);

        fprintf('\nSimulating...        ')
        i = 0;
        error_pos = norm(hand_goal-hand_start);

        % Run Simulation
        while error_pos > 0.2 && i < 10000
            i = i+1;
            if mod(i,1000) == 0
                i
                fprintf('%3.1f\n', error_pos)

            end
            %display_progress(i,nsteps);
            
            %PI neural excitation
            K = 2;
            I = 0.1;

            u = zeros(nmus,1);
            hand_current = wrist_position(x);
            error_pos = norm(hand_goal-hand_current);
            error_int = error_int + error_pos*tstep;

            u_pid = abs(K*error_pos + I*error_int);
            u_pid = min(u_pid, 1);

            muscles = whichMuscle(1);
            u(muscles)=max(u_pid);

            % Set moment and external force
            M = zeros(5,1);
            exF= zeros(2,1);

            try
                [x, xdot, step_u] = das3step(x, u, tstep, xdot, step_u, M, exF, handF);
            
                % Save data
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

        error_pos = calculate_error(xout,hand_goal,j);
        figure()
        plot_wrist_positions(xout,model,hand_goal);
        
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