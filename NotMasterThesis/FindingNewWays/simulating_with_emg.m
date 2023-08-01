clear
close all
clc

% Initialize DAS model
[model, nstates, ndof, nmus, iLce] = initialize_model();
%load('C:\Users\s202421\Documents\GitHub\MasterThesis\DataCreation\modelpoints\neuralexcitations\1.mat')
load('C:\Users\s202421\Documents\GitHub\MasterThesis\DataCreation\excitations1.mat')
x_data = xout;
warning('error', 'MATLAB:nearlysingularMatrix');
%data = load('equilibrium.mat');
x_start = xout(1,:);
plot_wrist_positions(x_start,model);
x = zeros(298,1);
pos = x_start(:,1:11);
x(1:11,:)=pos;
CEE = x_start(:,23:160);
x(23:160,:) = CEE;
%x = initialize_state(pos, nstates, iLce);

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
t = 0;
warnCount = 0;
complete = 0;

% Excitation parameters
act_fill = x_data(:,161:end);
time_exct = time -2;
index = find_index_closest_value(time_exct,0);
u = act_fill(index,:)';
%u(u < 0.1) = 0;
%activation = [0.077; 0.1223; 0.1203];
%u = zeros(138,1);
%u for i=1:3
%    muscles = whichMuscle(i);
%    u(muscles) = activation(i);
%end
while complete == 0
    % Run Simulation
    for i=2:nsteps
        lastwarn('');
    
        display_progress(i,nsteps);
        
        %Set neural excitation
    
    
        % Set moment and external force
        M = zeros(5,1);
        exF= zeros(2,1);
        handF = [0;0;0];
    
        try
            % Advance simulation by a step
            [x, xdot, step_u] = das3step(x, u, tstep, xdot, step_u, M, exF, handF);
           
            % Save data
            forces(i,:)= handF;
            xout(i,:)=x;
            uout(i,:)=u;
            armTorque(i,:)=das3('Jointmoments',x);
            [warnMsg, warnId] = lastwarn;
    
            
            t = t +tstep;
            index = find_index_closest_value(time_exct,t);
            u = act_fill(index,:)';
            %u(u < 0.1) = 0;

            if nsteps == 500
                a = 1;
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
        x(1:11)=pos;
        x(iLce)=-3+6*rand(nmus,1); % randomly select initial Lce
    else
        complete=1;
    end
end

function index = find_index_closest_value(array, target_value)
    % array: The input array
    % target_value: The value for which we want to find the closest element
    
    % Compute the absolute difference between each element and the target value
    absolute_diff = abs(array - target_value);
    
    % Find the index of the element with the minimum absolute difference
    [~, index] = min(absolute_diff);
end
