%% Initialise variables and model
% Some model related variables
clc;
clear;
modelparams = load('model_struct'); 
model = modelparams.model;

ndof = model.nDofs;
nmus = model.nMus;
nstates = 2*ndof + 2*nmus;

% Initialize the model
das3('Initialize',model);
% Define indices to the state variables within the state vector x
iq = 1:ndof;
iqdot = max(iq) + (1:ndof);
iLce = max(iqdot) + (1:nmus);

%% Insert time start, end and time step
tstep = .003;
tend = 3;
t = 0;
nsteps = round((tend-t)/tstep);

%% Create space for variables
tout = tstep*(0:nsteps)';

step_u = zeros(nmus,1);
step_xdot = zeros(nstates,1); 
M = zeros(5,1);
exF = zeros(2,1);

trials = 10;


% Set equilibrium state as initial state
load ('equilibrium.mat')
lengths = das3('Musclelengths',x);
LCEopt = das3('LCEopt');
SEEslack = das3('SEEslack');
x(iLce) = (lengths - SEEslack)./LCEopt;

muscles = 0;

% Input Function
u_total = stim_fun(muscles, tstep, tend);

%Reference Value
% [z,x,y]

% PID Control;
Kpx = 10;
Kpy = 10;
Kpz = 10;

Kix = 100;
Kiy = 100;
Kiz = 100;
    
w_refs = create_grid(0.1,model);
%keyboard
wrist_b = wrist_position(x);
progressBar = waitbar(0, 'Progress; 0%');
tocheck = [17 18 ];

%tocheck = [22,26];

for ref = 1:length(w_refs)
    if ~ismember(ref,tocheck)
        continue
    end
    w_ref = w_refs(ref,:)';
    progress = ref/length(w_refs);

    waitbar(progress, progressBar, sprintf('Progress: %.1f%%', progress * 100))

    error = zeros(trials,3);
    error_int = zeros(trials,3);
    
    %Initiliase variables
    err = [0 0 0]';
    ei = [0 0 0]';

    handF_total = zeros(trials,3);
    %handF = [2.3 5 1]';
    handF = [0 5 1.5]';
    j = 0;
    handF_total(1,:) = handF;

    while j<trials
        load('equilibrium.mat')
        stable = 0;
        t = 0;
        xout = x;
        i = 1;
        
        while stable <3
            u = u_total(:,i);
        
            % Advance simulation by a step
            [x, step_xdot, step_u,~,qTH] = das3step(x, u, tstep, step_xdot, step_u,M,exF,handF);
                
            % store result
            xout= [xout x];
            
            wrist_c = wrist_position(x);
            diff = roundn((wrist_c-wrist_b), -4);
            if isequal(diff,[0 0 0]')
                stable = stable +1;
                wrist_b = wrist_c;
            end
    
            t = t + tstep;
            i = i+1;
            
            if i>nsteps
                break
            end
        
        end
        j = j+1;
        w = wrist_position(x);
        err = w_ref - w;
        error(j,:)=err';
    
        dt = tstep;
        ei = ei + err*dt; 
    
        Fx = +Kix*ei(1,1)+Kpx*err(1,1)+handF(1,:);
        Fy = +Kiy*ei(2,1)+Kpy*err(2,1)+handF(2,:);
        Fz = handF(3,:)+Kiz*ei(3,1)+Kpz*err(3,1);
    
        handF = [Fx Fy Fz]';
        handF_total(j,:) = handF;
        %plot_wrist_positions(xout',model,w_ref);
        %break;
    end
    %break;
    [test_error,xout_testing] = testing(xout,handF_total,w_ref,model);
    %if any(abs(test_error)>0.04)
    %    test_error = string(roundn(test_error',-3));
    %    fprintf('Testing error %s %s %s\n ', test_error(1,1), test_error(1,2), test_error(1,3))
    %    figure()
    %    plot_wrist_positions(xout_testing,model,w_ref);
    %    disp(handF');
        %close(progressBar);

        %break
    %end
    point = string(ref);
    config = [qTH' x(10,1) x(11,1)]';

    name = append('C:\Users\s202421\Documents/Github/MasterThesis\DataCreation\modelpoints/',point );
    % Export to mat file and to opensim motion file
    save(name,'handF_total','error','xout','config');
    wrist = string(w_ref');
    wrist_error = string(roundn(err',-3));
    fprintf('Data saved for w_ref x:%s+%s y:%s+%s z:%s+%s\n ', ...
        wrist(1,1), wrist_error(1,1), wrist(1,2), wrist_error(1,2), wrist(1,3), wrist_error(1,3))
    test_error = string(roundn(test_error',-3));
    fprintf('Testing error %s %s %s\n ', test_error(1,1), test_error(1,2), test_error(1,3))
end
close(progressBar);
