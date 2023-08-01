function valid = valid_dynamics(x,muscle,handF)
    valid = true;
%% Initialise variables and model
    % Some model related variables
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
    t = 0;
    tstep = .003;
    tend = 3;
    nsteps = round((tend-t)/tstep);
    
    %% Create space for variables
    tout = tstep*(0:nsteps)';
    xout = zeros(nsteps+1, nstates);
    uout = zeros(nsteps+1, 138);
    mfout = zeros(nsteps+1, 138);
    qTHout = zeros(nsteps+1,3);

    xout(1,:) = x';

    % Initialize variables
    step_u = zeros(nmus,1);
    step_xdot = zeros(nstates,1); 
    
    %%
    warning('error', 'MATLAB:nearlysingularMatrix');
    M = zeros(5,1);
    exF = [ 0 0]';
    if nargin<2
        handF = [0 0 0]';
    end
    for i=1:(nsteps)
        u = stim_fun(t,muscle);
        try
            % Advance simulation by a step
            [x, step_xdot, step_u,~,qTH] = das3step(x, u, tstep, step_xdot, step_u, M, exF, handF);
            mf = das3('Muscleforces',x);
            xout(i+1,:) = x';   % store result
            uout(i+1,:) = u';
            mfout(i+1,:) = mf';
            qTHout(i+1,:)=qTH;
        
            t = t + tstep;
            %[~, lastWarning] = lastwarn;
        catch exception
             if strcmp(exception.identifier, 'MATLAB:nearlySingularMatrix')
                 valid = false;
                 fprintf('Singularity found. Simulation stopped after %.3f seconds \n', t);
                 break
             end
        end

    
    end
    plot_wrist_positions(xout,model);
end

%==========================================================
function [u] = stim_fun(t, muscles)
    nmus = 138;
    u = zeros(nmus,1);
    if muscles ~= 0
        if t > 0.25
            u(muscles) = 1;
        end 
        if t > 0.5
            u(muscles)=0;
        end
    end
end

