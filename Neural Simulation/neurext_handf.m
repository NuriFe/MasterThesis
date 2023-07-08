function [xout,tout] = neurext_handf(muscle_name, muscle,handF,x, tend, tstep)
    if nargin > 4
        tstep = .003;
    elseif nargin>3
        tstep = .003;
        tend = 1;
    elseif nargin > 2
        load('equilibrium.mat');
        tstep = .003;
        tend = 1;
    elseif nargin > 1
        load('equilibrium.mat');
        tstep = .003;
        tend = 1;
        handF = [0;0;0];

    end
    
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
    nsteps = round((tend-t)/tstep);
    
    %% Create space for variables
    tout = tstep*(0:nsteps)';
    xout = zeros(nsteps+1, nstates);
    uout = zeros(nsteps+1, 138);
    mfout = zeros(nsteps+1, 138);
    qTHout = zeros(nsteps+1,3);

    % Set equilibrium state as initial state
    lengths = das3('Musclelengths',x);
    LCEopt = das3('LCEopt');
    SEEslack = das3('SEEslack');
    x(iLce) = (lengths - SEEslack)./LCEopt;

    xout(1,:) = x';

    % Initialize variables
    step_u = zeros(nmus,1);
    step_xdot = zeros(nstates,1); 
    M = zeros(5,1);
    exF = zeros(2,1);
    %%
    for i=1:(nsteps)
        u = stim_fun(t,muscle);
        
        % Advance simulation by a step
        [x, step_xdot, step_u,~,qTH] = das3step(x, u, tstep, step_xdot, step_u,M,exF,handF);
        mf = das3('Muscleforces',x);
        xout(i+1,:) = x';   % store result
        uout(i+1,:) = u';
        mfout(i+1,:) = mf';
        qTHout(i+1,:)=qTH;
    
        t = t + tstep;
    
    end

    %plot_wrist_positions(xout,model);
    %figure()
    %plot_wrist_positions_2D(tout,xout);
    %create_osim(ndof, model, muscle_name, tout, xout, mfout);

end

%==========================================================
function [u] = stim_fun(t, muscles)
    nmus = 138;
    u = zeros(nmus,1);
    if muscles ~= 0
        if t > 3.00
            u(muscles) = 1;
        end 
        if t > 3.5
            u(muscles)=0;
        end
    end
end
%==========================================================
function [] = create_osim(ndof, model, muscle_name, tout, xout, mfout)
    % Find DOF names
    dofnames = cell(ndof,1);
    for idof=1:ndof
        dofnames{idof} = model.dofs{idof}.osim_name;
    end
    
    name = append('C:\Users\s202421\Desktop\DataCreation\', muscle_name);
    % Export to mat file and to opensim motion file
    save(name,'tout','xout', 'mfout');
    make_osimm(name, dofnames, xout(:,1:ndof), tout);
    fprintf('Simulation result for %s has been saved.\n ',muscle_name)
end