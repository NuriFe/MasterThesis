clear;
clc;

directory = 'C:\Users\s202421\Documents/GitHub/MasterThesis\DataCreation\modelpoints/neuralexcitations';
fileList = dir(fullfile(directory, '*.mat'));
w_refs = create_grid(0.1);
% Model Variables
% Some model related variables
modelparams = load('model_struct'); 
model = modelparams.model;
ndof = model.nDofs;
nmus = model.nMus;
nstates = 2*ndof + 2*nmus;
das3('Initialize',model);

for i = 1:length(fileList)
    % Load the .mat file
    name = fileList(i).name;
    filePath = fullfile(directory, name);
    %data = load(filePath);
    data = load('C:\Users\s202421\Documents\GitHub\MasterThesis\Neural Excitation\test/output1.mat');
    time = data.time;
    xout = data.xout;
    act = xout(:,161:298);
    %act = data.act_fill;

    %% Prepare for Simulation
    %Calculate time variables
    total_time = time(end,:)-time(1,:);
    tstep = time(2,:)-time(1,:);
    %tend = total_time;
    tend = 3;
    t = 0;
    nsteps = round((tend-t)/tstep);
    

    
    % Define indices to the state variables within the state vector x
    iq = 1:ndof;
    iqdot = max(iq) + (1:ndof);
    iLce = max(iqdot) + (1:nmus);
    
    % Set equilibrium state as initial state
    %load('equilibrium.mat')
    point = load('C:\Users\s202421\Documents\GitHub\MasterThesis\DataCreation\modelpoints\1.mat');
    xout = point.xout;
    handfs = point.handF_total;

    %load('equilibrium.mat')
    x = xout(:,1);
    %next = load('C:\Users\s202421\Documents\GitHub\MasterThesis\DataCreation\modelpoints\neuralexcitations\1.mat');
    %activation = act_fill(:,end-100:end)';
    %activation = next.act_fill';
    %data_mean = mean(activation);

    %x_init= data.x_end';
    %wrist_position(x_init)
    % Construct a state that should be close to static equilibrium
    %x= zeros(nstates,1);
    % Joint angles such that the arm hangs down
    %x(1:ndof) = x_init(1:ndof);
    %x(iLce) = 1.0;
    %lengths = das3('Musclelengths',x);
    %LCEopt = das3('LCEopt');
    %SEEslack = das3('SEEslack');
    %x(iLce) = (lengths - SEEslack)./LCEopt;

    
    % Initialize variables
    step_u = zeros(nmus,1);
    step_xdot = zeros(nstates,1); 
    M = zeros(5,1);
    exF = zeros(2,1);
    %handF = zeros(3,1);
    handF = -handfs(end,:)';
    % Create space for variables
    %tstep = 0.001;
    tout = tstep*(0:nsteps)';
    xout = zeros(nsteps+1, nstates);
    uout = zeros(nsteps+1, 138);
    xout(1,:) = x;
    
    for i=1:(nsteps)
        %u = data_mean';
        u = zeros(1,138)';
        %u = act(i,:)';
        % Advance simulation by a step
        [x, step_xdot, step_u,~,qTH] = das3step(x, u, tstep, step_xdot, step_u,M,exF,handF);
        mf = das3('Muscleforces',x);
        xout(i+1,:) = x';   % store result
        uout(i+1,:) = u';
        mfout(i+1,:) = mf';
        qTHout(i+1,:)=qTH;
    
        t = t + tstep;
    end
    
    %Extract wrist reference
    % Extract the number using regular expressions
    number = regexp(name, '\d+', 'match');
    % Convert the extracted number to a numeric value
    number = str2double(number);

    figure();
    plot_wrist_positions(xout,model,w_refs(number,:));

    plot_neurexct(tout,xout,uout);
    plot_neurexct(tout,xout);
end