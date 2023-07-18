
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FindSingleM.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
close all
%clc

% Initialize the model
load model_struct
model.dofs{1}.range=[-0.419 -0.419];
model.dofs{2}.range=[0.0874 0.0874];
model.dofs{3}.range=[12 12]*pi/180;
model.dofs{4}.range=[34 34]*pi/180;
model.dofs{5}.range=[-20 -20]*pi/180;
model.dofs{6}.range=[-16 -16]*pi/180;
das3('Initialize',model);
disp('Done.');

warnidx=[]; % used to record warnings from MATLAB

% Initialize the model
ndof=11;
nmus=138;
nstates=2*ndof+2*nmus;

% indices to the state variables within the state vector
iq = 1:ndof;
iqdot = max(iq) + (1:ndof);
iLce = max(iqdot) + (1:nmus);
iAct = max(iLce) + (1:nmus);

% Load positions previously found
%load('Positions.mat')
% EXAMPLE POSITION FOR NURIA since she doesn't have Positions.mat
Positions(1,:)=[-0.419000000000000,0.0874000000000000,0.209439510239320,0.593411945678072,-0.349065850398866,-0.279252680319093,1.71899990059728,1.00567584900072,-0.476309534351695,0.210511043433220,1.90441457167920];
warnidx=[];

%% For loop to loop through multiple positions
for position=1 
    position
    
    % initialize variables
    Fsave=zeros(10,3);
    warn=0;
    
    %% Loop through muscles 1 at a time
    for muscle=1:10
        
        x=zeros(nstates,1);
        LCEopt=das3('LCEopt');
        muscle_tendon_lengths = das3('Musclelengths', x);    % only the first 11 elements of x (the joint angles) will be used
        slack_lengths = das3('SEEslack');
        Lce = muscle_tendon_lengths - slack_lengths;
        x(iLce)=Lce; % randomly select initial Lce
        x(1:11)=Positions(position,:);
        
        warnCount=0;
        complete=0;
        
        % EXIT loop if too many warnings
        while warnCount<=10 && complete==0
            warning('')
            tic;
            muscle
            % load passive equilibrium state x
            
            
            % Get initial and desired positions
            [~,stick]=pos_jacobian(x,model);
            Phand_start=stick';
            HandGoal=Phand_start';
            
            % model variables
            ndof=11;
            nmus=138;
            nstates=2*ndof+2*nmus;
            
            % Initialize derivatives and muscle excitations
            xdot = zeros(nstates,1);
            step_u = zeros(nmus,1);
            
            % Set simulation parameters
            t = 0;
            tend = 0.5;
            %if (nargin < 1)
            tstep = .001; % default was 0.003; anything bigger might yield inaccurate results
            % larger time step gets less "shaking" during arm movement
            %end
            nsteps = round((tend-t)/tstep)+1; % +1 allows for saving of initial state
            
            % Initialize variables for saving
            Fhand=zeros(nsteps,3);
            usave=zeros(nsteps,nmus);
            xsave=zeros(nsteps,length(x));
            HandLocation=zeros(nsteps,3);
            armTorque=zeros(nsteps,ndof);
            u=zeros(nmus,1);
            
            handF=[0;0;0];
            
            % Save starting point
            xsave(1,:)=x;
            usave(1,:)=u;
            [~,stick]=pos_jacobian(x,model);
            HandLocation(1,:)=stick';
            armTorque(1,:)=das3('Jointmoments',x);
            
            fprintf('\nSimulating...        ')
            sumErr=0;
            % Run simulation
            for i=2:nsteps
                u=zeros(nmus,1);
                muscles=whichMuscles(muscle);
                u(muscles)=1;
                
                % Display Progress
                if round(1000*i/nsteps)/10 < 10
                    fprintf('\b\b\b\b\b%3.1f%%\n', round(1000*i/nsteps)/10)
                elseif round(1000*i/nsteps)/10 < 100
                    fprintf('\b\b\b\b\b\b%3.1f%%\n', round(1000*i/nsteps)/10)
                elseif round(1000*i/nsteps)/10 < 1000
                    fprintf('\b\b\b\b\b\b\b%3.1f%%\n', round(1000*i/nsteps)/10)
                end
                
                M=zeros(5,1);
                %exF = [0.2;10];
                exF= [0;0];
                
                %% Adjust forces on end of wrist using a PID controller
                [dPhand_dx, Phand] = pos_jacobian(x,model); % find position and velocity of the hand
                K=eye(3)*2000; % Stiffness matrix (proportional gain)
                B=300; % Derivative term
                I=100; % Integration term
                
                Vhand=dPhand_dx*x(12:22);
                
                sumErr=sumErr+(HandGoal-Phand)*tstep;
                
                % PID caluclation for time step
                holdF=-K*(Phand-HandGoal)-B*Vhand+I*sumErr;
                
                % Arm Support
                supportEq=[0;0.3;-0.15];
                supportK=diag([0 30 30]);
                supportF=supportK*(supportEq-Phand);
                handF=holdF;%+supportF;
                
                Fhand(i,:)=holdF;
                % Advance simulation by a step
                [x, xdot, step_u] = das3step(x, u, tstep, xdot, step_u, M, exF, handF);%, K+supportK, B);
                
                xsave(i,:)=x;
                usave(i,:)=u;
                [~,stick]=pos_jacobian(x,model);
                HandLocation(i,:)=stick';
                armTorque(i,:)=das3('Jointmoments',x);
                
                [warnMsg, warnId] = lastwarn;
                if ~isempty(warnMsg)
                    warnCount=warnCount+1;
                    break;
                end
                
            end
            if ~isempty(warnMsg)
                x=zeros(nstates,1);
                x(1:11)=Positions(position,:);
                x(iLce)=-3+6*rand(nmus,1); % randomly select initial Lce
            else
                complete=1;
            end
            
        end
        
        endErr=HandLocation(1,:)-HandLocation(end,:)
        
        if warnCount>10
            warnidx=[warnidx position];
            warn=1;
        end
        
        toc
        
        
        
        tout=tstep*(1:nsteps);
        figure(1)
        plot(tout,HandLocation);

        figure(2)
        plot(tout,Fhand)

        drawnow
        
        Fsave=mean(Fhand(450:end,:));
        
        save(['Data/data',num2str(position),'_',num2str(muscle),'.mat'],'x','Fsave','warn','Fhand','xsave');
    end
    
    
end
save('Data/failedID','warnidx')