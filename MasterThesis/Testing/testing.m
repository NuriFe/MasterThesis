function [error,xout] = testing(xouts, fs,wref,muscle, tend, tstep)
    

    x = xouts(1,:)';
    q = x(1:11);
    [model, nstates, ndof, nmus, iLce] = initialize_model();
    x = initialize_state(q,nstates,iLce);
    warnCount = 0;
    complete = 0;
    while warnCount<=10 && complete ==0
            warning('')
            t = 0;
            tend = tend;
            tstep = tstep;
            tout = 0:tstep:tend;
            nsteps = round((tend-t)/tstep)+1; % +1 allows for saving of initial state

            % Initialize variables
            xdot = zeros(nstates,1);
            step_u = zeros(nmus,1);            
            xout = zeros(nsteps,nstates);
            forces = zeros(nsteps,3);

            % Set start and goal hand positions
            hand_start = wrist_position(x)';
            hand_goal = hand_start';

            % Set initial force hand
            handF = fs';

            % Save starting point
            xout(1,:)=x;
            uout(1,:)=zeros(nmus,1);

            % Run Simulation
            error_int = 0;
            for i=2:nsteps
                display_progress(i,nsteps);
                
                %Set neural excitation
                u = zeros(nmus,1);
                muscles = muscledict(muscle);
                u(muscles)=1;

                % Set moment and external force
                M = zeros(5,1);
                exF= zeros(2,1);

                holdTime=1000;
                %% PI Controller
                % K = eye(3)*2000;
                % I = 100;
                % %% PID 
                % [dPhand_dx, Phand] = pos_jacobian(x,model); % find position and velocity of the hand
                % B= 300;
                % Vhand=dPhand_dx*x(12:22);
                % 
                % hand_current = wrist_position(x);
                % error_pos = hand_goal-hand_current;
                % error_int = error_int + error_pos*tstep;
                % 
                % % PI calculation for time step
                % handF = K*error_pos+I*error_int;
                % Arm Support
                [dPhand_dx, Phand] = pos_jacobian(x,model);
                supportEq=[0;0.3;-0.15];
                K=diag([0 30 30]);
                B=diag([120 120 120]);
                Vhand=dPhand_dx*x(12:22);
                handF=-K*(Phand-supportEq)-B*Vhand;
                
                holdTime=166;
                if i<holdTime
                    Khold=eye(3)*2000;
                    Bhold=eye(3)*200;
                    holdF=-Khold*(Phand-hand_start')-Bhold*Vhand;
                    handF=handF+holdF;
                    
                    K=K+Khold;
                    B=B+Bhold;
                else

                end
                forces(1,:) = handF;

               try
                    % Advance simulation by a step
                    [x, xdot, step_u] = das3step(x, u, tstep, xdot, step_u, M, exF, handF);
                    
                    % Save data
                    xout(i,:)=x;
                    uout(i,:)=u;
                    forces(i,:) = handF;

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

            plot_multiple(xout,uout,forces,tstep,tend,tout,hand_goal,model)




    end
            
    x_final = xout(end,:)';
    w_final = wrist_position(x_final);
    error = wref-w_final;
    
    %plot_wrist_positions(xout,model,wref);
end