function [handF_total,error,xout,qTH] = simulate_force_PID(nmus,nstates,x, handF, u_total, tstep,nsteps)
   
   
    step_u = zeros(nmus,1);
    step_xdot = zeros(nstates,1); 
    M = zeros(5,1);
    exF = zeros(2,1);
    
    trials = 20;

           
    modelparams = load('model_struct'); 
    model = modelparams.model;
    
    %Reference Value
    % [z,x,y]
    
    % PID Control;
    Kpx = 80;
    Kpy = 80;
    Kpz = 80;
    
    Kix = 300;
    Kiy = 300;
    Kiz = 300; 
    
    w_ref = wrist_position(x);  
    wrist_b =w_ref;
    error = zeros(trials,3);
    error_int = zeros(trials,3);
    
    %Initiliase variables
    err = [0 0 0]';
    ei = [0 0 0]';

    handF_total = zeros(trials,3);
    j = 1;
    handF_total(1,:) = handF;
    x_ref = x;
    warning('error', 'MATLAB:nearlysingularMatrix');

    while j<trials
        stable = 0;
        t = 0;
        x = x_ref;
        xout = x_ref;
        singularity = false;
        for i=1:(nsteps)
            try
                u = u_total(:,i);
            
                % Advance simulation by a step
                [x, step_xdot, step_u,~,qTH] = das3step(x, u, tstep, step_xdot, step_u,M,exF,handF);
                    
                % store result
                xout= [xout x];
        
                t = t + tstep;
            catch exception
                if strcmp(exception.identifier, 'MATLAB:nearlySingularMatrix')
                 %disp(t);
                 singularity = true;
                 break
                end
            end
   
        end

        if j ==19
            figure();
            plot_wrist_positions(xout',model,w_ref);
        end
        %plot_wrist_positions(xout',model,w_ref);
        
        if singularity == false
            j = j+1;
            w = wrist_position(x);
            err = w_ref - w;
            error(j,:)=err';
            dt = tstep;
            ei = ei + err*dt;
        else
            err = [ 0.01 0.01 0.001]';
            ei = [0 0 0]';
        end

    
        Fx = +Kix*ei(1,1)+Kpx*err(1,1)+handF(1,:);
        Fy = +Kiy*ei(2,1)+Kpy*err(2,1)+handF(2,:);
        Fz = +Kiz*ei(3,1)+Kpz*err(3,1)+handF(3,:);
    
        handF = [Fx Fy Fz]';
        handF_total(j,:) = handF;
    end
    %point = string(ref);
    %config = [qTH' x(10,1) x(11,1)]';
    % name = append('C:\Users\s202421\Documents/Github/MasterThesis\DataCreation\points/point',point );
    % Export to mat file and to opensim motion file
    %save(name,'handF_total','error','xout','config');
    %wrist = string(w_ref');
    %wrist_error = string(roundn(err',-3));
    %fprintf('Data saved for w_ref x:%s+%s y:%s+%s z:%s+%s\n ', ...
    %    wrist(1,1), wrist_error(1,1), wrist(1,2), wrist_error(1,2), wrist(1,3), wrist_error(1,3))

end

