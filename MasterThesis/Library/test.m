function wrist_error = test(xout,force,wref,model, tend,tstep)
    name = '';
    muscles = [];
    [error,xout_test] = testing(xout, force,wref,name,muscles,model, tend, tstep);
    wrist = string(wref');
    wrist_error = string(roundn(error',-3));

    message = sprintf('Data for w_ref x:%s+%s y:%s+%s z:%s+%s\n ', ...
    wrist(1,1), wrist_error(1,1), wrist(1,2), wrist_error(1,2), wrist(1,3), wrist_error(1,3));
    
    if any(abs(error)>0.04)
        warning(message);
        %disp(fs(end,:));
        figure();
        plot_wrist_positions(xout_test,model,wref)
        %title(name);
    else
        fprintf(message);
    end
end