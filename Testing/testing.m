function [error,xout] = testing(xouts, fs,wref,name,muscles,model)

    x = xouts(:,1);
    handF = fs(end,:);
    [xout,tout,uout] = neurext_handf(model,name, muscles,handF',x,1);
    x_final = xout(end,:)';
    w_final = wrist_position(x_final);
    error = wref-w_final;
    
    %plot_wrist_positions(xout,model,wref);
end