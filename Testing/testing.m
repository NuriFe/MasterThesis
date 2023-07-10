function [error,xout] = testing(xouts, fs,wref,model)
    name='pop';
    %muscles = 109:1:115;
    muscles = 0;
    x = xouts(:,1);
    handF = fs(end,:);
    [xout,tout,uout] = neurext_handf(name, muscles,handF',x,3);
    x_final = xout(end,:)';
    w_final = wrist_position(x_final);
    error = wref-w_final;
    
    %plot_wrist_positions(xout,model,wref);
end