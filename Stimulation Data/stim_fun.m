function [u_total] = stim_fun(muscles, tstep, tend)
    nmus = 138;

    t = 0;
    nsteps = round((tend-t)/tstep);
    u_total = zeros(nmus,nsteps);
    for i=1:(nsteps)
        u = zeros(nmus,1);
        if muscles ~= 0
            if t > 0.25
                u(muscles) = 1;
            end 
            if t > 0.5
                u(muscles)=0;
            end
        end
        u_total(:,i)=u;
        t = t + tstep;
    end
end