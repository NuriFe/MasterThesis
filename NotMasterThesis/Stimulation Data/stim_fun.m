function [u_total] = stim_fun(muscles, tstep, tend)
    nmus = 138;

    t = 0;
    nsteps = round((tend-t)/tstep);
    u_total = zeros(nmus,nsteps+1);
    for i=1:(nsteps+1)
        u = zeros(nmus,1);
        if muscles ~= 0
            u(muscles) = 1;
            %if t > 0.5
            %    u(muscles)=0;
            %end
        end
        u_total(:,i)=u;
        t = t + tstep;
    end
end