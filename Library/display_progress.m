function display_progress(i,nsteps)
    if round(1000*i/nsteps)/10 < 10
        fprintf('\b\b\b\b\b%3.1f%%\n', round(1000*i/nsteps)/10)
    elseif round(1000*i/nsteps)/10 < 100
        fprintf('\b\b\b\b\b\b%3.1f%%\n', round(1000*i/nsteps)/10)
    elseif round(1000*i/nsteps)/10 < 1000
        fprintf('\b\b\b\b\b\b\b%3.1f%%\n', round(1000*i/nsteps)/10)
    end
end