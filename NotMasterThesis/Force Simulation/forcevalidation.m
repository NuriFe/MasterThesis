function [x_valid, x_nonvalid]= forcevalidation(x)
    total_it = size(x,2);
    x_valid = [zeros(298,1)];
    x_nonvalid = [zeros(298,1)];
    progressBar = waitbar(0, 'Progress; 0%');

    for i=1:total_it
        progress = i/total_it;
        waitbar(progress, progressBar, sprintf('Progress: %.1f%%', progress * 100))
        x_test = x(:,i);
        valid = valid_dynamics(x_test,0);
        if valid
            x_valid = [x_valid x_test];
        else
            x_nonvalid = [x_nonvalid xtest];
        end
        
    end
    x_valid = x_valid(:, 2:end)';
    x_nonvalid = x_nonvalid(:,2:end)';
    close(progressBar);

end