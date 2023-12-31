function [time_fill, act_fill] = fill_in_data(time_simple, act_simple)
    diff = 0.003;
    time_simple = roundn(time_simple,-4);
    i = 2;
    while i<=length(time_simple)
    
        % Calculate the initial difference between the first two numbers
        difference = time_simple(i,1) - time_simple(i-1,1);
        
        if roundn(diff,-4) ~= roundn(difference,-4)
            missing_value = time_simple(i-1)+diff;
            time_simple = [time_simple(1:i-1,1); missing_value; time_simple(i:end,1)];
            missing_data = act_simple(i-1,:);
            act_simple = [act_simple(1:i-1,:); missing_data; act_simple(i:end,:)];
            
        end
        i = i+1;
    end
    time_fill = time_simple;
    act_fill = act_simple;
end