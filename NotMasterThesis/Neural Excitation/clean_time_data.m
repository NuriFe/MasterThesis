function [time_simple, act_simple] = clean_time_data(data_time, data_act)
    l = length(data_time);
    time = 0;
    time_simple = [];
    act_simple = [];
    same_time_actv = [];

    for i=1:l
        actv = data_act(i,:);
        if roundn(data_time(i,1),-4) == roundn(time,-4)
            same_time_actv = [same_time_actv; actv];
        else
            time_simple = [time_simple; time];
            mean_actv = mean(same_time_actv,1);
            act_simple = [act_simple; mean_actv];
            time = data_time(i,1);
            same_time_actv = [actv];
        end

    end
    time_simple = time_simple(2:end,:);

    
end