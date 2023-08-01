function result = term_calculation(type,value)
    if type == "error"
        a = 2;
    elseif type == "effort"
        a = 6;
    end
    Nm = 1;
    cumulative_error = 0;
    for i = 1:Nm
        value_int= integration(value);
        cummulatve_error = cummulatve_error + value_int;
    end

    result = sqrt(1/(a*T*Nm)*sum_int);

end
function sum_int = integration(value)
    value_s2 = value.^2;
    T = 2;
    time_step = 0.003;
    integral = trapz(0:time_step:(T-1)*time_step, value_s2(1));
    
    sum_int = sum(integral);

end