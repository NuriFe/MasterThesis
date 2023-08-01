function error = calculate_error(xout, wref, pos)
    x_final = xout(end,:)';
    w_final = wrist_position(x_final);
    error = wref-w_final;
    wrist = string(wref');
    wrist_error = string(roundn(error',-3));

    message = sprintf('Data for w_ref %s x:%s+%s y:%s+%s z:%s+%s\n ', ...
    string(pos), wrist(1,1), wrist_error(1,1), wrist(1,2), wrist_error(1,2), wrist(1,3), wrist_error(1,3));
    disp(message);
end
