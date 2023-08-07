function alpha = compute_neural_excitation(q,torque,alpha0 )
    if nargin <3
        alpha0 = zeros(8,1);
    end
    if nargin <2
        torque = predict_static_torque(q)';
    end
    R = predict_activation_torque(q);
    
    alpha = computeActivation_Nuri(R,torque,alpha0);


end