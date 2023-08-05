function alpha = compute_neural_excitation(q,alpha0)
    if nargin == 1
        alpha0 = zeros(8,1);
    end
    static_torque = predict_static_torque(q)';

    R = predict_activation_torque(q);
    
    alpha = computeActivation_Nuri(R,static_torque,alpha0);


end