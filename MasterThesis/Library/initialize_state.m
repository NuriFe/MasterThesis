function [x] = initialize_state(q, nstates,iLce)
    x=zeros(nstates,1);
    x(1:11)=q;
    LCEopt=das3('LCEopt');
    muscle_tendon_lengths = das3('Musclelengths', x);    % only the first 11 elements of x (the joint angles) will be used
    slack_lengths = das3('SEEslack');
    Lce = muscle_tendon_lengths - slack_lengths;
    x(iLce)=Lce;
end