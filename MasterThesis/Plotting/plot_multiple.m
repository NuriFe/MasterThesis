function plot_multiple(xout,uout,forces,tstep,tend,t,w_ref,model)
    figure()
    subplot(1,2,1)
    plot_wrist_positions(xout, model, w_ref);

    subplot(1,2,2)
    plot_hand_forces(forces',tstep,tend);

    figure()
    plot_neurexct(t,uout);
end