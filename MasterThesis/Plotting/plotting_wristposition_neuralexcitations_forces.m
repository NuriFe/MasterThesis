clear
clc
close all
directory = 'C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis/Data\Stimulated Forces';
fileList = dir(fullfile(directory, '*.mat'));
[model, nstates, ndof, nmus, iLce] = initialize_model();
muscle = 1;
for i = muscle:10:length(fileList)
    name = fileList(i).name;
    filePath = fullfile(directory, name);
    data = load(filePath);
    xout = data.xout;
    w_ref = wrist_position(data.x);
    forces = (data.forces)';
    tstep = 0.001;
    tend=0.5;
    t = 0:tstep:tend;
    figure()
    subplot(1,3,1)
    plot_wrist_positions(xout, model, w_ref);

    subplot(1,3,2)
    plot_hand_forces(forces,tstep,tend);

    subplot(1,3,3)
    plot_wrist_positions_2D(t,xout);

end