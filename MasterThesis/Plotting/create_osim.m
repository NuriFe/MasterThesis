function create_osim(ndof, model, muscle_name, tout, xout)
    % Find DOF names
    dofnames = cell(ndof,1);
    for idof=1:ndof
        dofnames{idof} = model.dofs{idof}.osim_name;
    end
    
    name = append('C:\Users\s202421\Documents\GitHub\MasterThesis\MasterThesis\Data\Osim\', muscle_name);
    % Export to mat file and to opensim motion file
    make_osimm(name, dofnames, xout(:,1:ndof), tout);
    fprintf('Simulation result for %s has been saved.\n ',muscle_name)
end