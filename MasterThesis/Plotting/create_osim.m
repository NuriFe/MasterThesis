function create_osim(ndof, model, muscle_name, tout, xout, mfout)
    % Find DOF names
    dofnames = cell(ndof,1);
    for idof=1:ndof
        dofnames{idof} = model.dofs{idof}.osim_name;
    end
    
    name = append('C:\Users\s202421\Desktop\DataCreation\', muscle_name);
    % Export to mat file and to opensim motion file
    save(name,'tout','xout', 'mfout');
    make_osimm(name, dofnames, xout(:,1:ndof), tout);
    fprintf('Simulation result for %s has been saved.\n ',muscle_name)
end