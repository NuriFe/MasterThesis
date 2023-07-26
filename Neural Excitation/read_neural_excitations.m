function read_neural_excitations(points)
    % Read STO data
    for i=1
        try
            number = int2str(i);
            path =  strcat('C:\Users\s202421\Documents\GitHub\MasterThesis\OpenSim/point',number,'/');
            file = strcat('point',number,'_states.sto');
            %path = 'C:\Users\s202421\Documents\GitHub\MasterThesis\DataCreation\sto/';
            %file = '6.mat.sto';

            [data,C,tableData] =  readMOTSTOTRCfiles(path,file);
            fprintf('Read sto %s \n ', file)
    
            % Reorganise DATA
            act = 24:2:298;
            pos = 2:2:22;
            vel = 3:2:23;
            musclelength= 25:2:299;
            %states = 2:1:299;
            states = size(tableData,1);
            xout = zeros(states,298);
            xout(:,1:11) = data(:,pos);
            xout(:,12:22) = data(:,vel);
            xout(:,23:160)=data(:,act);
            xout(:,161:298) = data(:,musclelength);
            time = data(:,1);

            x_init=data(1,2:end);
            disp(wrist_position(x_init'));
            x_end = data(end,2:end);
            disp(wrist_position(x_end'));
            plot_neurexct(time,xout);

            [time, xout] = clean_time_data(time, xout);
            plot_neurexct(time,xout);
            %[time, xout] = fill_in_data(time, xout);
    
            %name = append('C:\Users\s202421\Documents/Github/MasterThesis\DataCreation\modelpoints/neuralexcitations/',number );
            name = append('C:\Users\s202421\Documents\GitHub\MasterThesis\DataCreation\excitations',number);
            % Export to mat file and to opensim motion file
            save(name,'x_end','xout','time');
            fprintf('Saved cleaned data  %s \n ', file)
        catch exception
            exception
            continue
        end
    end
end

