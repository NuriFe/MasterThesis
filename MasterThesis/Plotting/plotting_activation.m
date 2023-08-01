directory = 'C:\Users\s202421\Documents\GitHub\MasterThesis\Data\neural_excitation';
fileList = dir(fullfile(directory, '*.mat'));
muscles = [1 2 5];
yrange = [0 1];
for i = 1:length(fileList)
    name = fileList(i).name;
    filePath = fullfile(directory, name);
    data = load(filePath);
    activation = data.actStep;

    figure()

    subplot(1,3,1)
    plot(activation(:,1));
    legend(whichMusclename(1));
    ylabel('neural excitation');
    ylim([0, 1]);

    subplot(1,3,2)
    plot(activation(:,2));
    legend(whichMusclename(2));
    ylabel('neural excitation');
    ylim([0, 1]);

    subplot(1,3,3)
    plot(activation(:,3));
    legend(whichMusclename(5));
    ylabel('neural excitation');
    ylim([0, 1]);

    title(name);
    keyboard
end

function name = whichMusclename(muscle)

switch muscle
    case 1
        name = "triceps";
    case 2
        name = "deltoids";
    case 3
        name = "lat dorsi";
    case 4
        name = "serratus anterior";
    case 5
        name = "biceps/brachialis";
    case 6
        name = "supra/infraspinatus";
    case 7
        name = "rhomboids";
    case 8
        name = "lower pectoralis";
    case 9 
        name = "upper pectoralis";
end

end