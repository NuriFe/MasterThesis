% Load Positions
directory = 'C:\Users\s202421\Documents\GitHub\MasterThesis\Data\forces/PID';
fileList = dir(fullfile(directory, '*.mat'));
positions = zeros(18,11);
for i = 1:4:length(fileList)
    % Load the .mat file
    name1 = fileList(i).name;
    name2= fileList(i+1).name;
    name3 = fileList(i+2).name;
    name4 = fileList(i+3).name;

    filePath1 = fullfile(directory, name1);
    filePath2 = fullfile(directory, name2);
    filePath3 = fullfile(directory, name3);
    filePath4 = fullfile(directory, name4);

    data1 = load(filePath1);
    data2 = load(filePath2);
    data3 = load(filePath3);
    data4 = load(filePath4);

    xout1 = data1.xout;
    xout2 = data2.xout;
    xout3 = data3.xout;
    xout4 = data4.xout;

    error1 = wrist_position(xout1(end,:)')-wrist_position(xout1(1,:)');
    error2 = wrist_position(xout2(end,:)')-wrist_position(xout2(1,:)');
    error3 = wrist_position(xout3(end,:)')-wrist_position(xout3(1,:)');
    error4 = wrist_position(xout4(end,:)')-wrist_position(xout4(1,:)');

    % Create a figure and plot the data
    figure;
    subplot(2, 2, 1);
    plot_wrist_positions(xout1,model)
    text(1.5, 0.5, num2str(error1));
    title(name1);

    subplot(2, 2, 2);
    plot_wrist_positions(xout2,model)
    text(1.5, 0.5, num2str(error2));
    title(name2);

    subplot(2, 2, 3);
    plot_wrist_positions(xout3,model)
    text(1.5, 0.5, num2str(error3));
    title(name3);

    subplot(2, 2, 4);
    plot_wrist_positions(xout4,model)
    text(1.5, 0.5, num2str(error4));
    title(name4);
    
    keyboard();
    close all
end