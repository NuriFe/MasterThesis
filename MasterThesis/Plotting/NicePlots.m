% Assuming all data (tsave, MuscleForces, actStep, FBForce, FBTorque, GoalLocation) is loaded into the workspace.

% Define figure and subplots
figure;
subplot(2, 2, 1); % First plot

% Plot tsave with MuscleForces for indices 1, 2, 5
plot(tsave, MuscleForces(:, 1), '-r', 'LineWidth', 2);
hold on;
plot(tsave, MuscleForces(:, 2), '-g', 'LineWidth', 2);
plot(tsave, MuscleForces(:, 5), '-b', 'LineWidth', 2);
ylim([-5 20]); % Set Y-limits for MuscleForces
legend('Triceps', 'Deltoids', 'Biceps'); % Add legend
title('Muscle Forces');
xlabel('Time (s)');
ylabel('Force (N)');
grid on;

subplot(2, 2, 2); % Second plot

% Plot tsave with actStep for indices 1, 2, 5
plot(tsave, actStep(:, 1), '-r', 'LineWidth', 2);
hold on;
plot(tsave, actStep(:, 2), '-g', 'LineWidth', 2);
plot(tsave, actStep(:, 5), '-b', 'LineWidth', 2);
ylim([0 1]); % Set Y-limits for actStep
legend('Triceps', 'Deltoids', 'Biceps'); % Add legend
title('Activation Step');
xlabel('Time (s)');
ylabel('Activation Value');
grid on;

subplot(2, 2, 3); % Third plot

% Plot FBForce
plot(tsave, FBForce, 'LineWidth', 2);
legend('Fx', 'Fy', 'Fz'); % Add legend for FBForce components
title('Feedback Force');
xlabel('Time (s)');
ylabel('Force (N)');
grid on;

subplot(2, 2, 4); % Fourth plot

% Plot FBTorque
plot(tsave, FBTorque, 'LineWidth', 2);
legend('Shoulder Elevation Plane', 'Shoulder Elevation', 'Shoulder Rotation', 'Elbow Flexion', 'Elbow Pronation'); % Add legend for FBTorque components
title('Feedback Torque');
xlabel('Time (s)');
ylabel('Torque (Nm)');
grid on;

% Adjust figure layout
sgtitle('Muscle Forces, Activation, Feedback Force, and Feedback Torque'); % Super title for the entire figure
