

% Step 2: Define the Constraints
lb = -2; % Lower bound for the controller position (x)
ub = 2;  % Upper bound for the controller position (x)

% Step 3: Run the Optimization using Simulated Annealing
options = optimoptions('simulannealbnd', 'Display', 'iter', 'MaxIterations', 100);

% Set custom annealing schedule in options
%options.AnnealingFcn = @customAnnealingSchedule;
%options.TemperatureFcn = @customAnnealingSchedule;
options.FunctionTolerance = 1e-6;
initial_guess = 0;
% Run the simulated annealing optimization
[optimal_position, optimal_cost] = simulannealbnd(@costfunction, initial_guess, lb, ub, options);

% Display the results
disp('Optimal Position:')
disp(optimal_position)
disp('Optimal Cost:')
disp(optimal_cost)

% Step 1: Define the Cost Function
function cost = myCostFunction(x)
    % Assume the target position is at x_target = 10
    x_target = 10;
    
    % Calculate the error term based on cumulative distance to the reaching target
    error_term = abs(x - x_target);
    
    % Calculate the effort term derived from the amount of muscle force used
    % Let's assume a linear relationship between distance and effort
    effort_term = 0.5 * error_term; % You can adjust the coefficient as per your requirements
    
    % Combine the error and effort terms using equal weights
    weight_error = 1;
    weight_effort = 1;
    cost = weight_error * error_term + weight_effort * effort_term;
end

% Custom annealing schedule function with 20% reduction factor

function cost = costfunction(G)

    % Target position
    target = 5455;%data.target;

    % Number of movements
    starts = 13451;%data.start_poses;
    Nm = length(starts);

    error = zeros(Nm,3);
    effort = zeros(Nm,9);
    for movement = 1:Nm
        % Simulate the reaching movement and calculate the error
        [error(movement,:),effort(movement,:)] = simulate_reaching_movement(G, target);

    end
    error_term = term_calculation("error",error);
    error_effort = term_calculation("effort",effort);

    cost = error_term + error_effort;

    end

function result = term_calculation(type,value)
    if type == "error"
        a = 2;
    elseif type == "effort"
        a = 6;
    end
    Nm = 12;
    cumulative_error = 0;
    for i = 1:Nm
        value_int= integration(value(i));
        cummulatve_error = cummulatve_error + value_int;
    end

    result = sqrt(1/(a*T*Nm)*sum_int);

end

function integration(value)
    value_s2 = value.^2;
    T = 2;
    time_step = 0.003;
    integral = trapz(0:time_step:(T-1)*time_step, value_s2);
    
    sum_int = sum(integral);

end