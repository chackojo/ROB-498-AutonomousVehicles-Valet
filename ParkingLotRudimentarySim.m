% Set up the figure
figure('Position', [100, 100, 800, 600]);
hold on;
grid on;

% Define parking lot dimensions (in meters)
lot_width = 80;
lot_height = 60;

% Define parking spot dimensions
spot_length = 5;    % Length of a parking spot
spot_width = 2.5;   % Width of a parking spot
lane_width = 6;     % Width of driving lanes
entrance_width = 8; % Width of entrance/exit

% Calculate number of possible spots vertically
num_spots = floor((lot_height-2*lane_width) / spot_width);

% Calculate x-positions for columns and lanes
% Pattern: column | lane | column column | lane | column
col1_x = 0;                                    % First column
lane1_x = spot_length + lane_width/2;          % First lane
col2_x = lane1_x + lane_width/2;              % Second column
col3_x = col2_x + spot_length;                % Third column
lane2_x = col3_x + spot_length + lane_width/2; % Second lane
col4_x = lane2_x + lane_width/2;              % Fourth column
col5_x = col4_x + spot_length;                % Fifth column
lane3_x = col5_x + spot_length + lane_width/2; % Third lane
col6_x = lane3_x + lane_width/2;              % Sixth column
col7_x = col6_x + spot_length;                % Seventh column
lane4_x = col7_x + spot_length + lane_width/2; % Fourth lane
col8_x = lane4_x + lane_width/2;              % Eighth column
col9_x = col8_x + spot_length;                % Ninth column
lane5_x = col9_x + spot_length + lane_width/2; % Fifth lane
col10_x = lane5_x + lane_width/2;             % Tenth column

% Define column positions array
column_x = [col1_x, col2_x, col3_x, col4_x, col5_x, col6_x, col7_x, col8_x, col9_x, col10_x];
num_columns = length(column_x);  % Calculate number of columns

num_cars = 20;
car_states = struct([]);  % Array to track all cars
waiting_queue = [];       % Queue of cars waiting to enter
parked_cars = [];        % List of currently parked cars

% Initialize parking spot occupancy (start empty)
spot_assignments = zeros(num_spots, num_columns);  % Stores car IDs

% Initialize parking spot occupancy (random 70% occupancy)
parking_spots = zeros(num_spots, num_columns);  % 0 = empty, 1 = occupied
for i = 1:num_spots
    for j = 1:num_columns
        parking_spots(i,j) = rand() < 0.7;  % 70% chance of being occupied
    end
end

% Function to find nearest available spot
function [spot_pos, spot_i, spot_j] = find_nearest_spot(car_pos, parking_spots, column_x, lane_width, spot_width)
    [num_spots, num_columns] = size(parking_spots);
    min_cost = inf;
    spot_pos = [];
    spot_i = -1;
    spot_j = -1;
    
    for i = 1:num_spots
        for j = 1:num_columns
            if parking_spots(i,j) == 0
                y_pos = lane_width + (i-1)*spot_width;
                potential_pos = [column_x(j), y_pos];
                cost = norm(potential_pos - car_pos);
                
                if cost < min_cost
                    min_cost = cost;
                    spot_pos = potential_pos;
                    spot_i = i;
                    spot_j = j;
                end
            end
        end
    end
end

% Function to process car entry
function [car_states, waiting_queue] = process_car_entry(car_id, car_states, waiting_queue, entrance_width)
    new_car = struct('id', car_id, ...
                    'state', 'entering', ...
                    'position', [entrance_width/2, 1], ...
                    'path', [], ...
                    'target_spot', [], ...
                    'parking_time', now + rand()*1/24); % Random stay duration (0-1 hours)
    
    if isempty(car_states)
        car_states = new_car;
    else
        car_states(end+1) = new_car;
    end
    waiting_queue(end+1) = car_id;
end

% Main simulation loop
simulation_time = now;
end_time = simulation_time + 24/24; % 2 hours simulation
car_counter = 1;
dt = 1/24/60; % 1 minute time steps

while simulation_time < end_time
    % Add new cars with random arrival times
    if car_counter <= num_cars && rand() < 0.1 % 10% chance of new car each minute
        [car_states, waiting_queue] = process_car_entry(car_counter, car_states, waiting_queue, entrance_width);
        car_counter = car_counter + 1;
    end
    
    % Process each car
    for i = 1:length(car_states)
        if isempty(car_states(i).state)
            continue;
        end
        
        switch car_states(i).state
            case 'entering'
                if ~isempty(waiting_queue) && car_states(i).id == waiting_queue(1) % First in queue
                    % Find parking spot
                    [spot_pos, spot_i, spot_j] = find_nearest_spot(car_states(i).position, ...
                                                                 parking_spots, column_x, lane_width, spot_width);
                    if ~isempty(spot_pos)
                        car_states(i).target_spot = [spot_i, spot_j];
                        car_states(i).state = 'parking';
                        parking_spots(spot_i, spot_j) = 1;
                        spot_assignments(spot_i, spot_j) = car_states(i).id;
                        waiting_queue(1) = [];
                    end
                end
                
            case 'parking'
                % Move towards target spot
                target = [column_x(car_states(i).target_spot(2)), ...
                         lane_width + (car_states(i).target_spot(1)-1)*spot_width];
                dir = target - car_states(i).position;
                if norm(dir) < 0.1
                    car_states(i).state = 'parked';
                    car_states(i).position = target;
                else
                    car_states(i).position = car_states(i).position + ...
                                           0.1 * dir/norm(dir);
                end
                car_states(i).path(end+1,:) = car_states(i).position;
                
            case 'parked'
                % Check if it's time to leave
                if simulation_time >= car_states(i).parking_time
                    car_states(i).state = 'leaving';
                    spot_i = car_states(i).target_spot(1);
                    spot_j = car_states(i).target_spot(2);
                    parking_spots(spot_i, spot_j) = 0;
                    spot_assignments(spot_i, spot_j) = 0;
                end
                
            case 'leaving'
                % Move towards exit
                exit_pos = [entrance_width/2, 1];
                dir = exit_pos - car_states(i).position;
                if norm(dir) < 0.1
                    car_states(i).state = 'exited';
                else
                    car_states(i).position = car_states(i).position + ...
                                           0.1 * dir/norm(dir);
                end
                car_states(i).path(end+1,:) = car_states(i).position;
        end
    end
    
    % Update visualization
    clf;
    hold on;
    
    % Draw the outer boundary of the parking lot
    plot([0 lot_width lot_width 0 0], [0 0 lot_height lot_height 0], 'k-', 'LineWidth', 2);
    
    % Draw lanes
    plot([0 lot_width], [lane_width/2 lane_width/2], 'b--', 'LineWidth', 1.5);
    plot([0 lot_width], [lot_height-lane_width/2 lot_height-lane_width/2], 'b--', 'LineWidth', 1.5);
    plot([lane1_x lane1_x], [0 lot_height], 'b--', 'LineWidth', 1.5);
    plot([lane2_x lane2_x], [0 lot_height], 'b--', 'LineWidth', 1.5);
    plot([lane3_x lane3_x], [0 lot_height], 'b--', 'LineWidth', 1.5);
    plot([lane4_x lane4_x], [0 lot_height], 'b--', 'LineWidth', 1.5);
    plot([lane5_x lane5_x], [0 lot_height], 'b--', 'LineWidth', 1.5);
    
    % Draw parking spots with occupancy
    for spot = 0:num_spots-1
        y_base = lane_width + spot*spot_width;
        for col = 1:num_columns
            color = 'w';
            if parking_spots(spot+1, col) == 1
                color = [1 0.8 0.8];  % Light red for occupied
            end
            rectangle('Position', [column_x(col), y_base, spot_length, spot_width], ...
                     'EdgeColor', 'k', 'LineWidth', 1, 'FaceColor', color);
            
            % Add car ID text if spot is occupied
            if spot_assignments(spot+1, col) > 0
                text(column_x(col) + spot_length/2, y_base + spot_width/2, ...
                     num2str(spot_assignments(spot+1, col)), ...
                     'HorizontalAlignment', 'center');
            end
        end
    end
    
    % Draw entrance/exit
    rectangle('Position', [0, 0, entrance_width, 2], 'FaceColor', 'g', 'EdgeColor', 'g');
    text(entrance_width/2, -2, 'Entrance/Exit', 'HorizontalAlignment', 'center');
    
    % Draw cars and their paths
    for i = 1:length(car_states)
        if ~strcmp(car_states(i).state, 'exited')
            % Draw path
            if ~isempty(car_states(i).path)
                plot(car_states(i).path(:,1), car_states(i).path(:,2), ...
                     'r--', 'LineWidth', 1);
            end
            
            % Draw car
            plot(car_states(i).position(1), car_states(i).position(2), ...
                 'r*', 'MarkerSize', 10);
            text(car_states(i).position(1), car_states(i).position(2)-1, ...
                 num2str(car_states(i).id), 'Color', 'r');
        end
    end
    
    % Update axes and title
    axis([-5 lot_width+5 -5 lot_height+5]);
    xlabel('x [m]');
    ylabel('y [m]');
    title(sprintf('Multi-Car Valet Simulation - Time: %s', datestr(simulation_time, 'HH:MM')));
    axis equal;
    grid on;
    
    % Add legend
    legend('Lot Boundary', 'Driving Lanes', 'Car Path', 'Location', 'southeast');
    
    drawnow;
    pause(0.1);
    
    % Advance simulation time
    simulation_time = simulation_time + dt;
end