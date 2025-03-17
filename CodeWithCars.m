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

% Define car dimensions
car_length = 4.5;
car_width = 1.8;

% Define wheel properties
wheel_diameter = 0.5;  % Wheel diameter in meters (50cm)
wheel_radius = wheel_diameter / 2;  % Wheel radius in meters
wheel_positions = [-car_length/3, -car_width/2;  % Rear left
                  car_length/3, -car_width/2;   % Front left
                  car_length/3, car_width/2;    % Front right
                  -car_length/3, car_width/2];  % Rear right

% PID controller parameters
pid_kp = 2.0;  % Proportional gain
pid_ki = 0.5;  % Integral gain
pid_kd = 0.1;  % Derivative gain
pid_max_output = 50;  % Maximum controller output (RPM)

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

% Define lane centers for path planning
lane_centers_y = [lane_width/2, lot_height-lane_width/2];
lane_centers_x = [lane1_x, lane2_x, lane3_x, lane4_x, lane5_x];

% Define lane centers for path planning
lane_centers_y = [lane_width/2, lot_height-lane_width/2];
lane_centers_x = [lane1_x, lane2_x, lane3_x, lane4_x, lane5_x];

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
function [spot_pos, spot_i, spot_j] = find_nearest_spot(car_pos, parking_spots, column_x, lane_width, spot_width, spot_length)
    [num_spots, num_columns] = size(parking_spots);
    min_cost = inf;
    spot_pos = [];
    spot_i = -1;
    spot_j = -1;
    
    for i = 1:num_spots
        for j = 1:num_columns
            if parking_spots(i,j) == 0
                y_pos = lane_width + (i-1)*spot_width + spot_width/2; % Center of spot
                potential_pos = [column_x(j) + spot_length/2, y_pos]; % Center of spot
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

% Function to calculate target wheel speeds based on car movement
function target_speeds = calculate_target_wheel_speeds(current_pos, prev_pos, orientation, dt, wheel_radius, wheel_positions)
    % Calculate car movement vector
    movement = current_pos - prev_pos;
    
    % Calculate car movement speed in m/s
    car_speed = norm(movement) / dt;
    
    % Convert orientation to radians
    orientation_rad = orientation * pi/180;
    
    % Calculate direction of movement
    if norm(movement) > 0
        movement_dir = movement / norm(movement);
    else
        movement_dir = [cos(orientation_rad), sin(orientation_rad)];
    end
    
    % Initialize target speeds array
    target_speeds = zeros(4, 1);
    
    % Calculate turning radius if car is turning
    % Get car heading vector
    heading = [cos(orientation_rad), sin(orientation_rad)];
    
    % Calculate cross product to determine if turning left or right
    cross_product = heading(1) * movement_dir(2) - heading(2) * movement_dir(1);
    
    % Calculate dot product to determine if moving forward or backward
    dot_product = heading(1) * movement_dir(1) + heading(2) * movement_dir(2);
    
    % Wheel speed differentials for turning (simplified model)
    wheel_speed_diff = 0;
    
    % If turning significantly
    if abs(cross_product) > 0.1 && dot_product > 0
        % Turning factor (positive for right turn, negative for left turn)
        wheel_speed_diff = cross_product * 10; % Scale factor for differential
    end
    
    % Calculate individual wheel speeds
    for i = 1:4
        % Base speed - all wheels get the same base speed
        base_speed = car_speed / wheel_radius;
        
        % Apply differential to left/right wheels when turning
        if i == 1 || i == 2  % Left wheels
            wheel_speed = base_speed - wheel_speed_diff;
        else  % Right wheels
            wheel_speed = base_speed + wheel_speed_diff;
        end
        
        % Ensure speed is not negative
        wheel_speed = max(0, wheel_speed);
        
        % Store the target speed
        target_speeds(i) = wheel_speed;
    end
    
    % Convert to RPM
    target_speeds = target_speeds * 60 / (2 * pi);
end

% PID controller for wheel speeds
function [wheel_speeds, pid_errors, pid_integrals] = apply_pid_control(target_speeds, current_speeds, pid_errors, pid_integrals, dt, pid_kp, pid_ki, pid_kd, pid_max_output)
    % Initialize output array
    wheel_speeds = zeros(4, 1);
    
    % Apply PID control to each wheel
    for i = 1:4
        % Calculate error
        error = target_speeds(i) - current_speeds(i);
        
        % Calculate integral term
        pid_integrals(i) = pid_integrals(i) + error * dt;
        
        % Calculate derivative term
        error_derivative = (error - pid_errors(i)) / dt;
        
        % Update error history
        pid_errors(i) = error;
        
        % Calculate PID output
        pid_output = pid_kp * error + pid_ki * pid_integrals(i) + pid_kd * error_derivative;
        
        % Apply limits to output
        pid_output = min(max(pid_output, -pid_max_output), pid_max_output);
        
        % Update wheel speed
        wheel_speeds(i) = current_speeds(i) + pid_output;
        
        % Ensure wheel speed is not negative
        wheel_speeds(i) = max(0, wheel_speeds(i));
    end
end

% Function to generate a path to a parking spot
function path = generate_path_to_spot(start_pos, target_spot, column_x, lane_centers_x, lane_centers_y, spot_width, lane_width, spot_length, entrance_width, num_spots)
    spot_j = target_spot(2);
    spot_i = target_spot(1);
    
    % Calculate target position (center of parking spot)
    target_x = column_x(spot_j) + spot_length/2;
    target_y = lane_width + (spot_i-1)*spot_width + spot_width/2;
    
    % Find nearest lane to the spot
    [~, nearest_lane_idx] = min(abs(lane_centers_x - target_x));
    nearest_lane_x = lane_centers_x(nearest_lane_idx);
    
    % Determine if spot is in upper or lower half of lot
    if target_y < (lane_width + num_spots*spot_width)/2
        nearest_lane_y = lane_centers_y(1); % Lower lane
    else
        nearest_lane_y = lane_centers_y(2); % Upper lane
    end
    
    % Define waypoints for path
    % 1. Start position
    % 2. Enter main lane from entrance
    % 3. Reach the nearest horizontal lane
    % 4. Reach the vertical lane nearest to spot
    % 5. Position in front of spot
    % 6. Final spot position
    
    waypoints = [
        start_pos;
        [entrance_width/2, lane_centers_y(1)]; % Enter main lower lane
        [nearest_lane_x, nearest_lane_y];      % Reach the lane near the spot
        [nearest_lane_x, target_y];            % Position in front of spot row
        [target_x, target_y]                   % Final spot position
    ];
    
    % Interpolate path between waypoints
    path = [];
    for i = 1:size(waypoints, 1)-1
        segment = interpolate_path(waypoints(i,:), waypoints(i+1,:), 10);
        path = [path; segment];
    end
end

% Helper function to interpolate between two points
function path = interpolate_path(start_point, end_point, num_points)
    t = linspace(0, 1, num_points)';
    path = start_point + t .* (end_point - start_point);
end

% Function to generate a path out of a parking spot
function path = generate_exit_path(spot_pos, target_spot, column_x, lane_centers_x, lane_centers_y, spot_width, lane_width, entrance_width, spot_length, num_spots)
    spot_j = target_spot(2);
    spot_i = target_spot(1);
    
    % Calculate starting position (center of parking spot)
    start_x = column_x(spot_j) + spot_length/2;
    start_y = lane_width + (spot_i-1)*spot_width + spot_width/2;
    
    % Find nearest lane to the spot
    [~, nearest_lane_idx] = min(abs(lane_centers_x - start_x));
    nearest_lane_x = lane_centers_x(nearest_lane_idx);
    
    % Determine if spot is in upper or lower half of lot
    if start_y < (lane_width + num_spots*spot_width)/2
        nearest_lane_y = lane_centers_y(1); % Lower lane
    else
        nearest_lane_y = lane_centers_y(2); % Upper lane
    end
    
    % Define waypoints for exit path
    % 1. Start in parking spot
    % 2. Back out to nearest vertical lane
    % 3. Drive to main horizontal lane
    % 4. Drive to exit
    
    waypoints = [
        [start_x, start_y];                   % Starting spot position
        [nearest_lane_x, start_y];            % Back out to lane
        [nearest_lane_x, lane_centers_y(1)];  % Drive to main lower lane
        [entrance_width/2, lane_centers_y(1)]; % Drive to exit position
        [entrance_width/2, 1]                 % Exit lot
    ];
    
    % Interpolate path between waypoints
    path = [];
    for i = 1:size(waypoints, 1)-1
        segment = interpolate_path(waypoints(i,:), waypoints(i+1,:), 10);
        path = [path; segment];
    end
end

% Function to process car entry
function [car_states, waiting_queue] = process_car_entry(car_id, car_states, waiting_queue, entrance_width)
    new_car = struct('id', car_id, ...
                    'state', 'entering', ...
                    'position', [entrance_width/2, 1], ...
                    'prev_position', [entrance_width/2, 1], ... % Add previous position
                    'orientation', 0, ...  % 0 degrees = facing north
                    'path', [], ...
                    'path_index', 1, ...
                    'target_spot', [], ...
                    'wheel_speeds', [0, 0, 0, 0], ... % Current wheel speeds
                    'target_wheel_speeds', [0, 0, 0, 0], ... % Target wheel speeds for PID
                    'pid_errors', [0, 0, 0, 0], ... % PID error history
                    'pid_integrals', [0, 0, 0, 0], ... % PID integral terms
                    'parking_time', now + (30 + rand()*30)/24/60); % Random stay duration (30-60 minutes)
    
    if isempty(car_states)
        car_states = new_car;
    else
        car_states(end+1) = new_car;
    end
    waiting_queue(end+1) = car_id;
end

% Calculate orientation based on movement direction
function orientation = calculate_orientation(pos1, pos2)
    dir = pos2 - pos1;
    orientation = atan2(dir(2), dir(1)) * 180/pi;
end

% Main simulation loop
simulation_time = now;
end_time = simulation_time + 5/24; % 1 hour simulation
car_counter = 1;
dt = 1/24/60/2; % 30 second time steps

while simulation_time < end_time
    % Add new cars with random arrival times
    if car_counter <= num_cars && rand() < 0.05 % 5% chance of new car each step
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
                                                                 parking_spots, column_x, lane_width, spot_width, spot_length);
                    
                    % Only proceed if an empty spot was found
                    if ~isempty(spot_pos) && spot_i > 0 && spot_j > 0
                        % Double check the spot is still empty before assigning
                        if parking_spots(spot_i, spot_j) == 0
                            car_states(i).target_spot = [spot_i, spot_j];
                            car_states(i).state = 'driving_to_spot';
                            car_states(i).path = generate_path_to_spot(car_states(i).position, ...
                                                                      car_states(i).target_spot, ...
                                                                      column_x, lane_centers_x, lane_centers_y, ...
                                                                      spot_width, lane_width, spot_length, entrance_width, num_spots);
                            car_states(i).path_index = 1;
                            
                            % Mark the spot as occupied
                            parking_spots(spot_i, spot_j) = 1;
                            spot_assignments(spot_i, spot_j) = car_states(i).id;
                            waiting_queue(1) = [];
                        end
                    end
                end
                
            case 'driving_to_spot'
                % Move along path to target spot
                if car_states(i).path_index < size(car_states(i).path, 1)
                    % Store previous position
                    car_states(i).prev_position = car_states(i).position;
                    
                    car_states(i).path_index = car_states(i).path_index + 1;
                    
                    % Calculate orientation based on movement direction
                    if car_states(i).path_index > 1
                        car_states(i).orientation = calculate_orientation(car_states(i).path(car_states(i).path_index-1,:), car_states(i).path(car_states(i).path_index,:));
                    end
                    
                    car_states(i).position = car_states(i).path(car_states(i).path_index,:);
                    
                    % Calculate target wheel speeds
                    car_states(i).target_wheel_speeds = calculate_target_wheel_speeds(car_states(i).position, car_states(i).prev_position, car_states(i).orientation, dt, wheel_radius, wheel_positions);
                    
                    % Apply PID control
                    [car_states(i).wheel_speeds, car_states(i).pid_errors, car_states(i).pid_integrals] = apply_pid_control(car_states(i).target_wheel_speeds, car_states(i).wheel_speeds, car_states(i).pid_errors, car_states(i).pid_integrals, dt, pid_kp, pid_ki, pid_kd, pid_max_output);
                else
                    car_states(i).state = 'parked';
                    % Final position is center of parking spot
                    spot_i = car_states(i).target_spot(1);
                    spot_j = car_states(i).target_spot(2);
                    car_states(i).position = [column_x(spot_j) + spot_length/2, lane_width + (spot_i-1)*spot_width + spot_width/2];
                    car_states(i).orientation = 90; % Facing east in the spot
                    car_states(i).wheel_speeds = [0, 0, 0, 0]; % Set wheel speeds to zero when parked
                end
                
            case 'parked'
                % Check if it's time to leave
                if simulation_time >= car_states(i).parking_time
                    car_states(i).state = 'leaving_spot';
                    car_states(i).path = generate_exit_path(car_states(i).position, ...
                                                           car_states(i).target_spot, ...
                                                           column_x, lane_centers_x, lane_centers_y, ...
                                                           spot_width, lane_width, entrance_width, spot_length, num_spots);
                    car_states(i).path_index = 1;
                    
                    % Update spot status
                    spot_i = car_states(i).target_spot(1);
                    spot_j = car_states(i).target_spot(2);
                    parking_spots(spot_i, spot_j) = 0;
                    spot_assignments(spot_i, spot_j) = 0;
                end
                
            case 'leaving_spot'
                % Move along exit path
                if car_states(i).path_index < size(car_states(i).path, 1)
                    % Store previous position
                    car_states(i).prev_position = car_states(i).position;
                    
                    car_states(i).path_index = car_states(i).path_index + 1;
                    
                    % Calculate orientation based on movement direction
                    if car_states(i).path_index > 1
                        car_states(i).orientation = calculate_orientation(car_states(i).path(car_states(i).path_index-1,:), car_states(i).path(car_states(i).path_index,:));
                    end
                    
                    car_states(i).position = car_states(i).path(car_states(i).path_index,:);
                    
                    % Calculate target wheel speeds
                    car_states(i).target_wheel_speeds = calculate_target_wheel_speeds(car_states(i).position, car_states(i).prev_position, car_states(i).orientation, dt, wheel_radius, wheel_positions);
                    
                    % Apply PID control
                    [car_states(i).wheel_speeds, car_states(i).pid_errors, car_states(i).pid_integrals] = apply_pid_control(car_states(i).target_wheel_speeds, car_states(i).wheel_speeds, car_states(i).pid_errors, car_states(i).pid_integrals, dt, pid_kp, pid_ki, pid_kd, pid_max_output);
                else
                    car_states(i).state = 'exited';
                end
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
        end
    end
    
    % Draw entrance/exit
    rectangle('Position', [0, 0, entrance_width, 2], 'FaceColor', 'g', 'EdgeColor', 'g');
    text(entrance_width/2, -2, 'Entrance/Exit', 'HorizontalAlignment', 'center');
    
    % Draw cars
    for i = 1:length(car_states)
        if ~strcmp(car_states(i).state, 'exited')
            % Draw car using rectangle
            car_x = car_states(i).position(1) - car_length/2;
            car_y = car_states(i).position(2) - car_width/2;
            
            % Draw car as a rectangle with proper orientation
            rotation_center = car_states(i).position;
            
            % Colors based on car state
            switch car_states(i).state
                case 'entering'
                    car_color = [0, 0.7, 0]; % Green
                case 'driving_to_spot'
                    car_color = [0, 0, 0.8]; % Blue
                case 'parked'
                    car_color = [0.8, 0, 0]; % Red
                case 'leaving_spot'
                    car_color = [0.8, 0.5, 0]; % Orange
                otherwise
                    car_color = [0.5, 0.5, 0.5]; % Gray
            end
            
            % Create a rectangle patch that we can rotate
            angle_rad = car_states(i).orientation * pi/180;
            
            % Define car corners relative to center
            corners_x = [-car_length/2, car_length/2, car_length/2, -car_length/2];
            corners_y = [-car_width/2, -car_width/2, car_width/2, car_width/2];
            
            % Rotate corners
            rotated_x = corners_x * cos(angle_rad) - corners_y * sin(angle_rad);
            rotated_y = corners_x * sin(angle_rad) + corners_y * cos(angle_rad);
            
            % Translate to car position
            rotated_x = rotated_x + car_states(i).position(1);
            rotated_y = rotated_y + car_states(i).position(2);
            
            % Draw car as a patch
            patch(rotated_x, rotated_y, car_color, 'EdgeColor', 'k', 'LineWidth', 1);
            
            % Add car ID text
            text(car_states(i).position(1), car_states(i).position(2), ...
                 num2str(car_states(i).id), 'Color', 'w', 'FontWeight', 'bold', ...
                 'HorizontalAlignment', 'center');
            
            % Draw the wheels and their speeds
            if strcmp(car_states(i).state, 'driving_to_spot') || strcmp(car_states(i).state, 'leaving_spot')
                angle_rad = car_states(i).orientation * pi/180;
                
                for w = 1:4
                    % Calculate wheel position in car coordinates
                    wheel_x = wheel_positions(w,1);
                    wheel_y = wheel_positions(w,2);
                    
                    % Rotate to match car orientation
                    rotated_x = wheel_x * cos(angle_rad) - wheel_y * sin(angle_rad);
                    rotated_y = wheel_x * sin(angle_rad) + wheel_y * cos(angle_rad);
                    
                    % Translate to car position
                    wheel_pos_x = rotated_x + car_states(i).position(1);
                    wheel_pos_y = rotated_y + car_states(i).position(2);
                    
                    % Draw small circle for wheel
                    plot(wheel_pos_x, wheel_pos_y, 'ko', 'MarkerSize', 3, 'MarkerFaceColor', 'k');
                    
                    % Display wheel speed
                    if car_states(i).wheel_speeds(w) > 0
                        % Format speed to 1 decimal place
                        speed_text = sprintf('%.1f', car_states(i).wheel_speeds(w));
                        
                        % Determine text color based on difference between target and actual speed
                        target = car_states(i).target_wheel_speeds(w);
                        actual = car_states(i).wheel_speeds(w);
                        speed_diff = abs(target - actual);
                        
                        % Use color coding to show PID control status
                        if speed_diff < 1.0
                            text_color = 'g'; % Green for good tracking
                        elseif speed_diff < 5.0
                            text_color = 'b'; % Blue for medium tracking
                        else
                            text_color = 'r'; % Red for poor tracking
                        end
                        
                        text(wheel_pos_x + 0.2, wheel_pos_y + 0.2, speed_text, 'FontSize', 7, 'Color', text_color);
                    end
                end
            else
                % For parked cars, show 0 RPM wheels
                angle_rad = car_states(i).orientation * pi/180;
                
                for w = 1:4
                    % Calculate wheel position in car coordinates
                    wheel_x = wheel_positions(w,1);
                    wheel_y = wheel_positions(w,2);
                    
                    % Rotate to match car orientation
                    rotated_x = wheel_x * cos(angle_rad) - wheel_y * sin(angle_rad);
                    rotated_y = wheel_x * sin(angle_rad) + wheel_y * cos(angle_rad);
                    
                    % Translate to car position
                    wheel_pos_x = rotated_x + car_states(i).position(1);
                    wheel_pos_y = rotated_y + car_states(i).position(2);
                    
                    % Draw small circle for wheel
                    plot(wheel_pos_x, wheel_pos_y, 'ko', 'MarkerSize', 3, 'MarkerFaceColor', 'k');
                    
                    % Display 0 RPM for parked cars
                    if strcmp(car_states(i).state, 'parked')
                        text(wheel_pos_x + 0.2, wheel_pos_y + 0.2, '0.0', 'FontSize', 7);
                    end
                end
            end
            
            % Draw path if car is moving
            if strcmp(car_states(i).state, 'driving_to_spot') || strcmp(car_states(i).state, 'leaving_spot')
                % Calculate remaining path
                remaining_path = car_states(i).path(car_states(i).path_index:end, :);
                if ~isempty(remaining_path)
                    plot(remaining_path(:,1), remaining_path(:,2), '--', 'Color', [0.7, 0.7, 0.7], 'LineWidth', 1);
                end
            end
        end
    end
    
    % Update axes and title
    axis([-5 lot_width+5 -5 lot_height+5]);
    xlabel('x [m]');
    ylabel('y [m]');
    % Count active cars (not exited)
    active_cars = 0;
    for i = 1:length(car_states)
        if ~isempty(car_states(i).state) && ~strcmp(car_states(i).state, 'exited')
            active_cars = active_cars + 1;
        end
    end
    
    title_str = sprintf('Parking Lot Simulation - Time: %s\nCars: %d/%d (Active: %d, Waiting: %d)', ...
                       datestr(simulation_time, 'HH:MM'), ...
                       car_counter-1, num_cars, ...
                       active_cars, ...
                       length(waiting_queue));
    title(title_str);
    axis equal;
    grid on;
    
    % Add legend
    legend('Lot Boundary', 'Driving Lanes', 'Location', 'southeast');
    
    drawnow;
    pause(0.1);
    
    % Advance simulation time
    simulation_time = simulation_time + dt;
end