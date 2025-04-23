% --- Simulation Setup ---
clear; clc; close all;

% Set up the figure and UI controls
fig = figure('Position', [100, 100, 900, 700]);
ax = axes('Parent', fig, 'Position', [0.1, 0.15, 0.8, 0.75]);
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');

% --- Parameters ---
% Parking Garage Dimensions
num_levels = 3;
lot_width = 80; % meters
lot_height = 60; % meters
level_height = 5; % Visual separation for 3D plot (if used)

% Parking Spot & Lane Dimensions
spot_length = 5;    % Length of a parking spot
spot_width = 2.5;   % Width of a parking spot
lane_width = 6;     % Width of driving lanes
entrance_width = 8; % Width of entrance/exit

% Car Dimensions
car_length = 4.5;
car_width = 1.8;
car_safety_radius = max(car_length, car_width) / 2 + 0.5; % For collision check

% Obstacle Properties
num_obstacles = 1; % Number of random obstacles (pedestrians)
obstacle_radius = 0.5; % Radius of obstacles
obstacle_safety_margin = 0.3; % Extra space around obstacles

% Simulation Time
simulation_start_time = now; % Use MATLAB's date number format
simulation_duration_hours = 8; % Simulate an 8-hour workday
end_time = simulation_start_time + simulation_duration_hours/24;
dt_seconds = 10; % Time step in seconds (keep this for simulation logic)
dt = dt_seconds / (24 * 3600); % Time step in days (for 'now')
current_time = simulation_start_time;

% Car Properties & Control
max_cars_total = 700; % Increased limit to accommodate pre-parked cars + new arrivals
initial_fill_percentage = 0.90; % Initialize 90% full
car_arrival_probability = 0.03; % Chance of a new car arriving each time step (dynamic arrivals)
min_stay_hours = 1; % Min stay for both pre-parked and new cars
max_stay_hours = 8; % Max stay for both pre-parked and new cars

% Calculate number of spots and column positions (remains the same per level)
num_spots_per_row = floor((lot_height - 2*lane_width) / spot_width);
col1_x = 0;
lane1_x = spot_length + lane_width/2;
col2_x = lane1_x + lane_width/2;
col3_x = col2_x + spot_length;
lane2_x = col3_x + spot_length + lane_width/2;
col4_x = lane2_x + lane_width/2;
col5_x = col4_x + spot_length;
lane3_x = col5_x + spot_length + lane_width/2;
col6_x = lane3_x + lane_width/2;
col7_x = col6_x + spot_length;
lane4_x = col7_x + spot_length + lane_width/2;
col8_x = lane4_x + lane_width/2;
col9_x = col8_x + spot_length;
lane5_x = col9_x + spot_length + lane_width/2;
col10_x = lane5_x + lane_width/2;

column_x = [col1_x, col2_x, col3_x, col4_x, col5_x, col6_x, col7_x, col8_x, col9_x, col10_x];
num_columns = length(column_x);
lane_centers_x = [lane1_x, lane2_x, lane3_x, lane4_x, lane5_x];
lane_centers_y = [lane_width/2, lot_height - lane_width/2]; % Lower and Upper main lanes

% Define Ramp Zones (simple areas for level transition)
ramp_zone_x = [entrance_width, entrance_width + lane_width]; % Near entrance
ramp_zone_y_lower = [0, lane_width]; % Lower lane ramp area
ramp_zone_y_upper = [lot_height - lane_width, lot_height]; % Upper lane ramp area (less used here)

% --- State Initialization ---
car_states = struct('id', {}, 'level', {}, 'state', {}, 'position', {}, ...
                   'orientation', {}, 'path', {}, 'path_index', {}, ...
                   'target_spot', {}, 'parking_time_leave', {}, ...
                   'collision_check_timer', {}); % Added timer to prevent rapid state flip
parking_spots = zeros(num_spots_per_row, num_columns, num_levels); % 0=empty, 1=occupied
spot_assignments = zeros(num_spots_per_row, num_columns, num_levels); % Stores car IDs
obstacles = struct('id', {}, 'position', {}, 'radius', {}, 'level', {}); % Obstacles per level
car_counter = 0; % Counter for dynamically arriving cars (positive IDs)
preparked_car_counter = 0; % Counter for pre-parked cars (negative IDs)
active_cars_list = []; % Keep track of active cars by ID

% --- Initialize Pre-Parked Cars (90% full) ---
disp('Initializing pre-parked cars...');
for level = 1:num_levels
    for r = 1:num_spots_per_row
        for c = 1:num_columns
            if rand() < initial_fill_percentage
                preparked_car_counter = preparked_car_counter - 1; % Use negative IDs

                % Calculate spot center position
                spot_center_y = lane_width + (r-1)*spot_width + spot_width/2;
                spot_center_x = column_x(c) + spot_length/2;

                % Create pre-parked car state
                pre_parked_car = struct(...
                    'id', preparked_car_counter, ...
                    'level', level, ...
                    'state', 'parking', ... % Start in 'parking' state
                    'position', [spot_center_x, spot_center_y], ... % Positioned in the spot
                    'orientation', 90, ...  % Parked facing east (standard orientation)
                    'path', [], ...
                    'path_index', 1, ...
                    'target_spot', [r, c, level], ... % Store its own spot info
                    'parking_time_leave', current_time + (min_stay_hours + rand()*(max_stay_hours-min_stay_hours))/24, ... % Random leave time within simulation duration
                    'collision_check_timer', 0);

                if length(car_states) < max_cars_total
                    car_states(end+1) = pre_parked_car;
                    parking_spots(r, c, level) = 1; % Mark spot occupied
                    spot_assignments(r, c, level) = pre_parked_car.id; % Assign negative ID
                    active_cars_list(end+1) = pre_parked_car.id; % Add to active list
                else
                    warning('Maximum car limit reached during pre-parking initialization.');
                    break; % Stop adding cars if limit reached
                end
            end
        end
         if length(car_states) >= max_cars_total, break; end
    end
     if length(car_states) >= max_cars_total, break; end
end
fprintf('Initialized %d pre-parked cars.\n', abs(preparked_car_counter));


% --- UI Control ---
current_level_display = 1;
level_slider = uicontrol('Parent', fig, 'Style', 'slider', ...
    'Min', 1, 'Max', num_levels, 'Value', current_level_display, ...
    'Position', [150, 10, 300, 20], ...
    'Callback', @(src, ~) update_level_display(src), ... % Simplified callback
    'SliderStep', [1/(num_levels-1), 1/(num_levels-1)]);
uicontrol('Parent', fig, 'Style', 'text', 'Position', [100, 10, 50, 20], 'String', 'Level:');
level_text = uicontrol('Parent', fig, 'Style', 'text', 'Position', [460, 10, 30, 20], 'String', num2str(current_level_display));


% --- Main Simulation Loop ---
while current_time < end_time
    % Update level based on slider value (read directly inside loop)
    current_level_display = round(level_slider.Value);
    level_text.String = num2str(current_level_display);

    % 1. Generate New Cars (Dynamic Arrivals)
   % 1. Generate New Cars (Dynamic Arrivals)
    allow_generation = false; % Default to not allowing generation

    % Initial checks: max cars and probability
    if length(car_states) < max_cars_total && rand() < car_arrival_probability
        % Check condition based on the last generated car's state
        last_car_parked = true; % Assume parked if no relevant car found
        positive_ids = [car_states([car_states.id] > 0).id]; % Get IDs of dynamically generated cars

        if ~isempty(positive_ids)
            last_car_id = max(positive_ids);
            last_car_idx = find([car_states.id] == last_car_id, 1);

            if ~isempty(last_car_idx)
                last_car_state = car_states(last_car_idx).state;

                % Block generation IF the last car is NOT in the 'parking' state
                % (or 'exited', 'exiting' which imply it has parked and left)
                if ~ismember(last_car_state, {'parking', 'leaving_spot', 'exiting', 'exited'})
                    last_car_parked = false;
                     % fprintf('Skipped generation: Last car %d has not parked (state: %s)\n', last_car_id, last_car_state); % Debug
                end
            end
        end

        % Allow generation only if the last car has parked (or left)
        if last_car_parked
            allow_generation = true;
        end
    end

    % Generate the new car ONLY if conditions are met and last car parked/left
    if allow_generation
        car_counter = car_counter + 1; % Positive IDs for new cars
        new_car = struct(...
            'id', car_counter, ...
            'level', 1, ...
            'state', 'entering', ...
            'position', [entrance_width/2, 1], ... % Start just outside
            'orientation', 90, ...
            'path', [], ...
            'path_index', 1, ...
            'target_spot', [], ...
            'parking_time_leave', current_time + (min_stay_hours + rand()*(max_stay_hours-min_stay_hours))/24, ...
            'collision_check_timer', 0);
        car_states(end+1) = new_car;
        active_cars_list(end+1) = new_car.id;
         % fprintf('Generated Car %d\n', car_counter); % Optional Debug
    end

    % 2. Generate/Update Obstacles for all levels
    % (Consider optimizing if num_obstacles is very large)
    obstacles = [];
    for level = 1:num_levels
        for k = 1:num_obstacles
             % Place obstacles randomly in lane areas (same logic as before)
             obs_x = rand() * lot_width;
             obs_y = rand() * lot_height;
             is_in_lane = false;
             if (obs_y > lane_width && obs_y < lot_height - lane_width) || ...
                (obs_x > spot_length && obs_x < lane1_x + lane_width/2) || ...
                (obs_x > col3_x+spot_length && obs_x < lane2_x + lane_width/2) || ...
                (obs_x > col5_x+spot_length && obs_x < lane3_x + lane_width/2) || ...
                (obs_x > col7_x+spot_length && obs_x < lane4_x + lane_width/2) || ...
                (obs_x > col9_x+spot_length && obs_x < lane5_x + lane_width/2) || ...
                (obs_x > col10_x+spot_length && obs_x < lot_width)
                 is_in_lane = true;
             end

             if is_in_lane
                 obstacles(end+1).id = k + (level-1)*num_obstacles;
                 obstacles(end).position = [obs_x, obs_y];
                 obstacles(end).radius = obstacle_radius;
                 obstacles(end).level = level;
             end
        end
    end


    % 3. Update Car States
    cars_to_remove_indices = []; % Store indices for removal
    for i = 1:length(car_states)
        car_id = car_states(i).id;

        % Reduce collision check timer
        if car_states(i).collision_check_timer > 0
            car_states(i).collision_check_timer = car_states(i).collision_check_timer - dt;
        end

        % --- State Machine Logic (largely unchanged, except for 'parking' state start) ---
        switch car_states(i).state
            case 'entering'
                entry_target_y = lane_centers_y(1);
                entry_point_pos = [entrance_width/2, entry_target_y];
                entry_point_tolerance = car_width; % How close another car needs to be to block
    
                % Check if the entry point itself is blocked by another active car
                is_blocked = false;
                for j = 1:length(car_states)
                    if i == j, continue; end % Skip self
                    other_car = car_states(j);
                    % Check if another car is near the entry point AND is not yet driving/parking/leaving
                    if other_car.level == car_states(i).level && ...
                       norm(other_car.position - entry_point_pos) < entry_point_tolerance && ...
                       (strcmp(other_car.state, 'entering') || strcmp(other_car.state, 'finding_spot') || strcmp(other_car.state, 'waiting'))
                        is_blocked = true;
                        break;
                    end
                end
    
                % Only move if entry point is not blocked
                if ~is_blocked
                    move_speed_entering = 6; % Use the increased speed
                    current_y = car_states(i).position(2);
                    move_dist = move_speed_entering * dt * 3600;
    
                    % Move towards target Y, but don't overshoot
                    if current_y + move_dist >= entry_target_y
                        car_states(i).position(2) = entry_target_y; % Arrived at entry point
                        car_states(i).position(1) = entrance_width/2; % Ensure X is correct
                        car_states(i).state = 'finding_spot'; % Transition to find spot
                         % Optional: Add diagnostic print here if needed
                         fprintf('Car %d: Reached entry point. State -> finding_spot\n', car_id);
                    else
                        car_states(i).position(2) = current_y + move_dist; % Move forward
                         % Optional: Add diagnostic print here if needed
                         fprintf('Car %d: Moving towards entry point. Y: %.1f\n', car_id, car_states(i).position(2)); 
                    end
                else
                     % Entry blocked, do nothing this step (effectively waits)
                     % Optional: Add diagnostic print here if needed
                     fprintf('Car %d: Entry point blocked, waiting.\n', car_id);
                end

            case 'finding_spot'
                %[spot_pos, spot_r, spot_c, spot_l] = find_nearest_spot(car_states(i).position, car_states(i).level, parking_spots, column_x, lane_width, spot_width, spot_length, num_levels, lot_height, lot_width); % Added lot_width
                %[spot_pos, spot_r, spot_c, spot_l] = find_nearest_spot(car_states(i).position, car_states(i).level, parking_spots, column_x, lane_width, spot_width, spot_length, num_levels, lot_height, lot_width, num_spots_per_row); % Added num_spots_per_row
                [spot_pos, spot_r, spot_c, spot_l] = find_nearest_spot(car_states(i).position, car_states(i).level, parking_spots, column_x, lane_width, spot_width, spot_length, num_levels, lot_height, lot_width, num_spots_per_row, num_columns); % Added num_columns
                if ~isempty(spot_pos)
                    % Double-check spot availability before assigning
                    if parking_spots(spot_r, spot_c, spot_l) == 0
                        car_states(i).target_spot = [spot_r, spot_c, spot_l];
                        %car_states(i).path = generate_path_to_spot(car_states(i).position, car_states(i).level, car_states(i).target_spot, column_x, lane_centers_x, lane_centers_y, spot_width, lane_width, spot_length, entrance_width, num_spots_per_row, ramp_zone_x, ramp_zone_y_lower);
                        car_states(i).path = generate_path_to_spot(car_states(i).position, car_states(i).level, car_states(i).target_spot, column_x, lane_centers_x, lane_centers_y, spot_width, lane_width, spot_length, entrance_width, num_spots_per_row, ramp_zone_x, ramp_zone_y_lower, car_width); % Added car_width
                        car_states(i).path_index = 1;
                        car_states(i).state = 'driving_to_spot';
                         % --- ADDED PRINT ---
                         fprintf('Car %d: Found spot [%d, %d, %d]. Path generated (length %d). State -> driving_to_spot\n', car_id, spot_r, spot_c, spot_l, size(car_states(i).path, 1));
                        % --- END ADDED PRINT ---
                        parking_spots(spot_r, spot_c, spot_l) = 1; % Mark occupied
                        spot_assignments(spot_r, spot_c, spot_l) = car_id;
                    else
                         % Spot taken between finding and assigning, wait and retry
                         car_states(i).state = 'waiting';
                         car_states(i).collision_check_timer = 1*dt; % Wait briefly before finding again
                    end
                else
                    car_states(i).state = 'waiting'; % Wait if no spot found
                end

            case 'driving_to_spot'
                if isempty(car_states(i).path) || car_states(i).path_index > size(car_states(i).path, 1)
                    car_states(i).state = 'parking'; % Arrived at spot
                    % Final position adjustment
                    if ~isempty(car_states(i).target_spot)
                        spot_r = car_states(i).target_spot(1);
                        spot_c = car_states(i).target_spot(2);
                        spot_l = car_states(i).target_spot(3);
                        car_states(i).position = [column_x(spot_c) + spot_length/2, lane_width + (spot_r-1)*spot_width + spot_width/2];
                        car_states(i).orientation = 90;
                        car_states(i).level = spot_l;
                    end
                    continue; % Skip rest of logic for this car
                end
    
                % Check for level change (ramp) - Move this check before movement calc? Seems okay here.
                 current_pos = car_states(i).position;
                 target_level = car_states(i).target_spot(3);
                 if car_states(i).level ~= target_level
                     in_ramp_zone = current_pos(1) >= ramp_zone_x(1) && current_pos(1) <= ramp_zone_x(2) && ...
                                    current_pos(2) >= ramp_zone_y_lower(1) && current_pos(2) <= ramp_zone_y_lower(2);
                     if in_ramp_zone
                          path_segment_level = car_states(i).path(car_states(i).path_index, 3);
                          if path_segment_level == target_level
                              car_states(i).level = target_level;
                              car_states(i).state = 'waiting';
                              car_states(i).collision_check_timer = 0.5*dt_seconds / (24 * 3600);
                              level_slider.Value = target_level; % Update slider
                              continue; % Skip movement this step
                          end
                     end
                 end
    
                % --- Calculate Potential Next Position FIRST ---
                next_path_point = car_states(i).path(car_states(i).path_index, 1:2);
                move_vector = next_path_point - car_states(i).position;
                move_dist = norm(move_vector);
                move_speed_driving = 16; % Increased Speed (m/s)
                max_move = move_speed_driving * dt * 3600;
    
                potential_next_pos = car_states(i).position; % Default to current if no move needed
                if move_dist > 0
                    potential_next_pos = car_states(i).position + min(1, max_move / move_dist) * move_vector;
                end
                % --- End Calculation ---
    
                % --- Perform Collision Checks using potential_next_pos ---
                [car_obs_collision, ~] = check_collision(car_id, car_states(i).level, potential_next_pos, car_safety_radius, car_states, obstacles, obstacle_safety_margin, car_length, car_width);
                spot_collision = is_pos_in_occupied_spot(potential_next_pos, car_states(i).level, car_id, car_states(i).state, car_states(i).target_spot, parking_spots, spot_assignments, column_x, spot_length, spot_width, lane_width, num_spots_per_row, num_columns);
                collision_detected = car_obs_collision || spot_collision;
                % --- End Collision Checks ---
    
                % --- Decide Action Based on Collision ---
                if collision_detected && car_states(i).collision_check_timer <= 0
                     car_states(i).state = 'waiting';
                     % Use standard wait time if blocked by spot or car/obstacle
                     car_states(i).collision_check_timer = 1 * dt_seconds / (24*3600); % Wait 1 sec real time
                     % Optional Debug Prints
                     % if car_id == 1
                     if spot_collision, fprintf('Car %d: Spot collision detected! State -> waiting\n', car_id);
                     else, fprintf('Car %d: Car/Obstacle collision detected! State -> waiting\n', car_id); end
                     % end
                else % No collision or timer active -> Proceed with movement
                    % If previously waiting, resume state (but don't move yet if timer just expired)
                    if strcmp(car_states(i).state, 'waiting')
                        car_states(i).state = 'driving_to_spot';
                    end
    
                    % Move car along path if timer allows
                    if car_states(i).collision_check_timer <= 0
                        if move_dist > 0
                            car_states(i).orientation = atan2d(move_vector(2), move_vector(1));
                            if move_dist <= max_move
                                % Move to the exact path point
                                car_states(i).position = next_path_point;
                                car_states(i).path_index = car_states(i).path_index + 1;
                                fprintf('Car %d: Reached path point. Path Index: %d/%d\n', car_id, car_states(i).path_index, size(car_states(i).path, 1)); % Debug
                            else
                                % Move partially along the vector
                                car_states(i).position = car_states(i).position + (move_vector / move_dist) * max_move;
                                fprintf('Car %d: Moving along path. Pos: [%.1f, %.1f], Path Index: %d/%d\n', car_id, car_states(i).position(1), car_states(i).position(2), car_states(i).path_index, size(car_states(i).path, 1));
                            end
                        else % Already at the path point (move_dist == 0)
                             car_states(i).path_index = car_states(i).path_index + 1;
                             fprintf('Car %d: Reached path point. Path Index: %d/%d\n', car_id, car_states(i).path_index, size(car_states(i).path, 1)); % Debug
                        end
                    end % End check for collision timer <= 0 before moving
                end % End Action Decision

            case 'leaving_spot'
                 if isempty(car_states(i).path) || car_states(i).path_index > size(car_states(i).path, 1)
                     car_states(i).state = 'exiting'; % Path finished
                     continue; % Skip rest of logic
                 end
    
                 % Check for level change (ramp)
                 current_pos = car_states(i).position;
                 target_level = 1; % Exit always goes to level 1 eventually
                 if car_states(i).level ~= target_level
                     in_ramp_zone = current_pos(1) >= ramp_zone_x(1) && current_pos(1) <= ramp_zone_x(2) && ...
                                    current_pos(2) >= ramp_zone_y_lower(1) && current_pos(2) <= ramp_zone_y_lower(2);
                    if in_ramp_zone
                         path_segment_level = car_states(i).path(car_states(i).path_index, 3);
                         if path_segment_level == target_level % Path indicates transition
                            car_states(i).level = target_level;
                            car_states(i).state = 'waiting';
                            car_states(i).collision_check_timer = 0.5*dt_seconds / (24 * 3600); % Wait 0.5 sec real time
                            level_slider.Value = target_level; % Update slider
                            continue; % Skip movement this step
                         end
                     end
                 end
    
                 % --- Calculate Potential Next Position FIRST ---
                 next_path_point = car_states(i).path(car_states(i).path_index, 1:2);
                 move_vector = next_path_point - car_states(i).position;
                 move_dist = norm(move_vector);
                 move_speed_leaving = 16; % Increased Speed (m/s)
                 max_move = move_speed_leaving * dt * 3600;
    
                 potential_next_pos = car_states(i).position; % Default to current if no move needed
                 if move_dist > 0
                    potential_next_pos = car_states(i).position + min(1, max_move / move_dist) * move_vector;
                 end
                 % --- End Calculation ---
    
                 % --- Perform Collision Checks using potential_next_pos ---
                 [car_obs_collision, ~] = check_collision(car_id, car_states(i).level, potential_next_pos, car_safety_radius, car_states, obstacles, obstacle_safety_margin, car_length, car_width);
                 % Pass car's original spot info (target_spot) for context in the check
                 spot_collision = is_pos_in_occupied_spot(potential_next_pos, car_states(i).level, car_id, car_states(i).state, car_states(i).target_spot, parking_spots, spot_assignments, column_x, spot_length, spot_width, lane_width, num_spots_per_row, num_columns);
                 collision_detected = car_obs_collision || spot_collision;
                 % --- End Collision Checks ---
    
                 % --- Decide Action Based on Collision ---
                 if collision_detected && car_states(i).collision_check_timer <= 0
                     car_states(i).state = 'waiting';
                     car_states(i).collision_check_timer = 1 * dt_seconds / (24*3600); % Wait 1 sec real time
                 else % No collision or timer active -> Proceed with movement
                     % If previously waiting, resume state
                     if strcmp(car_states(i).state, 'waiting')
                         car_states(i).state = 'leaving_spot';
                     end
    
                     % Move car along path if timer allows
                     if car_states(i).collision_check_timer <= 0
                        if move_dist > 0
                            car_states(i).orientation = atan2d(move_vector(2), move_vector(1));
                            if move_dist <= max_move
                                car_states(i).position = next_path_point;
                                car_states(i).path_index = car_states(i).path_index + 1;
                            else
                                car_states(i).position = car_states(i).position + (move_vector / move_dist) * max_move;
                            end
                        else % Already at the path point
                            car_states(i).path_index = car_states(i).path_index + 1;
                        end
                    end % End check for collision timer <= 0 before moving
                 end % End Action Decision

            case 'waiting'
             % --- DIAGNOSTIC PRINT (Optional) ---
              if car_states(i).collision_check_timer > 0
                 fprintf('Car %d: Entering waiting state (Timer: %.2f)\n', car_id, car_states(i).collision_check_timer * 24*3600);
              end
             % --- END DIAGNOSTIC PRINT ---

             % Check if condition causing wait is resolved ONLY if timer expired
             if car_states(i).collision_check_timer <= 0
                 current_pos = car_states(i).position; % Check safety at current position

                 % Call MODIFIED collision check
                 [collision, blocking_agent_id] = check_collision(car_id, car_states(i).level, current_pos, car_safety_radius, car_states, obstacles, obstacle_safety_margin, car_length, car_width);

                 if ~collision
                     % --- Obstruction Cleared ---
                     % Determine previous state to return to
                     if ~isempty(car_states(i).target_spot) && spot_assignments(car_states(i).target_spot(1), car_states(i).target_spot(2), car_states(i).target_spot(3)) == car_id
                         % Check if path still exists
                         if ~isempty(car_states(i).path) && car_states(i).path_index <= size(car_states(i).path,1)
                             car_states(i).state = 'driving_to_spot';
                             % Add small random delay to stagger resume
                             car_states(i).collision_check_timer = rand() * 0.5 * dt_seconds / (24*3600); % Stagger resume (0-0.5s)
                         else % Path lost or finished? Re-evaluate
                             dist_to_target = norm(car_states(i).position - [column_x(car_states(i).target_spot(2)) + spot_length/2, lane_width + (car_states(i).target_spot(1)-1)*spot_width + spot_width/2]);
                             if dist_to_target < spot_length / 2
                                 car_states(i).state = 'parking'; % Park if close enough
                             else
                                 car_states(i).state = 'finding_spot'; % Re-plan if path lost/invalid
                                 spot_assignments(car_states(i).target_spot(1), car_states(i).target_spot(2), car_states(i).target_spot(3)) = 0;
                                 parking_spots(car_states(i).target_spot(1), car_states(i).target_spot(2), car_states(i).target_spot(3)) = 0;
                                 car_states(i).target_spot = [];
                             end
                         end
                     elseif ~isempty(car_states(i).path) % If path exists, assume leaving
                         car_states(i).state = 'leaving_spot';
                         % Add small random delay to stagger resume
                         car_states(i).collision_check_timer = rand() * 0.5 * dt_seconds / (24*3600); % Stagger resume (0-0.5s)
                     else % No target spot, no path
                         car_states(i).state = 'finding_spot'; % Try finding spot again if stuck entering
                     end
                     % --- End Obstruction Cleared ---
                 else
                     % --- Still Blocked: Implement Deadlock Check ---
                     standard_wait_time = 1 * dt_seconds / (24*3600); % Standard 1 sec wait
                     longer_wait_base = 2 * dt_seconds / (24*3600); % Base 2 sec wait for yielding
                     random_yield_add = rand() * 2 * dt_seconds / (24*3600); % Add 0-2 sec random for yielding

                     if blocking_agent_id > 0 % Blocked by another car
                         % Find the blocking car's state
                         blocker_idx = find([car_states.id] == blocking_agent_id, 1);
                         if ~isempty(blocker_idx) && strcmp(car_states(blocker_idx).state, 'waiting')
                             % --- Deadlock Detected: Both are waiting ---
                             if car_id > blocking_agent_id
                                 % This car has higher ID, yields (waits longer)
                                 car_states(i).collision_check_timer = longer_wait_base + random_yield_add;
                                 % fprintf('Car %d: Deadlock with %d. Yielding (waiting longer).\n', car_id, blocking_agent_id); % Debug
                             else
                                 % This car has lower ID, standard wait (other car should yield)
                                 car_states(i).collision_check_timer = standard_wait_time;
                                 % fprintf('Car %d: Deadlock with %d. Standard wait.\n', car_id, blocking_agent_id); % Debug
                             end
                         else
                             % Blocked by a car, but it's not waiting (e.g., driving), standard wait
                             car_states(i).collision_check_timer = standard_wait_time;
                         end
                     else
                         % Blocked by obstacle (blocking_agent_id < 0) or unknown (0)
                         car_states(i).collision_check_timer = standard_wait_time;
                     end
                     % --- End Deadlock Check ---
                 end
             end

            case 'exiting'
                % Move car out of the lot
                move_speed_exiting = 20; % m/s
                car_states(i).position(2) = car_states(i).position(2) - move_speed_exiting * dt * 3600;
                if car_states(i).position(2) < -car_length % Car is fully out
                    car_states(i).state = 'exited';
                    cars_to_remove_indices(end+1) = i; % Mark car for removal by index
                end
        end
    end

    % Remove exited cars from state and active list
    if ~isempty(cars_to_remove_indices)
       ids_to_remove = [car_states(cars_to_remove_indices).id];
       car_states(cars_to_remove_indices) = []; % Remove by index
       active_cars_list = setdiff(active_cars_list, ids_to_remove, 'stable'); % Remove by ID
    end

    % 4. Update Visualization
    cla(ax);
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    axis(ax, [-10 lot_width+10 -10 lot_height+10]);
    xlabel(ax, 'X (m)');
    ylabel(ax, 'Y (m)');
    num_active_cars = length(active_cars_list); % Count based on active list
    title(ax, sprintf('Parking Garage Level %d - Time: %s - Cars Active: %d', current_level_display, datestr(current_time, 'HH:MM:SS'), num_active_cars));

    % Draw parking spots for the current level
    for r = 1:num_spots_per_row
        y_base = lane_width + (r-1)*spot_width;
        for c = 1:num_columns
            spot_state = parking_spots(r, c, current_level_display);
            car_in_spot_id = spot_assignments(r,c,current_level_display);
            color = 'w'; % Empty

            if spot_state == 1 % Occupied or Reserved
                 car_idx = find([car_states.id] == car_in_spot_id, 1);
                 if ~isempty(car_idx)
                     if strcmp(car_states(car_idx).state, 'parking')
                        color = [1 0.7 0.7]; % Parked - Light Red
                     else
                        color = [0.8 0.8 0.8]; % Reserved (driving to) - Gray
                     end
                 else
                     color = [0.9 0.9 0.9]; % Occupied but car state lost? (Error indication)
                 end
            end
            rectangle(ax, 'Position', [column_x(c), y_base, spot_length, spot_width], ...
                      'EdgeColor', 'k', 'LineWidth', 0.5, 'FaceColor', color);
             % Optional: Display Car ID in occupied spots
             % if car_in_spot_id ~= 0
             %     text(ax, column_x(c)+spot_length/2, y_base+spot_width/2, num2str(car_in_spot_id), ...
             %         'HorizontalAlignment','center', 'VerticalAlignment','middle', 'FontSize', 7, 'Color', 'k');
             % end
        end
    end

     % Draw Lane Markings (Simplified)
     plot(ax, [0 lot_width], [lane_centers_y(1) lane_centers_y(1)], 'b:', 'LineWidth', 1); % Lower lane center
     plot(ax, [0 lot_width], [lane_centers_y(2) lane_centers_y(2)], 'b:', 'LineWidth', 1); % Upper lane center
     for lx = lane_centers_x
         plot(ax, [lx lx], [0 lot_height], 'b:', 'LineWidth', 1); % Vertical lane centers
     end

     % Draw Ramp Zones (visual cue on all levels for orientation)
     rectangle(ax, 'Position', [ramp_zone_x(1), ramp_zone_y_lower(1), diff(ramp_zone_x), diff(ramp_zone_y_lower)], 'EdgeColor', 'm', 'LineStyle',':', 'LineWidth', 1.5);
     text(ax, mean(ramp_zone_x), mean(ramp_zone_y_lower), 'Ramp', 'Color', 'm', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');

    % Draw Obstacles for the current level
    for k = 1:length(obstacles)
        if obstacles(k).level == current_level_display
             center = obstacles(k).position;
             radius = obstacles(k).radius;
             rectangle(ax, 'Position', [center(1)-radius, center(2)-radius, 2*radius, 2*radius], ...
                      'Curvature', [1 1], 'FaceColor', [0.6 0.6 0.1], 'EdgeColor', 'none'); % Dark Yellow circles
        end
    end

    % Draw Cars for the current level
    for i = 1:length(car_states)
        if car_states(i).level == current_level_display
            draw_car(ax, car_states(i), car_length, car_width);
             % Draw path if driving
             if (strcmp(car_states(i).state, 'driving_to_spot') || strcmp(car_states(i).state, 'leaving_spot')) && ~isempty(car_states(i).path)
                 path_indices_on_level = find(car_states(i).path(:, 3) == current_level_display);
                 valid_path_indices = path_indices_on_level(path_indices_on_level >= car_states(i).path_index);
                 if ~isempty(valid_path_indices)
                      plot(ax, car_states(i).path(valid_path_indices, 1), car_states(i).path(valid_path_indices, 2), 'g:', 'LineWidth', 1);
                 end
             end
        end
    end

    % Draw Entrance/Exit marker ONLY on Level 1
    if current_level_display == 1
        rectangle(ax, 'Position', [0, -5, entrance_width, 5], 'FaceColor', [0.2 0.8 0.2], 'EdgeColor', 'none');
        text(ax, entrance_width/2, -2.5, 'Entry/Exit', 'HorizontalAlignment', 'center', 'Color', 'w');
    end


    drawnow limitrate; % Update plot efficiently
    % SLOW DOWN SIMULATION by increasing pause duration
    pause(0.01); % Changed from 0.01 to 0.1 seconds pause per frame

    % 5. Advance Time
    current_time = current_time + dt;
end

disp('Simulation finished.');

% --- Helper Functions ---

% Function to Update Level Display (Callback for Slider) - Simplified
function update_level_display(slider_obj)
    level = round(slider_obj.Value);
    % Find the text object next to the slider
    parent_fig = slider_obj.Parent;
    text_obj = findobj(parent_fig, 'Style', 'text', 'String', 'Level:');
    level_val_text = findobj(parent_fig, 'Style', 'text', '-regexp', 'Position', '\[460,.*');
    if ~isempty(level_val_text)
        level_val_text.String = num2str(level);
    end
    % Redrawing happens in main loop, no need to update 'ax' here
end


% Function to Find Nearest Available Spot (Multi-Level)
function [spot_pos, spot_r, spot_c, spot_l] = find_nearest_spot(car_pos, car_level, parking_spots, column_x, lane_width, spot_width, spot_length, num_levels, lot_height, lot_width, num_spots_per_row, num_columns) % Ensure all args are here

    % Initialize variables to store the best spot found
    min_cost = inf; % Initialize min_cost HERE, before any loops start using it
    spot_pos = [];
    spot_r = -1; % Row
    spot_c = -1; % Column
    spot_l = -1; % Level

    % Search starting from the car's current level, then others
    level_search_order = [car_level, setdiff(1:num_levels, car_level)];

    for level = level_search_order
        for r = 1:num_spots_per_row
            for c = 1:num_columns
                % Check if spot is empty (strict 0 check)
                if parking_spots(r, c, level) == 0
                    % Calculate spot center position
                    y_pos = lane_width + (r-1)*spot_width + spot_width/2;
                    x_pos = column_x(c) + spot_length/2;
                    potential_pos = [x_pos, y_pos];

                    % Cost function: Euclidean distance + penalty for changing levels
                    dist_cost = norm(potential_pos - car_pos);
                    level_change_cost = abs(level - car_level) * (lot_height + lot_width); % Heuristic cost for level change
                    cost = dist_cost + level_change_cost;

                    % Compare cost with the current minimum cost found so far
                    if cost < min_cost % Check against the initialized min_cost
                        min_cost = cost; % Update min_cost if this spot is better
                        spot_pos = potential_pos;
                        spot_r = r;
                        spot_c = c;
                        spot_l = level;
                    end
                end
            end
        end
         % Optional optimization: If a spot is found on the current level, stop searching further levels
         if ~isempty(spot_pos) && level == car_level
              break; % Prioritize spots on the same level if found
         end
    end
    % Return the details of the best spot found (or empty/negative values if none)
end

% Function to Generate Path to Spot (Multi-Level) - Unchanged
% Function to Generate Path to Spot (Multi-Level) - MODIFIED Entry Maneuver
%function path = generate_path_to_spot(start_pos, start_level, target_spot_info, column_x, lane_centers_x, lane_centers_y, spot_width, lane_width, spot_length, entrance_width, num_spots_per_row, ramp_zone_x, ramp_zone_y_lower)
    target_r = target_spot_info(1);
    target_c = target_spot_info(2);
    target_l = target_spot_info(3);

    target_x = column_x(target_c) + spot_length/2; % Spot Center X
    target_y = lane_width + (target_r-1)*spot_width + spot_width/2; % Spot Center Y
    target_pos = [target_x, target_y];

    % Find nearest vertical lane to the target spot
    [~, nearest_lane_idx] = min(abs(lane_centers_x - target_x));
    nearest_lane_x = lane_centers_x(nearest_lane_idx);

    % Determine entry lane based on spot Y position
    if target_y < (lane_width + num_spots_per_row*spot_width/2)
        target_entry_lane_y = lane_centers_y(1); % Lower lane
    else
        target_entry_lane_y = lane_centers_y(2); % Upper lane
    end

    waypoints = [];
    current_level = start_level;

    % --- Waypoint Generation (Standard part) ---
    waypoints = [waypoints; start_pos, current_level];
    waypoints = [waypoints; start_pos(1), lane_centers_y(1), current_level]; % Align vertically with lower lane

    if current_level ~= target_l % Handle level change via ramp zone
        ramp_target_x = mean(ramp_zone_x);
        waypoints = [waypoints; ramp_target_x, lane_centers_y(1), current_level];
        current_level = target_l;
        waypoints = [waypoints; ramp_target_x, lane_centers_y(1), current_level];
    end

    waypoints = [waypoints; nearest_lane_x, target_entry_lane_y, current_level]; % Move to intersection of horiz/vert lanes
    waypoints = [waypoints; nearest_lane_x, target_y, current_level]; % Move along vert lane to align with spot row

    % --- MODIFIED: Add intermediate point for safer turn ---
    % Calculate a point just outside the spot boundary, aligned with spot center Y, but on the lane side X
    x_offset_direction = sign(nearest_lane_x - target_x); % Is lane left (-1) or right (+1) of spot center?
    spot_entry_x = target_x + x_offset_direction * (spot_length / 2 + car_width*0.1); % Point slightly outside spot edge X
    % Ensure entry point isn't exactly lane center if spot is right next to it
    if abs(spot_entry_x - nearest_lane_x) < 0.1
        spot_entry_x = nearest_lane_x + x_offset_direction*0.2;
    end
    waypoints = [waypoints; spot_entry_x, target_y, current_level]; % Intermediate point just before final turn
    % --- END MODIFIED ---

    waypoints = [waypoints; target_x, target_y, current_level]; % Final spot center

    % --- Interpolation (Unchanged) ---
    path = [];
    num_interp_points = 5;
    for i = 1:size(waypoints, 1)-1
         p1 = waypoints(i,:);
         p2 = waypoints(i+1,:);
         segment = [linspace(p1(1), p2(1), num_interp_points)', ...
                    linspace(p1(2), p2(2), num_interp_points)', ...
                    repmat(p2(3), num_interp_points, 1)];
         if i > 1 && norm(segment(1,1:2) - path(end,1:2)) < 1e-6
              path = [path; segment(2:end,:)];
         else
              path = [path; segment];
         end
    end
     if ~isempty(path) && norm(path(end,1:2) - waypoints(end,1:2)) > 1e-6
         path(end+1,:) = waypoints(end,:);
     elseif isempty(path) && size(waypoints,1)>0
          path = waypoints(end,:);
     end
end
function path = generate_path_to_spot(start_pos, start_level, target_spot_info, column_x, lane_centers_x, lane_centers_y, spot_width, lane_width, spot_length, entrance_width, num_spots_per_row, ramp_zone_x, ramp_zone_y_lower, car_width) % Added car_width
% Function to Generate Exit Path (Multi-Level) - Unchanged
%function path = generate_exit_path(spot_pos, spot_level, target_spot_info, column_x, lane_centers_x, lane_centers_y, spot_width, lane_width, entrance_width, spot_length, num_spots_per_row, ramp_zone_x, ramp_zone_y_lower)
    spot_r = target_spot_info(1);
    spot_c = target_spot_info(2);

    start_x = column_x(spot_c) + spot_length/2; % Spot center X
    start_y = lane_width + (spot_r-1)*spot_width + spot_width/2; % Spot center Y
    start_pos = [start_x, start_y];

    % Find nearest vertical lane
    [~, nearest_lane_idx] = min(abs(lane_centers_x - start_x));
    nearest_lane_x = lane_centers_x(nearest_lane_idx);

    % Determine which horizontal lane car is exiting towards initially
    if start_y < (lane_width + num_spots_per_row*spot_width/2)
        exit_lane_y = lane_centers_y(1); % Lower lane
    else
        exit_lane_y = lane_centers_y(2); % Upper lane
    end

    waypoints = [];
    current_level = spot_level;

    % --- Waypoint Generation ---
    waypoints = [waypoints; start_pos, current_level]; % 1. Start in spot

    % --- MODIFIED: Add intermediate point for safer exit turn ---
    x_offset_direction = sign(nearest_lane_x - start_x); % Is lane left (-1) or right (+1) of spot center?
    spot_exit_x = start_x + x_offset_direction * (spot_length / 2 + car_width*0.1); % Point slightly outside spot edge X
    if abs(spot_exit_x - nearest_lane_x) < 0.1 % Prevent being identical to lane center
         spot_exit_x = nearest_lane_x + x_offset_direction*0.2;
    end
    waypoints = [waypoints; spot_exit_x, start_y, current_level]; % 2. Intermediate point just outside spot
    % --- END MODIFIED ---

    waypoints = [waypoints; nearest_lane_x, start_y, current_level]; % 3. Move fully onto lane centerline (aligned with spot row)
    waypoints = [waypoints; nearest_lane_x, exit_lane_y, current_level]; % 4. Move along lane to appropriate horizontal lane
     if exit_lane_y ~= lane_centers_y(1) % If started in upper lane, move to lower lane for exit
         waypoints = [waypoints; nearest_lane_x, lane_centers_y(1), current_level];
     end

    % Handle level change via ramp zone (if needed)
    if current_level ~= 1
        ramp_target_x = mean(ramp_zone_x);
        waypoints = [waypoints; ramp_target_x, lane_centers_y(1), current_level];
        current_level = 1;
        waypoints = [waypoints; ramp_target_x, lane_centers_y(1), current_level];
    end

    waypoints = [waypoints; entrance_width/2, lane_centers_y(1), current_level]; % Move towards exit point
    waypoints = [waypoints; entrance_width/2, -5, current_level]; % Move out of garage

    % --- Interpolation (Unchanged) ---
     path = [];
     num_interp_points = 5;
     for i = 1:size(waypoints, 1)-1
         p1 = waypoints(i,:);
         p2 = waypoints(i+1,:);
         segment = [linspace(p1(1), p2(1), num_interp_points)', ...
                    linspace(p1(2), p2(2), num_interp_points)', ...
                    repmat(p2(3), num_interp_points, 1)];
         if i > 1 && norm(segment(1,1:2) - path(end,1:2)) < 1e-6
              path = [path; segment(2:end,:)];
         else
              path = [path; segment];
         end
     end
     if ~isempty(path) && norm(path(end,1:2) - waypoints(end,1:2)) > 1e-6
         path(end+1,:) = waypoints(end,:);
     elseif isempty(path) && size(waypoints,1)>0
          path = waypoints(end,:);
     end
end
function path = generate_exit_path(spot_pos, spot_level, target_spot_info, column_x, lane_centers_x, lane_centers_y, spot_width, lane_width, entrance_width, spot_length, num_spots_per_row, ramp_zone_x, ramp_zone_y_lower, car_width) % Added car_width

% Function to Draw a Car with Orientation - Unchanged
function draw_car(ax, car_state, car_length, car_width)
    pos = car_state.position;
    angle_deg = car_state.orientation;
    angle_rad = deg2rad(angle_deg);

    corners_x_rel = [-car_length/2, car_length/2, car_length/2, -car_length/2];
    corners_y_rel = [-car_width/2, -car_width/2, car_width/2, car_width/2];

    R = [cos(angle_rad), -sin(angle_rad); sin(angle_rad), cos(angle_rad)];
    rotated_corners = R * [corners_x_rel; corners_y_rel];

    corners_x_abs = rotated_corners(1,:) + pos(1);
    corners_y_abs = rotated_corners(2,:) + pos(2);

    color = [0.5 0.5 0.5]; % Default gray
    switch car_state.state
        case {'entering', 'finding_spot'}
            color = [0 0.8 0]; % Green
        case 'driving_to_spot'
            color = [0 0 1];   % Blue
        case 'parking'
            color = [1 0 0];   % Red
        case 'leaving_spot'
            color = [1 0.6 0]; % Orange
         case 'waiting'
             color = [1 1 0]; % Yellow
        case 'exiting'
            color = [0.2 0.8 0.2]; % Lighter green
    end

    patch(ax, corners_x_abs, corners_y_abs, color, 'EdgeColor', 'k', 'LineWidth', 1);
    text(ax, pos(1), pos(2), num2str(car_state.id), 'Color', 'w', 'FontWeight', 'bold', ...
         'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FontSize', 8);
     if strcmp(car_state.state, 'waiting')
         text(ax, pos(1), pos(2)+car_width*0.7, 'W', 'Color', 'k', 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'FontSize', 9);
     end
end

% Function to Check Collision - Unchanged
function [collision_detected, blocking_agent_id] = check_collision(car_id, car_level, car_pos, car_radius, all_cars, obstacles, obstacle_margin, car_length, car_width)
    collision_detected = false;
    blocking_agent_id = 0; % 0: No collision, >0: Car ID, <0: Obstacle ID

    % Check against other cars
    for i = 1:length(all_cars)
        other_car = all_cars(i);
        % Skip self, cars on different levels, or parked cars
        if other_car.id == car_id || other_car.level ~= car_level || strcmp(other_car.state, 'parking')
            continue;
        end

        dist_sq = sum((car_pos - other_car.position).^2);
        effective_other_radius = max(car_length, car_width)/2 + 0.2; % Buffer
        min_dist_sq = (car_radius + effective_other_radius)^2;

        if dist_sq < min_dist_sq
            collision_detected = true;
            blocking_agent_id = other_car.id; % Return ID of blocking car
            % fprintf('Collision check: Car %d blocked by Car %d\n', car_id, blocking_agent_id); % Debug
            return; % Exit early
        end
    end

    % Check against obstacles
    for k = 1:length(obstacles)
        obs = obstacles(k);
         if obs.level ~= car_level
             continue;
         end
        dist_sq = sum((car_pos - obs.position).^2);
        min_dist_sq = (car_radius + obs.radius + obstacle_margin)^2;

        if dist_sq < min_dist_sq
            collision_detected = true;
            blocking_agent_id = -obs.id; % Return negative ID for obstacle
             % fprintf('Collision check: Car %d blocked by Obstacle %d\n', car_id, -blocking_agent_id); % Debug
            return; % Exit early
        end
    end
end 

% Function to Check if a Position is Inside an Occupied Parking Spot
function spot_collision = is_pos_in_occupied_spot(pos_to_check, car_level, car_id, car_state, target_spot, parking_spots, spot_assignments, column_x, spot_length, spot_width, lane_width, num_spots_per_row, num_columns)
    spot_collision = false;
    px = pos_to_check(1);
    py = pos_to_check(2);

    for r = 1:num_spots_per_row
        for c = 1:num_columns
            % Check if this spot on the correct level is occupied
            if parking_spots(r, c, car_level) == 1
                % Get the ID of the car supposedly in this spot
                parked_car_id = spot_assignments(r, c, car_level);

                % Define spot boundaries
                spot_x_min = column_x(c);
                spot_x_max = spot_x_min + spot_length;
                spot_y_min = lane_width + (r-1)*spot_width;
                spot_y_max = spot_y_min + spot_width;

                % Allow car to enter/exit its *own* target/origin spot
                is_own_target_spot = false;
                if ~isempty(target_spot) && isequal([r, c, car_level], target_spot)
                    % Allow entering target spot or leaving origin spot
                    if strcmp(car_state, 'driving_to_spot') || strcmp(car_state, 'leaving_spot')
                       is_own_target_spot = true;
                    end
                    % Additionally, make sure it's not someone else's spot
                    if parked_car_id ~= car_id && parked_car_id ~= 0 % Check if occupied by another known car
                         is_own_target_spot = false; % Prevent entry if occupied by someone else
                    end
                end

                % If the position is within the bounds of an occupied spot
                % AND it's not the car's own target/origin spot it's allowed to interact with
                if ~is_own_target_spot && ...
                   px >= spot_x_min && px <= spot_x_max && ...
                   py >= spot_y_min && py <= spot_y_max

                    spot_collision = true;
                    % fprintf('Car %d avoided collision with occupied spot [%d, %d, %d]\n', car_id, r, c, car_level); % Debug
                    return; % Exit early
                end
            end
        end
    end
end