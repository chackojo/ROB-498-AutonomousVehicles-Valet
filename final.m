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
num_obstacles = 5; % REDUCED number of random obstacles (pedestrians)
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
car_arrival_probability = 0.05; % REDUCED Chance of a new car arriving
min_stay_hours = 0.1; % Min stay for both pre-parked and new cars
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

                % Block generation IF the last car is NOT parked or has left
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
    obstacles = [];
    for level = 1:num_levels
        for k = 1:num_obstacles
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

        % --- State Machine Logic ---
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
                    if other_car.level == car_states(i).level && ...
                       norm(other_car.position - entry_point_pos) < entry_point_tolerance && ...
                       (strcmp(other_car.state, 'entering') || strcmp(other_car.state, 'finding_spot') || strcmp(other_car.state, 'waiting'))
                        is_blocked = true;
                        break;
                    end
                end

                % Only move if entry point is not blocked
                if ~is_blocked
                    move_speed_entering = 6; % Increased speed
                    current_y = car_states(i).position(2);
                    move_dist = move_speed_entering * dt * 3600;
                    if current_y + move_dist >= entry_target_y
                        car_states(i).position(2) = entry_target_y;
                        car_states(i).position(1) = entrance_width/2;
                        car_states(i).state = 'finding_spot';
                    else
                        car_states(i).position(2) = current_y + move_dist;
                    end
                end
                % If blocked, do nothing (wait)

            case 'finding_spot'
                [spot_pos, spot_r, spot_c, spot_l] = find_nearest_spot(car_states(i).position, car_states(i).level, parking_spots, column_x, lane_width, spot_width, spot_length, num_levels, lot_height, lot_width, num_spots_per_row, num_columns);
                if ~isempty(spot_pos)
                    if parking_spots(spot_r, spot_c, spot_l) == 0
                        car_states(i).target_spot = [spot_r, spot_c, spot_l];
                        car_states(i).path = generate_path_to_spot(car_states(i).position, car_states(i).level, car_states(i).target_spot, column_x, lane_centers_x, lane_centers_y, spot_width, lane_width, spot_length, entrance_width, num_spots_per_row, ramp_zone_x, ramp_zone_y_lower, car_width); % Pass car_width
                        car_states(i).path_index = 1;
                        car_states(i).state = 'driving_to_spot';
                        parking_spots(spot_r, spot_c, spot_l) = 1;
                        spot_assignments(spot_r, spot_c, spot_l) = car_id;
                         % fprintf('Car %d: Found spot [%d, %d, %d]. Path generated (length %d). State -> driving_to_spot\n', car_id, spot_r, spot_c, spot_l, size(car_states(i).path, 1)); % Debug
                    else
                         car_states(i).state = 'waiting'; % Spot taken
                         car_states(i).collision_check_timer = 1*dt;
                    end
                else
                    car_states(i).state = 'waiting'; % No spot found
                end

            case 'driving_to_spot'
                if isempty(car_states(i).path) || car_states(i).path_index > size(car_states(i).path, 1)
                    car_states(i).state = 'parking'; % Arrived at spot
                    if ~isempty(car_states(i).target_spot)
                        spot_r = car_states(i).target_spot(1);
                        spot_c = car_states(i).target_spot(2);
                        spot_l = car_states(i).target_spot(3);
                        car_states(i).position = [column_x(spot_c) + spot_length/2, lane_width + (spot_r-1)*spot_width + spot_width/2];
                        car_states(i).orientation = 90;
                        car_states(i).level = spot_l;
                    end
                    continue;
                end

                % Check for level change (ramp)
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
                             continue;
                         end
                     end
                 end

                % --- Calculate Potential Next Position FIRST ---
                next_path_point = car_states(i).path(car_states(i).path_index, 1:2);
                move_vector = next_path_point - car_states(i).position;
                move_dist = norm(move_vector);
                move_speed_driving = 8; % INCREASED Speed (m/s)
                max_move = move_speed_driving * dt * 3600;

                potential_next_pos = car_states(i).position;
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
                     car_states(i).collision_check_timer = 1 * dt_seconds / (24*3600); % Wait 1 sec
                     % if spot_collision, fprintf('Car %d: Spot collision detected! State -> waiting\n', car_id); % Debug
                     % else, fprintf('Car %d: Car/Obstacle collision detected! State -> waiting\n', car_id); end % Debug
                else % No collision or timer active -> Proceed with movement
                    if strcmp(car_states(i).state, 'waiting')
                        car_states(i).state = 'driving_to_spot';
                    end
                    if car_states(i).collision_check_timer <= 0
                        if move_dist > 0
                            car_states(i).orientation = atan2d(move_vector(2), move_vector(1));
                            if move_dist <= max_move
                                car_states(i).position = next_path_point;
                                car_states(i).path_index = car_states(i).path_index + 1;
                            else
                                car_states(i).position = car_states(i).position + (move_vector / move_dist) * max_move;
                            end
                        else
                             car_states(i).path_index = car_states(i).path_index + 1;
                        end
                    end
                end

            case 'parking'
                if current_time >= car_states(i).parking_time_leave
                     if ~isempty(car_states(i).target_spot)
                         spot_r = car_states(i).target_spot(1);
                         spot_c = car_states(i).target_spot(2);
                         spot_l = car_states(i).target_spot(3);
                         car_states(i).state = 'leaving_spot';
                         car_states(i).path = generate_exit_path(car_states(i).position, car_states(i).level, car_states(i).target_spot, column_x, lane_centers_x, lane_centers_y, spot_width, lane_width, entrance_width, spot_length, num_spots_per_row, ramp_zone_x, ramp_zone_y_lower, car_width); % Pass car_width
                         car_states(i).path_index = 1;
                         parking_spots(spot_r, spot_c, spot_l) = 0;
                         spot_assignments(spot_r, spot_c, spot_l) = 0;
                     else
                          car_states(i).state = 'exited'; % Error case
                          cars_to_remove_indices(end+1) = i;
                     end
                end

            case 'leaving_spot'
                 if isempty(car_states(i).path) || car_states(i).path_index > size(car_states(i).path, 1)
                     car_states(i).state = 'exiting';
                     continue;
                 end

                 % Check for level change (ramp)
                 current_pos = car_states(i).position;
                 target_level = 1;
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
                            continue;
                         end
                     end
                 end

                 % --- Calculate Potential Next Position FIRST ---
                 next_path_point = car_states(i).path(car_states(i).path_index, 1:2);
                 move_vector = next_path_point - car_states(i).position;
                 move_dist = norm(move_vector);
                 move_speed_leaving = 8; % INCREASED Speed (m/s)
                 max_move = move_speed_leaving * dt * 3600;

                 potential_next_pos = car_states(i).position;
                 if move_dist > 0
                    potential_next_pos = car_states(i).position + min(1, max_move / move_dist) * move_vector;
                 end
                 % --- End Calculation ---

                 % --- Perform Collision Checks ---
                 [car_obs_collision, ~] = check_collision(car_id, car_states(i).level, potential_next_pos, car_safety_radius, car_states, obstacles, obstacle_safety_margin, car_length, car_width);
                 spot_collision = is_pos_in_occupied_spot(potential_next_pos, car_states(i).level, car_id, car_states(i).state, car_states(i).target_spot, parking_spots, spot_assignments, column_x, spot_length, spot_width, lane_width, num_spots_per_row, num_columns);
                 collision_detected = car_obs_collision || spot_collision;
                 % --- End Checks ---

                 % --- Decide Action ---
                 if collision_detected && car_states(i).collision_check_timer <= 0
                     car_states(i).state = 'waiting';
                     car_states(i).collision_check_timer = 1 * dt_seconds / (24*3600);
                 else
                     if strcmp(car_states(i).state, 'waiting')
                         car_states(i).state = 'leaving_spot';
                     end
                     if car_states(i).collision_check_timer <= 0
                        if move_dist > 0
                            car_states(i).orientation = atan2d(move_vector(2), move_vector(1));
                            if move_dist <= max_move
                                car_states(i).position = next_path_point;
                                car_states(i).path_index = car_states(i).path_index + 1;
                            else
                                car_states(i).position = car_states(i).position + (move_vector / move_dist) * max_move;
                            end
                        else
                            car_states(i).path_index = car_states(i).path_index + 1;
                        end
                    end
                 end

            case 'waiting'
             % Check if timer expired
             if car_states(i).collision_check_timer <= 0
                 current_pos = car_states(i).position;
                 [collision, blocking_agent_id] = check_collision(car_id, car_states(i).level, current_pos, car_safety_radius, car_states, obstacles, obstacle_safety_margin, car_length, car_width);
                 spot_collision = is_pos_in_occupied_spot(current_pos, car_states(i).level, car_id, car_states(i).state, car_states(i).target_spot, parking_spots, spot_assignments, column_x, spot_length, spot_width, lane_width, num_spots_per_row, num_columns);

                 if ~collision && ~spot_collision
                     % --- Obstruction Cleared ---
                     if ~isempty(car_states(i).target_spot) && isequal(car_states(i).target_spot(1:2), [-1 -1]) % Special target for just entered?
                         % This condition might need review - was intended for resuming drive/leave
                         car_states(i).state = 'finding_spot'; % Default if target unclear
                     elseif ~isempty(car_states(i).target_spot) && spot_assignments(car_states(i).target_spot(1), car_states(i).target_spot(2), car_states(i).target_spot(3)) == car_id
                         if ~isempty(car_states(i).path) && car_states(i).path_index <= size(car_states(i).path,1)
                             car_states(i).state = 'driving_to_spot';
                             car_states(i).collision_check_timer = rand() * 0.5 * dt_seconds / (24*3600); % Stagger
                         else % Path lost/finished?
                             dist_to_target = norm(car_states(i).position - [column_x(car_states(i).target_spot(2)) + spot_length/2, lane_width + (car_states(i).target_spot(1)-1)*spot_width + spot_width/2]);
                             if dist_to_target < spot_length / 2
                                 car_states(i).state = 'parking';
                             else % Re-plan if far from target without path
                                 car_states(i).state = 'finding_spot';
                                 spot_assignments(car_states(i).target_spot(1), car_states(i).target_spot(2), car_states(i).target_spot(3)) = 0;
                                 parking_spots(car_states(i).target_spot(1), car_states(i).target_spot(2), car_states(i).target_spot(3)) = 0;
                                 car_states(i).target_spot = [];
                             end
                         end
                     elseif ~isempty(car_states(i).path) % If path exists, assume leaving
                         car_states(i).state = 'leaving_spot';
                         car_states(i).collision_check_timer = rand() * 0.5 * dt_seconds / (24*3600); % Stagger
                     else
                         car_states(i).state = 'finding_spot'; % Default if unsure
                     end
                 else % Still Blocked
                     standard_wait_time = 1 * dt_seconds / (24*3600);
                     longer_wait_base = 2 * dt_seconds / (24*3600);
                     random_yield_add = rand() * 2 * dt_seconds / (24*3600);

                     if collision && blocking_agent_id > 0 % Blocked by another car (use collision check result)
                         blocker_idx = find([car_states.id] == blocking_agent_id, 1);
                         if ~isempty(blocker_idx) && strcmp(car_states(blocker_idx).state, 'waiting')
                             if car_id > blocking_agent_id
                                 car_states(i).collision_check_timer = longer_wait_base + random_yield_add; % Yield
                             else
                                 car_states(i).collision_check_timer = standard_wait_time; % Don't yield
                             end
                         else
                              car_states(i).collision_check_timer = standard_wait_time; % Blocker not waiting
                         end
                     else % Blocked by obstacle or spot
                         car_states(i).collision_check_timer = standard_wait_time;
                     end
                 end
             end

            case 'exiting'
                move_speed_exiting = 10; % INCREASED Speed (m/s)
                car_states(i).position(2) = car_states(i).position(2) - move_speed_exiting * dt * 3600;
                if car_states(i).position(2) < -car_length
                    car_states(i).state = 'exited';
                    cars_to_remove_indices(end+1) = i;
                end
        end % End switch state
    end % End for each car

    % Remove exited cars
    if ~isempty(cars_to_remove_indices)
       ids_to_remove = [car_states(cars_to_remove_indices).id];
       car_states(cars_to_remove_indices) = [];
       active_cars_list = setdiff(active_cars_list, ids_to_remove, 'stable');
    end

    % 4. Update Visualization
    cla(ax);
    hold(ax, 'on');
    grid(ax, 'on');
    axis(ax, 'equal');
    axis(ax, [-10 lot_width+10 -10 lot_height+10]);
    xlabel(ax, 'X (m)');
    ylabel(ax, 'Y (m)');
    num_active_cars = length(active_cars_list);
    title(ax, sprintf('Parking Garage Level %d - Time: %s - Cars Active: %d', current_level_display, datestr(current_time, 'HH:MM:SS'), num_active_cars));

    % Draw spots
    for r = 1:num_spots_per_row
        y_base = lane_width + (r-1)*spot_width;
        for c = 1:num_columns
            spot_state = parking_spots(r, c, current_level_display);
            car_in_spot_id = spot_assignments(r,c,current_level_display);
            color = 'w'; % Empty
            if spot_state == 1
                 car_idx = find([car_states.id] == car_in_spot_id, 1);
                 if ~isempty(car_idx)
                     if strcmp(car_states(car_idx).state, 'parking')
                        color = [1 0.7 0.7]; % Parked - Light Red
                     else
                        color = [0.8 0.8 0.8]; % Reserved - Gray
                     end
                 else; color = [0.9 0.9 0.9]; end % Error state color
            end
            rectangle(ax, 'Position', [column_x(c), y_base, spot_length, spot_width], ...
                      'EdgeColor', 'k', 'LineWidth', 0.5, 'FaceColor', color);
        end
    end

     % Draw Lane Markings & Ramp Zone
     plot(ax, [0 lot_width], [lane_centers_y(1) lane_centers_y(1)], 'b:', 'LineWidth', 1);
     plot(ax, [0 lot_width], [lane_centers_y(2) lane_centers_y(2)], 'b:', 'LineWidth', 1);
     for lx = lane_centers_x; plot(ax, [lx lx], [0 lot_height], 'b:', 'LineWidth', 1); end
     rectangle(ax, 'Position', [ramp_zone_x(1), ramp_zone_y_lower(1), diff(ramp_zone_x), diff(ramp_zone_y_lower)], 'EdgeColor', 'm', 'LineStyle',':', 'LineWidth', 1.5);
     text(ax, mean(ramp_zone_x), mean(ramp_zone_y_lower), 'Ramp', 'Color', 'm', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');

    % Draw Obstacles
    for k = 1:length(obstacles)
        if obstacles(k).level == current_level_display
             center = obstacles(k).position; radius = obstacles(k).radius;
             rectangle(ax, 'Position', [center(1)-radius, center(2)-radius, 2*radius, 2*radius], ...
                      'Curvature', [1 1], 'FaceColor', [0.6 0.6 0.1], 'EdgeColor', 'none');
        end
    end

    % Draw Cars
    for i = 1:length(car_states)
        if car_states(i).level == current_level_display
            draw_car(ax, car_states(i), car_length, car_width);
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

    drawnow limitrate;
    pause(0.1); % SLOWED DOWN

    % 5. Advance Time
    current_time = current_time + dt;
end

disp('Simulation finished.');

% --- Helper Functions ---

% Function to Update Level Display
function update_level_display(slider_obj)
    level = round(slider_obj.Value);
    parent_fig = slider_obj.Parent;
    level_val_text = findobj(parent_fig, 'Style', 'text', '-regexp', 'Position', '\[460,.*');
    if ~isempty(level_val_text); level_val_text.String = num2str(level); end
end

% Function to Find Nearest Available Spot
function [spot_pos, spot_r, spot_c, spot_l] = find_nearest_spot(car_pos, car_level, parking_spots, column_x, lane_width, spot_width, spot_length, num_levels, lot_height, lot_width, num_spots_per_row, num_columns)
    min_cost = inf; spot_pos = []; spot_r = -1; spot_c = -1; spot_l = -1;
    level_search_order = [car_level, setdiff(1:num_levels, car_level)];
    for level = level_search_order
        for r = 1:num_spots_per_row
            for c = 1:num_columns
                if parking_spots(r, c, level) == 0
                    y_pos = lane_width + (r-1)*spot_width + spot_width/2;
                    x_pos = column_x(c) + spot_length/2;
                    potential_pos = [x_pos, y_pos];
                    dist_cost = norm(potential_pos - car_pos);
                    level_change_cost = abs(level - car_level) * (lot_height + lot_width);
                    cost = dist_cost + level_change_cost;
                    if cost < min_cost
                        min_cost = cost; spot_pos = potential_pos; spot_r = r; spot_c = c; spot_l = level;
                    end
                end
            end
        end
         if ~isempty(spot_pos) && level == car_level; break; end % Prioritize same level
    end
end

% Function to Generate Path to Spot (MODIFIED Entry Maneuver)
function path = generate_path_to_spot(start_pos, start_level, target_spot_info, column_x, lane_centers_x, lane_centers_y, spot_width, lane_width, spot_length, entrance_width, num_spots_per_row, ramp_zone_x, ramp_zone_y_lower, car_width) % Added car_width
    target_r = target_spot_info(1); target_c = target_spot_info(2); target_l = target_spot_info(3);
    target_x = column_x(target_c) + spot_length/2; target_y = lane_width + (target_r-1)*spot_width + spot_width/2;
    [~, nearest_lane_idx] = min(abs(lane_centers_x - target_x)); nearest_lane_x = lane_centers_x(nearest_lane_idx);
    if target_y < (lane_width + num_spots_per_row*spot_width/2); target_entry_lane_y = lane_centers_y(1); else; target_entry_lane_y = lane_centers_y(2); end
    waypoints = []; current_level = start_level;
    waypoints = [waypoints; start_pos, current_level]; waypoints = [waypoints; start_pos(1), lane_centers_y(1), current_level];
    if current_level ~= target_l; ramp_target_x = mean(ramp_zone_x); waypoints = [waypoints; ramp_target_x, lane_centers_y(1), current_level]; current_level = target_l; waypoints = [waypoints; ramp_target_x, lane_centers_y(1), current_level]; end
    waypoints = [waypoints; nearest_lane_x, target_entry_lane_y, current_level]; waypoints = [waypoints; nearest_lane_x, target_y, current_level];
    x_offset_direction = sign(nearest_lane_x - target_x); spot_entry_x = target_x + x_offset_direction * (spot_length / 2 + car_width*0.1);
    if abs(spot_entry_x - nearest_lane_x) < 0.1; spot_entry_x = nearest_lane_x + x_offset_direction*0.2; end
    waypoints = [waypoints; spot_entry_x, target_y, current_level]; % Intermediate point
    waypoints = [waypoints; target_x, target_y, current_level]; % Final spot center
    path = []; num_interp_points = 5;
    for i = 1:size(waypoints, 1)-1; p1 = waypoints(i,:); p2 = waypoints(i+1,:); segment = [linspace(p1(1), p2(1), num_interp_points)', linspace(p1(2), p2(2), num_interp_points)', repmat(p2(3), num_interp_points, 1)]; if i > 1 && norm(segment(1,1:2) - path(end,1:2)) < 1e-6; path = [path; segment(2:end,:)]; else; path = [path; segment]; end; end
    if ~isempty(path) && norm(path(end,1:2) - waypoints(end,1:2)) > 1e-6; path(end+1,:) = waypoints(end,:); elseif isempty(path) && size(waypoints,1)>0; path = waypoints(end,:); end
end % Removed extra end here

% Function to Generate Exit Path (MODIFIED Exit Maneuver)
function path = generate_exit_path(spot_pos, spot_level, target_spot_info, column_x, lane_centers_x, lane_centers_y, spot_width, lane_width, entrance_width, spot_length, num_spots_per_row, ramp_zone_x, ramp_zone_y_lower, car_width) % Added car_width
    spot_r = target_spot_info(1); spot_c = target_spot_info(2);
    start_x = column_x(spot_c) + spot_length/2; start_y = lane_width + (spot_r-1)*spot_width + spot_width/2; start_pos = [start_x, start_y];
    [~, nearest_lane_idx] = min(abs(lane_centers_x - start_x)); nearest_lane_x = lane_centers_x(nearest_lane_idx);
    if start_y < (lane_width + num_spots_per_row*spot_width/2); exit_lane_y = lane_centers_y(1); else; exit_lane_y = lane_centers_y(2); end
    waypoints = []; current_level = spot_level;
    waypoints = [waypoints; start_pos, current_level]; % 1. Start
    x_offset_direction = sign(nearest_lane_x - start_x); spot_exit_x = start_x + x_offset_direction * (spot_length / 2 + car_width*0.1);
    if abs(spot_exit_x - nearest_lane_x) < 0.1; spot_exit_x = nearest_lane_x + x_offset_direction*0.2; end
    waypoints = [waypoints; spot_exit_x, start_y, current_level]; % 2. Intermediate point
    waypoints = [waypoints; nearest_lane_x, start_y, current_level]; % 3. Onto lane center
    waypoints = [waypoints; nearest_lane_x, exit_lane_y, current_level]; % 4. Move to horiz lane
    if exit_lane_y ~= lane_centers_y(1); waypoints = [waypoints; nearest_lane_x, lane_centers_y(1), current_level]; end
    if current_level ~= 1; ramp_target_x = mean(ramp_zone_x); waypoints = [waypoints; ramp_target_x, lane_centers_y(1), current_level]; current_level = 1; waypoints = [waypoints; ramp_target_x, lane_centers_y(1), current_level]; end
    waypoints = [waypoints; entrance_width/2, lane_centers_y(1), current_level]; waypoints = [waypoints; entrance_width/2, -5, current_level];
    path = []; num_interp_points = 5;
    for i = 1:size(waypoints, 1)-1; p1 = waypoints(i,:); p2 = waypoints(i+1,:); segment = [linspace(p1(1), p2(1), num_interp_points)', linspace(p1(2), p2(2), num_interp_points)', repmat(p2(3), num_interp_points, 1)]; if i > 1 && norm(segment(1,1:2) - path(end,1:2)) < 1e-6; path = [path; segment(2:end,:)]; else; path = [path; segment]; end; end
    if ~isempty(path) && norm(path(end,1:2) - waypoints(end,1:2)) > 1e-6; path(end+1,:) = waypoints(end,:); elseif isempty(path) && size(waypoints,1)>0; path = waypoints(end,:); end
end % Removed extra end here

% Function to Draw a Car
function draw_car(ax, car_state, car_length, car_width)
    pos = car_state.position; angle_deg = car_state.orientation; angle_rad = deg2rad(angle_deg);
    corners_x_rel = [-car_length/2, car_length/2, car_length/2, -car_length/2]; corners_y_rel = [-car_width/2, -car_width/2, car_width/2, car_width/2];
    R = [cos(angle_rad), -sin(angle_rad); sin(angle_rad), cos(angle_rad)]; rotated_corners = R * [corners_x_rel; corners_y_rel];
    corners_x_abs = rotated_corners(1,:) + pos(1); corners_y_abs = rotated_corners(2,:) + pos(2);
    color = [0.5 0.5 0.5]; % Default
    switch car_state.state; case {'entering', 'finding_spot'}; color = [0 0.8 0]; case 'driving_to_spot'; color = [0 0 1]; case 'parking'; color = [1 0 0]; case 'leaving_spot'; color = [1 0.6 0]; case 'waiting'; color = [1 1 0]; case 'exiting'; color = [0.2 0.8 0.2]; end
    patch(ax, corners_x_abs, corners_y_abs, color, 'EdgeColor', 'k', 'LineWidth', 1);
    text(ax, pos(1), pos(2), num2str(car_state.id), 'Color', 'w', 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'FontSize', 8);
    if strcmp(car_state.state, 'waiting'); text(ax, pos(1), pos(2)+car_width*0.7, 'W', 'Color', 'k', 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'FontSize', 9); end
end

% Function to Check Collision (Returns blocker ID)
function [collision_detected, blocking_agent_id] = check_collision(car_id, car_level, car_pos, car_radius, all_cars, obstacles, obstacle_margin, car_length, car_width)
    collision_detected = false; blocking_agent_id = 0;
    for i = 1:length(all_cars) % Check other cars
        other_car = all_cars(i);
        if other_car.id == car_id || other_car.level ~= car_level || strcmp(other_car.state, 'parking'); continue; end
        dist_sq = sum((car_pos - other_car.position).^2); effective_other_radius = max(car_length, car_width)/2 + 0.2; min_dist_sq = (car_radius + effective_other_radius)^2;
        if dist_sq < min_dist_sq; collision_detected = true; blocking_agent_id = other_car.id; return; end
    end
    for k = 1:length(obstacles) % Check obstacles
        obs = obstacles(k); if obs.level ~= car_level; continue; end
        dist_sq = sum((car_pos - obs.position).^2); min_dist_sq = (car_radius + obs.radius + obstacle_margin)^2;
        if dist_sq < min_dist_sq; collision_detected = true; blocking_agent_id = -obs.id; return; end
    end
end

% Function to Check if Position is Inside an Occupied Spot
function spot_collision = is_pos_in_occupied_spot(pos_to_check, car_level, car_id, car_state, target_spot, parking_spots, spot_assignments, column_x, spot_length, spot_width, lane_width, num_spots_per_row, num_columns)
    spot_collision = false; px = pos_to_check(1); py = pos_to_check(2);
    for r = 1:num_spots_per_row
        for c = 1:num_columns
            if parking_spots(r, c, car_level) == 1 % Spot occupied
                parked_car_id = spot_assignments(r, c, car_level);
                spot_x_min = column_x(c); spot_x_max = spot_x_min + spot_length; spot_y_min = lane_width + (r-1)*spot_width; spot_y_max = spot_y_min + spot_width;
                is_own_target_spot = false;
                if ~isempty(target_spot) && isequal([r, c, car_level], target_spot)
                    if strcmp(car_state, 'driving_to_spot') || strcmp(car_state, 'leaving_spot'); is_own_target_spot = true; end
                    if parked_car_id ~= car_id && parked_car_id ~= 0; is_own_target_spot = false; end % Don't enter/leave if different car is assigned
                end
                if ~is_own_target_spot && px >= spot_x_min && px <= spot_x_max && py >= spot_y_min && py <= spot_y_max
                    spot_collision = true; return;
                end
            end
        end
    end
end