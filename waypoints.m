function wp = waypoints(x, xp, radar_points)
% WAYPOINTS Discretizes the local path segment, applies Z-axis radar inversion,
% and shifts all nodes within the perpendicular frame in REVERSE sequence 
% to allow downstream obstacle avoidance to propagate backward to the next node.
%
% Inputs:
%   x            - Current vehicle state vector. x(7:9) is [X; Y; Z] position (m).
%   xp           - Global goal state vector. xp(7:9) is target [X; Y; Z] (m).
%   radar_points - Matrix of raw obstacle coordinates (Nx3) [X, Y, Z_raw].
%
% Output:
%   wp           - Next local waypoint coordinate vector [X, Y, Z] (m).

    % 1. Extract and standardize 3D vehicle and target positions
    current_pos = x(7:9);
    goal_pos    = xp(7:9);
    
    current_pos = current_pos(:);
    goal_pos    = goal_pos(:);
    
    % 2. Calculate vector metrics to the global goal
    to_goal_vec = goal_pos - current_pos;
    dist_to_goal = norm(to_goal_vec);
    
    % Exit early if already precisely at the goal location
    if dist_to_goal < 0.1
        wp = goal_pos';
        return;
    end
    dir_to_goal = to_goal_vec / dist_to_goal;

    % 3. Divide path into segments based on the 5m max speed window
    look_ahead_dist = min(3.0, dist_to_goal);
    %In case the goal is beyond the look-ahead distance
    segment_target = current_pos + (dir_to_goal * look_ahead_dist);
    
    % Generate discrete test nodes along this specific look-ahead path
    num_nodes = 6;
    t = linspace(0, 1, num_nodes + 1); 
    t = t(2:end); % The final node (t=1) is segment_target
    
    nodes = current_pos' + t' * (segment_target - current_pos)'; % [num_nodes x 3]

    % 4. Build a coordinate framework perpendicular to the trajectory line
    if abs(dir_to_goal(3)) < 0.9
        perp1 = cross(dir_to_goal, [0; 0; 1]);
    else
        perp1 = cross(dir_to_goal, [0; 1; 0]);
    end
    perp1 = perp1 / norm(perp1);
    perp2 = cross(dir_to_goal, perp1); % Second basis vector of the perpendicular plane

    % 5. Invert the Z-axis of incoming radar points to match vehicle frame
    if nargin >= 3 && ~isempty(radar_points)
        % Strip out uninitialized or zero-filled sensor rows
        radar_points = radar_points(any(radar_points ~= 0, 2), 1:3);
        
        if ~isempty(radar_points)
            % Invert the Y and Z-axis coordinates to match vehicle spatial tracking frame
            radar_points(:, 2) = -radar_points(:, 2);
            radar_points(:, 3) = -radar_points(:, 3);
        end
    else
        radar_points = [];
    end

    % 6. REVERSE Node Validation and Perpendicular Shifting
    clearance_rad = 0.8; % Minimum allowed radius from obstacles (m)
    
    % REVERSE PIPELINE INITIALIZATION:
    % Node 5 is the furthest downstream node. Its "next sequential point" 
    % on the global trajectory line is the target endpoint of this segment.
    previous_node = segment_target'; 
    
    % GROUND CONSTRAINT SPECIFICATION (Z-Down / NED Frame)
    max_allowable_z = -0.3; 

    % Run the optimization backwards from the horizon node down to the immediate node
    drift_is_active = false;
    for i = num_nodes:-1:1
        node_pos = nodes(i, :);
        
        % Check if this node breaches the clearance threshold of transformed radar obstacles
        is_blocked = false;
        if ~isempty(radar_points)
            dists_to_obs = sqrt(sum((radar_points - node_pos).^2, 2));
            if any(dists_to_obs < clearance_rad)
                is_blocked = true;
                drift_is_active = true;
            end
        end
        
        % Also trigger a block if the default straight-line node itself is below the floor limit
        if node_pos(3) > max_allowable_z
            is_blocked = true;
        end
        
        % If blocked, optimize a shift within the perpendicular plane
        if drift_is_active
            best_shift_vec = [0, 0, 0];
            min_cost = Inf;
            
            % High-Density Candidate Mesh
            angles = linspace(0, 2*pi, 24);       
            step_sizes = linspace(0.0, 2.0, 20);   
            
            for step = step_sizes
             
                
                for theta = angles
                    % Construct a 3D displacement vector in the perpendicular plane
                    shift_candidate = (cos(theta) * perp1 + sin(theta) * perp2)' * step;
                    candidate_pos = node_pos + shift_candidate;
                    
                    % Explicitly reject candidates that breach the ground floor boundary
                    if candidate_pos(3) > max_allowable_z
                        continue; 
                    end
                    
                    % Check obstacle clearance using the inverted radar coordinates
                    if ~isempty(radar_points)
                        obs_dists = sqrt(sum((radar_points - candidate_pos).^2, 2));
                        if any(obs_dists < clearance_rad)
                            continue; 
                        end
                    end
                    
                    % Cost metrics relative to the line and the downstream optimized node
                    dist_from_line = step; 
                    dist_from_prev = norm(candidate_pos - previous_node);
                    
                    % Balance weights: 60% closeness to line, 40% closeness to downstream node
                    total_cost = (0.4 * dist_from_line) + (0.6 * dist_from_prev);
                    
                    if total_cost < min_cost
                        min_cost = total_cost;
                        best_shift_vec = shift_candidate;
                        found_safe_in_ring = true;
                    end
                end
                
          
            end
            
            % Apply the optimal calculated perpendicular deviation vector to the node
            if min_cost < Inf
                nodes(i, :) = node_pos + best_shift_vec;
            else
                % Emergency Fallback: Force node upwards relative to the downstream node state
                nodes(i, 3) = min(node_pos(3), previous_node(3)) - 0.5;
            end
        end
        
        % Pass this optimized node backward to become the target reference for the node behind it
        previous_node = nodes(i, :);
    end

    % 7. Raw Vector Assignment (Filtering completely disabled)
    wp = nodes(2, :);
    
    % Hard constraint verification safety check
    if wp(3) > max_allowable_z
        wp(3) = max_allowable_z;
    end
end
