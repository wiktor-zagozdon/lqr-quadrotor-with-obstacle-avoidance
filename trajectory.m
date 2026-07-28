function xt = trajectory(xp, wp, x, t, radar_points)   
xt=zeros(16,1);
%transformation matrix
    %inverse Euler angles
phi=x(10);
theta=x(11);
psi=x(12);

R_zeta=[cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
R_eta=[cos(theta) 0 sin(theta);0 1 0;-sin(theta) 0 cos(theta)];
R_xi=[1 0 0 ;0 cos(phi) -sin(phi);0 sin(phi) cos(phi)];
    %transformation from body reference frame to the Earth reference frame
T_BC_FC=R_zeta*R_eta*R_xi;

%trajectory computation
    %velocity at the waypoint (optional)
v_x=xp(1);
v_y=xp(2);
v_z=xp(3);
    %velocity in body reference frame
v=T_BC_FC'*[v_x;v_y;v_z];
    %planed waypoint
xt(7)=wp(1);
xt(8)=wp(2);
xt(9)=wp(3);


% %velocity correction based on Predictive-Horizon APF
k_att = 5.0;
k_rep = 5.0;
Q = 0.8;        % Base sensor range / activation radius
look_ahead_time = 0.1; % Time horizon in seconds (Delta t) - Adjust based on agility

% Current velocity vector in world/sensor frame (derived from v_att)
% Assumes v_att reflects your current desired velocity vector before correction
v_current = [(xt(7)-x(7)); (xt(8)-x(8)); (xt(9)-x(9))];
v_att = k_att * v_current;

% 1. Predict future drone position based on current velocity
% x(7), x(8), x(9) are current positions
x_future = x(7) + v_current(1) * look_ahead_time;
y_future = x(8) + v_current(2) * look_ahead_time;
z_future = x(9) + v_current(3) * look_ahead_time;
p_future = [x_future, y_future, z_future];

% 2. Format points and calculate distances from the FUTURE position
p_obs = [radar_points(:,1), -radar_points(:,2), -radar_points(:,3)];
diff_vecs_future = p_future - p_obs; % Vector pointing from obstacle to future position
rho_all_future = sqrt(sum(diff_vecs_future.^2, 2));

% Format points and calculate distances from the current position

diff_vecs = [x(7), x(8), x(9)] - p_obs;
rho_all = sqrt(sum(diff_vecs.^2, 2));
% Filter points within range Q
valid = (rho_all <= Q) & (rho_all > 0);
rho = rho_all(valid);

% 3. Filter points within range Q of the FUTURE position
valid = (rho_all_future <= Q) & (rho_all_future > 0);
rho_fut = rho_all_future(valid);
diffs_fut = diff_vecs_future(valid, :);

% 4. Calculate force based on predictive metrics
v_rep = zeros(3,1);
n_obs = sum(valid); 

if n_obs > 0
    % Uses your stable bounded formula, calculated from future penetration
    v_rep_i = k_rep * (Q - rho_fut).*(Q - rho_fut) .* (diffs_fut ./ rho_fut);
    
    % Sum forces and average by visible obstacles
    v_rep = sum(v_rep_i, 1)' / n_obs;
end

v_cor = T_BC_FC' * (v_att + v_rep);

%longitudinal velocity with limits
v_max=3.0;

%
% breaking near a waypoint or an obstacle
bd=2.5; %breaking distance
nrst=min(rho); %distance to the nearest obstacle
if n_obs>0
    %v_max=1.0;
    fprintf('Obstacle detected at the distance %f. \n', nrst)
end

if abs(x(7)-xp(7))<bd && abs(x(8)-xp(8))<bd
    v_max=1.0;
    fprintf('Reached Vicinity of the waypoint. Velocity reduced. \n')
end
%

% 1. Calculate the raw desired 3D velocity vector
v_raw = [v(1) + v_cor(1); v(2) + v_cor(2); v(3) + v_cor(3)];
total_speed = norm(v_raw);

% 3. Apply the limit to the vector magnitude
if total_speed > v_max
    % Scaling the entire vector preserves the exact 3D direction
    v_limited = (v_raw / total_speed) * v_max;
else
    v_limited = v_raw;
end

% 4. Assign back to target states
xt(1) = v_limited(1); % Limited Longitudinal
xt(2) = v_limited(2); % Limited Lateral
xt(3) = v_limited(3); % Limited Vertical


% --- Reference Generation ---
% Target Angles (Lateral/Longitudinal velocity error -> Tilt)
k_tilt=8.0;
xt(10) = -k_tilt * (x(2) - xt(2)) * pi/180;
xt(11) =  k_tilt * (x(1) - xt(1)) * pi/180;

% Target Rates (The change needed to reach those angles)
% Use a higher gain (e.g., 10 or 15) to reduce the calculation lag
k_tilt_rate=3.0;
xt(4) = k_tilt_rate * (xt(10) - x(10)); 
xt(5) = k_tilt_rate * (xt(11) - x(11));

%heading and bearing (yaw angle)
    %bearing based on the waypoint
xt(12)=atan2(xp(8)-x(8),xp(7)-x(7)); 
    %yaw rate based on shortest way
k_yaw=2.0;
xt(6)=k_yaw*angdiff(x(12), xt(12));
end