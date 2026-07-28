clear all;
close all;

% =========================================================================
% 1. LOAD FLIGHT DATA & EXTRACTIONS
% =========================================================================
bb = load('black_box.mat');

t_bb  = bb.t_bb;
x_bb  = bb.x_bb;
xt_bb = bb.xt_bb;
xp_bb = bb.xp_bb;

% Unpack using the corrected struct names
wind_data = bb.wind_bb;
turb_data = bb.turb_bb;

% Define constants
RPM_max = 8500; 
rad2deg = 180/pi;
rads2rpm = 30/pi; 

% High-contrast visualization parameters
lw = 1.5;             
c_act  = [0, 0.3, 0.7];   % Deep blue
c_tgt  = [0.85, 0, 0];    % Crimson red
c_plan = [0.15, 0.15, 0.15]; % Dark charcoal
c_env  = [0.46, 0.67, 0.18]; % High-contrast environmental green

%% === FIGURE 1: Longitudinal Dynamics ===
figure(1);
tlay1 = tiledlayout(3, 1, 'TileSpacing', 'compact');

nexttile;
h1 = plot(t_bb, x_bb(1,:), 'Color', c_act, 'LineWidth', lw); hold on; 
h2 = plot(t_bb, xt_bb(1,:), '--', 'Color', c_tgt, 'LineWidth', lw); hold off;
title('Longitudinal Linear Velocity'); ylabel('v_{\xi} (m/s)'); grid on;

nexttile;
plot(t_bb, rad2deg*x_bb(5,:), 'Color', c_act, 'LineWidth', lw); hold on; 
plot(t_bb, rad2deg*xt_bb(5,:), '--', 'Color', c_tgt, 'LineWidth', lw); hold off;
title('Pitch Rate'); ylabel('\omega_{\eta} (deg/s)'); grid on;

nexttile;
plot(t_bb, rad2deg*x_bb(11,:), 'Color', c_act, 'LineWidth', lw); hold on; 
plot(t_bb, rad2deg*xt_bb(11,:), '--', 'Color', c_tgt, 'LineWidth', lw); hold off;
title('Pitch Angle'); ylabel('\theta (deg)'); grid on;

xlabel(tlay1, 'Time (s)');
lgd1 = legend([h1, h2], {'Actual State', 'Target Command'}, 'Orientation', 'horizontal');
lgd1.Layout.Tile = 'north';

%% === FIGURE 2: Lateral Dynamics ===
figure(2);
tlay2 = tiledlayout(3, 1, 'TileSpacing', 'compact');

nexttile;
h1 = plot(t_bb, x_bb(2,:), 'Color', c_act, 'LineWidth', lw); hold on; 
h2 = plot(t_bb, xt_bb(2,:), '--', 'Color', c_tgt, 'LineWidth', lw); hold off;
title('Lateral Linear Velocity'); ylabel('v_{\eta} (m/s)'); grid on;

nexttile;
plot(t_bb, rad2deg*x_bb(4,:), 'Color', c_act, 'LineWidth', lw); hold on; 
plot(t_bb, rad2deg*xt_bb(4,:), '--', 'Color', c_tgt, 'LineWidth', lw); hold off;
title('Roll Rate'); ylabel('\omega_{\xi} (deg/s)'); grid on;

nexttile;
plot(t_bb, rad2deg*x_bb(10,:), 'Color', c_act, 'LineWidth', lw); hold on; 
plot(t_bb, rad2deg*xt_bb(10,:), '--', 'Color', c_tgt, 'LineWidth', lw); hold off;
title('Roll Angle'); ylabel('\phi (deg)'); grid on;

xlabel(tlay2, 'Time (s)');
lgd2 = legend([h1, h2], {'Actual State', 'Target Command'}, 'Orientation', 'horizontal');
lgd2.Layout.Tile = 'north';

%% === FIGURE 3: Spatial Positions (3D Trajectory) ===
figure(3);
tlay3 = tiledlayout(3, 1, 'TileSpacing', 'compact');

nexttile;
h1 = plot(t_bb, x_bb(7,:), 'Color', c_act, 'LineWidth', lw); hold on; 
h2 = plot(t_bb, xt_bb(7,:), '--', 'Color', c_tgt, 'LineWidth', lw); 
h3 = plot(t_bb, xp_bb(7,:), ':', 'Color', c_plan, 'LineWidth', lw); hold off;
title('Position X'); ylabel('x (m)'); grid on;

nexttile;
plot(t_bb, x_bb(8,:), 'Color', c_act, 'LineWidth', lw); hold on; 
plot(t_bb, xt_bb(8,:), '--', 'Color', c_tgt, 'LineWidth', lw); 
plot(t_bb, xp_bb(8,:), ':', 'Color', c_plan, 'LineWidth', lw); hold off;
title('Position Y'); ylabel('y (m)'); grid on;

nexttile;
plot(t_bb, x_bb(9,:), 'Color', c_act, 'LineWidth', lw); hold on; 
plot(t_bb, xt_bb(9,:), '--', 'Color', c_tgt, 'LineWidth', lw); 
plot(t_bb, xp_bb(9,:), ':', 'Color', c_plan, 'LineWidth', lw); hold off;
title('Position Z'); ylabel('z (m)'); grid on;

xlabel(tlay3, 'Time (s)');
lgd3 = legend([h1, h2, h3], {'Actual State', 'Target Command', 'Planned Path'}, 'Orientation', 'horizontal');
lgd3.Layout.Tile = 'north';

%% === FIGURE 4: Heading & Vertical Dynamics ===
figure(4);
tlay4 = tiledlayout(3, 1, 'TileSpacing', 'compact');

nexttile;
h1 = plot(t_bb, x_bb(3,:), 'Color', c_act, 'LineWidth', lw); hold on; 
h2 = plot(t_bb, xt_bb(3,:), '--', 'Color', c_tgt, 'LineWidth', lw); hold off;
title('Vertical Velocity'); ylabel('v_{\zeta} (m/s)'); grid on;

nexttile;
plot(t_bb, rad2deg*x_bb(6,:), 'Color', c_act, 'LineWidth', lw); hold on; 
plot(t_bb, rad2deg*xt_bb(6,:), '--', 'Color', c_tgt, 'LineWidth', lw); hold off;
title('Yaw Rate'); ylabel('\omega_{\zeta} (deg/s)'); grid on;

nexttile;
yaw_actual_deg = mod(rad2deg * x_bb(12,:), 360);
yaw_target_deg = mod(rad2deg * xt_bb(12,:), 360);
jump_actual = abs(diff(yaw_actual_deg)) > 180;
jump_target = abs(diff(yaw_target_deg)) > 180;
yaw_actual_plot = yaw_actual_deg;
yaw_target_plot = yaw_target_deg;
yaw_actual_plot([false, jump_actual]) = NaN;
yaw_target_plot([false, jump_target]) = NaN;

plot(t_bb, yaw_actual_plot, 'Color', c_act, 'LineWidth', lw); hold on; 
plot(t_bb, yaw_target_plot, '--', 'Color', c_tgt, 'LineWidth', lw); hold off;
title('Yaw Angle (Heading)'); ylabel('\psi (deg)'); grid on;
ylim([0 360]); yticks(0:90:360);

xlabel(tlay4, 'Time (s)');
lgd4 = legend([h1, h2], {'Actual State', 'Target Command'}, 'Orientation', 'horizontal');
lgd4.Layout.Tile = 'north';

%% === FIGURE 5: Actuator Rotor Speeds ===
figure(5);
tlay5 = tiledlayout(4, 1, 'TileSpacing', 'compact');

nexttile; plot(t_bb, x_bb(13,:) * rads2rpm, 'Color', c_act, 'LineWidth', lw); title('Rotor 1 Speed'); ylabel('\omega_1 (RPM)'); ylim([0 RPM_max]); grid on;
nexttile; plot(t_bb, x_bb(14,:) * rads2rpm, 'Color', c_act, 'LineWidth', lw); title('Rotor 2 Speed'); ylabel('\omega_2 (RPM)'); ylim([0 RPM_max]); grid on;
nexttile; plot(t_bb, x_bb(15,:) * rads2rpm, 'Color', c_act, 'LineWidth', lw); title('Rotor 3 Speed'); ylabel('\omega_3 (RPM)'); ylim([0 RPM_max]); grid on;
nexttile; plot(t_bb, x_bb(16,:) * rads2rpm, 'Color', c_act, 'LineWidth', lw); title('Rotor 4 Speed'); ylabel('\omega_4 (RPM)'); ylim([0 RPM_max]); grid on;

xlabel(tlay5, 'Time (s)');

%% === FIGURE 6: Wind Velocity Components ===
figure(6);
tlay6 = tiledlayout(3, 1, 'TileSpacing', 'compact');

nexttile;
plot(t_bb, wind_data(1,:), 'Color', c_env, 'LineWidth', lw);
title('Wind Velocity X (North Component)'); ylabel('w_x (m/s)'); grid on;

nexttile;
plot(t_bb, wind_data(2,:), 'Color', c_env, 'LineWidth', lw);
title('Wind Velocity Y (West Component)'); ylabel('w_y (m/s)'); grid on;

nexttile;
plot(t_bb, wind_data(3,:), 'Color', c_env, 'LineWidth', lw);
title('Wind Velocity Z (Upward Component)'); ylabel('w_z (m/s)'); grid on;

xlabel(tlay6, 'Time (s)');

%% === FIGURE 7: Atmospheric Turbulence Fluctuations ===
figure(7);
tlay7 = tiledlayout(3, 1, 'TileSpacing', 'compact');

nexttile;
plot(t_bb, turb_data(1,:), 'Color', c_env, 'LineWidth', lw);
title('Turbulence Linear Fluctuation X'); ylabel('v_{turb,x} (m/s)'); grid on;

nexttile;
plot(t_bb, turb_data(2,:), 'Color', c_env, 'LineWidth', lw);
title('Turbulence Linear Fluctuation Y'); ylabel('v_{turb,y} (m/s)'); grid on;

nexttile;
plot(t_bb, turb_data(3,:), 'Color', c_env, 'LineWidth', lw);
title('Turbulence Linear Fluctuation Z'); ylabel('v_{turb,z} (m/s)'); grid on;

xlabel(tlay7, 'Time (s)');

% =========================================================================
% GLOBAL AXES ADJUSTMENTS (Stopped at 25 s)
% =========================================================================
all_axes = findobj(groot, 'Type', 'axes');

t_start = 0;
t_end   = 25; 
t_step  = 5;  

% Apply strict bounds and grid density across all 7 figures simultaneously
set(all_axes, 'XLim', [t_start, t_end], ...
              'XTick', t_start:t_step:t_end, ...
              'XGrid', 'on', 'YGrid', 'on');

linkaxes(all_axes, 'x');

% =========================================================================
% AUTOMATIC HIGH-RESOLUTION FIGURE EXPORT
% =========================================================================
fprintf('Saving telemetry and environmental dashboard figures...\n');
opts = {'Resolution', 300};

if ishandle(1), exportgraphics(figure(1), 'longitudinal_motion_4.png', opts{:}); end
if ishandle(2), exportgraphics(figure(2), 'lateral_motion_4.png', opts{:}); end
if ishandle(3), exportgraphics(figure(3), 'spatial_position_4.png', opts{:}); end
if ishandle(4), exportgraphics(figure(4), 'heading_vertical_4.png', opts{:}); end
if ishandle(5), exportgraphics(figure(5), 'rotor_speeds_4.png', opts{:}); end
if ishandle(6), exportgraphics(figure(6), 'wind_profiles.png', opts{:}); end
if ishandle(7), exportgraphics(figure(7), 'turbulence_profiles.png', opts{:}); end

fprintf('All 7 figures exported successfully!\n');
