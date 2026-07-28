clear all
% 1. Load flight data
bb=load('black_box.mat');

% --- Scene Setup ---
plot_dim = 20;      % 20m x 20m
f_height = 1.5;     % 1.5m tall
f_thick = 0.1;      % 10cm thickness
res = 10;           % Number of points per edge (higher = denser cloud)

% Defining the 4 walls: [dx dy dz, x y z]
% We place them at the edges of the 20x20 area
walls = [
    plot_dim, f_thick, f_height,  0, -10, f_height/2; % Front
    plot_dim, f_thick, f_height,  0,  10, f_height/2; % Back
    f_thick, plot_dim, f_height, -10,  0, f_height/2; % Left
    f_thick, plot_dim, f_height,  10,  0, f_height/2  % Right
];

radar_points = []; % Master Nx3 matrix for radar imitation

% --- Visualization ---
figure; hold on; grid on; axis equal; view(3);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Drone Radar Point Cloud Simulation');
% --- Natural Visualization Pack ---
shading interp;
lighting phong;
camlight('headlight');
camproj('perspective');
material dull;
set(gcf, 'Color', [0.9 0.9 1]); % Delikatnie błękitne tło (niebo)
axis off; % Ukrywa techniczne osie XYZ dla lepszego efektu
light('Position', [10, 10, 20])

% --- Fix for Clipping and Zooming Issues ---
axis tight;            % Najpierw dopasuj do obiektów
ax = gca;
ax.Clipping = 'off';   % WYŁĄCZA UCINANIE obiektów wystających poza osie
camproj('perspective');
camva(4);             % Ustawia stały kąt widzenia (Field of View) - im mniejszy, tym większy zoom
grid on;


% --- Module: Fence ---
for i = 1:size(walls, 1)
    sz = walls(i, 1:3);
    pos = walls(i, 4:6);
    
    % Create local grid
    v = linspace(-1, 1, res);
    [m1, m2] = meshgrid(v, v);
    
    % Define 6 faces (simplified: no rotation for basic fence)
    faces = {
        m1*sz(1)/2, m2*sz(2)/2, -sz(3)/2*ones(res); % Bottom
        m1*sz(1)/2, m2*sz(2)/2,  sz(3)/2*ones(res); % Top
        m1*sz(1)/2, -sz(2)/2*ones(res), m2*sz(3)/2; % Front
        m1*sz(1)/2,  sz(2)/2*ones(res), m2*sz(3)/2; % Back
        -sz(1)/2*ones(res), m1*sz(2)/2, m2*sz(3)/2; % Left
         sz(1)/2*ones(res), m1*sz(2)/2, m2*sz(3)/2  % Right
    };

    for f = 1:6
        fX = faces{f,1} + pos(1);
        fY = faces{f,2} + pos(2);
        fZ = faces{f,3} + pos(3);
        
        % Convert face to Nx3 and add to master matrix
        face_pts = [fX(:), fY(:), fZ(:)];
        radar_points = [radar_points; face_pts];
        
        % Visualization
        surf(fX, fY, fZ, 'FaceColor', [0.5 0.5 0.5], 'EdgeColor', 'k');
    end
end


% --- Module: Grass with Gray Driveway (Fixed) ---
res_ground = 40; 
gv = linspace(-10, 10, res_ground);
[GX, GY] = meshgrid(gv, gv);
GZ = zeros(size(GX));

% Define driveway area (from house X=-2 to fence X=-10, width Y from -1.5 to 1.5)
is_driveway = (GX <= -1.5 & GX >= -10 & GY >= -2 & GY <= 2);

% 1. RADAR POINTS (Collect all ground points)
radar_points = [radar_points; GX(:), GY(:), GZ(:)];

% 2. VISUALIZATION
% We draw the whole ground green first
surf(GX, GY, GZ, 'FaceColor', [0.3 0.7 0.3], 'EdgeColor', [0.5 0.5 0.5]);

% We overlay the driveway part with gray color
% We use a small Z-offset (0.001) to avoid "z-fighting" (flickering of colors)
hold on;
GX_drive = GX; 
GY_drive = GY;
GZ_drive = GZ + 0.001; 
% Masking out everything that is NOT driveway
GX_drive(~is_driveway) = NaN; 

surf(GX_drive, GY_drive, GZ_drive, 'FaceColor', [0.5 0.5 0.5], 'EdgeColor', 'k');


% --- Module: Fixed House (Correct Gables & Windows) ---
h_p = [0, 0];    % Base center position
h_w = 4; h_l = 5; h_h = 3; 
r_h = 1.5; h_res = 10;

% 1. HOUSE BODY
[hm1, hm2] = meshgrid(linspace(-1, 1, h_res));
b_fcs = {hm1*h_w/2, hm2*h_l/2, -h_h/2*ones(h_res); ... % Floor
         hm1*h_w/2, hm2*h_l/2,  h_h/2*ones(h_res); ... % Ceiling
         hm1*h_w/2, -h_l/2*ones(h_res), hm2*h_h/2; ... % Y = -2
         hm1*h_w/2,  h_l/2*ones(h_res), hm2*h_h/2; ... % Y = 2
         -h_w/2*ones(h_res), hm1*h_l/2, hm2*h_h/2; ... % X = -2 (FRONT)
          h_w/2*ones(h_res), hm1*h_l/2, hm2*h_h/2};    % X = 2

for f = 1:6
    X = b_fcs{f,1} + h_p(1); Y = b_fcs{f,2} + h_p(2); Z = b_fcs{f,3} + h_h/2;
    radar_points = [radar_points; X(:), Y(:), Z(:)];
    surf(X, Y, Z, 'FaceColor', [0.8 0.8 0.8], 'EdgeColor', 'none');
end

% 2. ROOF SLOPES (Ridge along Y axis)
% Parametry
overhang = 0.5;
pitch_angle = atan(r_h / (h_w/2)); % Stały kąt nachylenia oryginalnego dachu

% 1. Rozszerzenie wzdłuż osi Y (przód/tył)
[ry, rs] = meshgrid(linspace(-h_l/2 - overhang, h_l/2 + overhang, h_res), linspace(0, 1, h_res)); 

% 2. X: Dach musi zacząć się od h_w/2 + overhang i kończyć na 0
rx_s = (h_w/2 + overhang) * (1 - rs);

% 3. Z: Wysokość musi być liczona od kalenicy w dół, zachowując kąt
% Kalenica (rs=1) jest zawsze na h_h + r_h
% Okap (rs=0) schodzi niżej niż h_h, bo dach jest dłuższy
z_top = h_h + r_h;
rz_s = z_top - rx_s * tan(pitch_angle);

% Wyświetlanie
surf(rx_s + h_p(1), ry + h_p(2), rz_s, 'FaceColor', [0.6 0.1 0.1], 'EdgeColor', 'k');
hold on;
surf(-rx_s + h_p(1), ry + h_p(2), rz_s, 'FaceColor', [0.6 0.1 0.1], 'EdgeColor', 'k');


% 3. ROOF GABLES (Triangular sides at Y = -2 and Y = 2)
[gx, gz] = meshgrid(linspace(-h_w/2, h_w/2, h_res), linspace(0, r_h, h_res));
mask = gz <= (r_h - (r_h/(h_w/2)) * abs(gx)); % Triangular mask

for side_y = [-h_l/2, h_l/2]
    curr_Y = (side_y + h_p(2)) * ones(sum(mask(:)), 1);
    radar_points = [radar_points; gx(mask)+h_p(1), curr_Y, gz(mask)+h_h];
    
    % Visualization for Gables
    fill3(gx(1,:)+h_p(1), side_y*ones(1,h_res)+h_p(2), ...
          (abs(gx(1,:)) <= h_w/2).*(h_h + r_h - (r_h/(h_w/2))*abs(gx(1,:))), [0.6 0.1 0.1], 'EdgeColor', 'none');
end

% 4. FRONT WALL (X = -2.01): Window (Left) and Door (Right)
[dy_win, dz_win] = meshgrid(linspace(0.4, 1.4, 8), linspace(1.2, 2.2, 8)); % Window Left
radar_points = [radar_points; (-h_w/2-0.01+h_p(1))*ones(numel(dy_win),1), dy_win(:)+h_p(2), dz_win(:)];
surf((-h_w/2-0.01+h_p(1))*ones(size(dy_win)), dy_win+h_p(2), dz_win, 'FaceColor', [0.5 0.8 1]);

[dy_door, dz_door] = meshgrid(linspace(-1.4, -0.4, 8), linspace(0, 2, 8)); % Door Right
radar_points = [radar_points; (-h_w/2-0.01+h_p(1))*ones(numel(dy_door),1), dy_door(:)+h_p(2), dz_door(:)];
surf((-h_w/2-0.01+h_p(1))*ones(size(dy_door)), dy_door+h_p(2), dz_door, 'FaceColor', [0.4 0.2 0.1]);

% 5. SYMMETRICAL WINDOWS (Other walls)
% Back wall (X = 2.01)
[wy2, wz2] = meshgrid(linspace(-0.5, 0.5, 8), linspace(1.2, 2.2, 8));
radar_points = [radar_points; (h_w/2+0.01+h_p(1))*ones(numel(wy2),1), wy2(:)+h_p(2), wz2(:)];
surf((h_w/2+0.01+h_p(1))*ones(size(wy2)), wy2+h_p(2), wz2, 'FaceColor', [0.5 0.8 1]);

% Side walls (Y = -2.01 and Y = 2.01)
[wx1, wz1] = meshgrid(linspace(-0.5, 0.5, 8), linspace(1.2, 2.2, 8));
for sy = [-h_l/2-0.01, h_l/2+0.01]
    radar_points = [radar_points; wx1(:)+h_p(1), (sy+h_p(2))*ones(numel(wx1),1), wz1(:)];
    surf(wx1+h_p(1), (sy+h_p(2))*ones(size(wx1)), wz1, 'FaceColor', [0.5 0.8 1]);
end

% 6. CHIMNEY (Gray Cuboid)
c_p = [1, 1, 4.0]; % Pozycja komina na dachu
c_d = [0.5, 0.5, 1.2]; % Wymiary komina
[cm1, cm2] = meshgrid(linspace(-1, 1, 5));

% Definicja 5 ścian komina (bez spodu, który jest w dachu)
c_fcs = {cm1*c_d(1)/2, cm2*c_d(2)/2,  c_d(3)/2*ones(5); ... % góra
         cm1*c_d(1)/2, -c_d(2)/2*ones(5), cm2*c_d(3)/2; ... % przód
         cm1*c_d(1)/2,  c_d(2)/2*ones(5), cm2*c_d(3)/2; ... % tył
         -c_d(1)/2*ones(5), cm1*c_d(2)/2, cm2*c_d(3)/2; ... % lewo
          c_d(1)/2*ones(5), cm1*c_d(2)/2, cm2*c_d(3)/2};    % prawo

for f = 1:5
    cX = c_fcs{f,1} + c_p(1);
    cY = c_fcs{f,2} + c_p(2);
    cZ = c_fcs{f,3} + c_p(3);
    
    % Zapis punktów i wizualizacja w kolorze ciemnoszarym
    radar_points = [radar_points; cX(:), cY(:), cZ(:)];
    surf(cX, cY, cZ, 'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'k');
end


% --- Module: Safe Car (Guaranteed Concatenation) ---
c_pos = [-6, 0.9, 0.3]; 
c_res = 10;
edge_col = 'k'; 

% 1. CHASSIS
c_dims = [4.0, 1.8, 0.8]; 
[cm1, cm2] = meshgrid(linspace(-1, 1, c_res));
c_fcs = {cm1*c_dims(1)/2, cm2*c_dims(2)/2, -c_dims(3)/2*ones(c_res); ... 
         cm1*c_dims(1)/2, cm2*c_dims(2)/2,  c_dims(3)/2*ones(c_res); ... 
         cm1*c_dims(1)/2, -c_dims(2)/2*ones(c_res), cm2*c_dims(3)/2; ... 
         cm1*c_dims(1)/2,  c_dims(2)/2*ones(c_res), cm2*c_dims(3)/2; ...
         -c_dims(1)/2*ones(c_res), cm1*c_dims(2)/2, cm2*c_dims(3)/2; ...
          c_dims(1)/2*ones(c_res), cm1*c_dims(2)/2, cm2*c_dims(3)/2};

for f = 1:6
    X = c_fcs{f,1} + c_pos(1); Y = c_fcs{f,2} + c_pos(2); Z = c_fcs{f,3} + c_pos(3) + 0.4;
    % Wymuszenie formatu Nx3
    radar_points = [radar_points; X(:), Y(:), Z(:)];
    surf(X, Y, Z, 'FaceColor', [0.1 0.3 0.6], 'EdgeColor', edge_col);
end

% 2. CABIN
cab_dims = [2.0, 1.5, 0.6];
cab_p = c_pos + [0, 0, 1.1]; 
[cbm1, cbm2] = meshgrid(linspace(-1, 1, c_res));
cab_fcs = {cbm1*cab_dims(1)/2, cbm2*cab_dims(2)/2,  cab_dims(3)/2*ones(c_res); ... 
           cbm1*cab_dims(1)/2, -cab_dims(2)/2*ones(c_res), cbm2*cab_dims(3)/2; ... 
           cbm1*cab_dims(1)/2,  cab_dims(2)/2*ones(c_res), cbm2*cab_dims(3)/2; ...
           -cab_dims(1)/2*ones(c_res), cbm1*cab_dims(2)/2, cbm2*cab_dims(3)/2; ...
            cab_dims(1)/2*ones(c_res), cbm1*cab_dims(2)/2, cbm2*cab_dims(3)/2};

for f = 1:5
    X = cab_fcs{f,1} + cab_p(1); Y = cab_fcs{f,2} + cab_p(2); Z = cab_fcs{f,3} + cab_p(3);
    radar_points = [radar_points; X(:), Y(:), Z(:)];
    surf(X, Y, Z, 'FaceColor', [0.5 0.8 1.0], 'EdgeColor', edge_col, 'FaceAlpha', 0.6);
end

% 3. CYLINDRICAL WHEELS
w_rad = 0.3; w_wid = 0.25; w_res = 20;
w_offs = [-1.2, -0.9, 0; 1.2, -0.9, 0; -1.2, 0.9, 0; 1.2, 0.9, 0];

for i = 1:4
    [z_cyl, x_cyl, y_cyl] = cylinder(w_rad, w_res); 
    y_cyl = y_cyl * w_wid - w_wid/2; 
    wp = c_pos + w_offs(i,:);
    fX = x_cyl + wp(1); fY = y_cyl + wp(2); fZ = z_cyl + wp(3);
    
    radar_points = [radar_points; fX(:), fY(:), fZ(:)];
    surf(fX, fY, fZ, 'FaceColor', [0.1 0.1 0.1], 'EdgeColor', edge_col);
    
    % Side Caps
    [R_c, T_c] = meshgrid(linspace(0, w_rad, 5), linspace(0, 2*pi, w_res+1));
    cap_x = R_c.*cos(T_c); cap_z = R_c.*sin(T_c);
    for side = [-w_wid/2, w_wid/2]
        cX = cap_x + wp(1); 
        cY = ones(size(cX)) * (side + wp(2)); % Macierz o tym samym rozmiarze co cX
        cZ = cap_z + wp(3);
        radar_points = [radar_points; cX(:), cY(:), cZ(:)];
        surf(cX, cY, cZ, 'FaceColor', [0.1 0.1 0.1], 'EdgeColor', edge_col);
    end
end


% --- Module: Black Bicycle (Wheels Aligned with Frame) ---
b_pos = [-6, 6, 0]; 
angle_deg = -45; 
R_rot = [cosd(angle_deg), -sind(angle_deg); sind(angle_deg), cosd(angle_deg)];
edge_col = [0.5 0.5 0.5]; 

% 1. WHEELS (Aligned with frame plane)
w_rad = 0.35; w_thick = 0.04; res_w = 16;
[wX_cyl, wZ_cyl, wY_cyl] = cylinder(w_rad, res_w); 
wY_cyl = wY_cyl * w_thick - w_thick/2; % Szerokość koła (oś Y)

wheel_offsets = [-0.6, 0.6]; % Tył i przód
for off = wheel_offsets
    % Lokalna pozycja: przesuń koło wzdłuż ramy (oś X) i unieś (oś Z)
    loc_X = wX_cyl + off;
    loc_Y = wY_cyl;
    loc_Z = wZ_cyl + w_rad;
    
    % Rotacja współrzędnych XY o 45 stopni
    finalX = loc_X; finalY = loc_Y;
    for r = 1:size(loc_X,1)
        for c = 1:size(loc_X,2)
            pt = R_rot * [loc_X(r,c); loc_Y(r,c)];
            finalX(r,c) = pt(1) + b_pos(1);
            finalY(r,c) = pt(2) + b_pos(2);
        end
    end
    finalZ = loc_Z + b_pos(3);
    
    radar_points = [radar_points; finalX(:), finalY(:), finalZ(:)];
    surf(finalX, finalY, finalZ, 'FaceColor', 'k', 'EdgeColor', 'k');
    
    % Boki kół (dekle) - opcjonalnie dla pełnego odbicia radaru
    theta = linspace(0, 2*pi, res_w+1);
    [R_cap, T_cap] = meshgrid(linspace(0, w_rad, 3), theta);
    capX_loc = R_cap.*cos(T_cap) + off;
    capZ_loc = R_cap.*sin(T_cap) + w_rad;
    for side = [-w_thick/2, w_thick/2]
        cX = capX_loc; cY = side * ones(size(capX_loc));
        rX = cX; rY = cY;
        for r = 1:size(cX,1), for c = 1:size(cX,2)
            pt = R_rot * [cX(r,c); cY(r,c)];
            rX(r,c) = pt(1) + b_pos(1);
            rY(r,c) = pt(2) + b_pos(2);
        end, end
        radar_points = [radar_points; rX(:), rY(:), capZ_loc(:)+b_pos(3)];
        surf(rX, rY, capZ_loc+b_pos(3), 'FaceColor', 'k', 'EdgeColor', 'k');
    end
end

% 2. FRAME (Solid Black Bars - same as before)
frame_parts = {
    [1.2, 0.05, 0.05], [0, 0, 0.45];   
    [0.05, 0.05, 0.5], [-0.3, 0, 0.4]; 
    [0.05, 0.05, 0.6], [0.5, 0, 0.4];  
    [0.05, 0.6, 0.05], [0.5, 0, 0.7];  
    [0.3, 0.2, 0.05], [-0.3, 0, 0.7]   
};

for i = 1:size(frame_parts, 1)
    d = frame_parts{i,1}; p_loc = frame_parts{i,2};
    [m1, m2] = meshgrid(linspace(-1,1,3));
    fcs = {m1*d(1)/2, m2*d(2)/2, -d(3)/2*ones(3); m1*d(1)/2, m2*d(2)/2, d(3)/2*ones(3); ...
           m1*d(1)/2, -d(2)/2*ones(3), m2*d(3)/2; m1*d(1)/2, d(2)/2*ones(3), m2*d(3)/2; ...
           -d(1)/2*ones(3), m1*d(2)/2, m2*d(3)/2; d(1)/2*ones(3), m1*d(2)/2, m2*d(3)/2};
    for f = 1:6
        Xl = fcs{f,1} + p_loc(1); Yl = fcs{f,2} + p_loc(2); Zl = fcs{f,3} + p_loc(3);
        Xr = Xl; Yr = Yl;
        for r = 1:3, for c = 1:3
            pt = R_rot * [Xl(r,c); Yl(r,c)];
            Xr(r,c) = pt(1) + b_pos(1);
            Yr(r,c) = pt(2) + b_pos(2);
        end, end
        Zr = Zl + b_pos(3);
        radar_points = [radar_points; Xr(:), Yr(:), Zr(:)];
        surf(Xr, Yr, Zr, 'FaceColor', 'k', 'EdgeColor', 'k');
    end
end


% --- Module: Three Tuyas (Fixed Concatenation) ---
tuya_positions = [-3, 9, 0; 0, 9, 0; 3, 9, 0];
tuya_h = 4; tuya_w = 1; res_tuya = 8;
edge_col = [0, 0.4, 0]; 

for i = 1:size(tuya_positions, 1)
    t_p = tuya_positions(i,:);
    [tx, ts] = meshgrid(linspace(-tuya_w/2, tuya_w/2, res_tuya), linspace(0, 1, res_tuya));
    ty_slope = (tuya_w/2) * (1 - ts); 
    tz = tuya_h * ts;                

    for s = [1, -1]
        % Ściany Y
        fX = tx + t_p(1); fY = s*ty_slope + t_p(2); fZ = tz + t_p(3);
        % FIX: Używamy [:] i łączymy poziomo przed dodaniem pionowym
        new_pts = [fX(:), fY(:), fZ(:)]; 
        radar_points = [radar_points; new_pts];
        surf(fX, fY, fZ, 'FaceColor', [0.1 0.4 0.1], 'EdgeColor', edge_col);
        
        % Ściany X
        fX_rot = s*ty_slope + t_p(1); fY_rot = tx + t_p(2); fZ_rot = tz + t_p(3);
        new_pts_rot = [fX_rot(:), fY_rot(:), fZ_rot(:)];
        radar_points = [radar_points; new_pts_rot];
        surf(fX_rot, fY_rot, fZ_rot, 'FaceColor', [0.1 0.4 0.1], 'EdgeColor', edge_col);
    end
end


% --- Module: Updated Clothesline and Towels (Correct Orientation) ---
% Start: middle tuya (0, 9, 1.5), End: house back wall (0, 2, 1.5)
p1 = [0, 9, 1.5]; 
p2 = [0, 2, 1.5]; 

% 1. ROPE
rope_res = 20;
t_line = linspace(0, 1, rope_res);
rx = p1(1) + t_line*(p2(1)-p1(1));
ry = p1(2) + t_line*(p2(2)-p1(2));
rz = p1(3) + t_line*(p2(3)-p1(3));

radar_points = [radar_points; rx', ry', rz'];
plot3(rx, ry, rz, 'Color', [0.4 0.3 0.2], 'LineWidth', 1.5);

% 2. TOWELS (Hanging along the Y-Z plane)
towel_len = 1; % długość ręcznika wzdłuż liny
towel_h = 1.0;   % wysokość ręcznika (jak nisko zwisa)
t_res = 8;
edge_col = [0.5 0.5 0.5];

% Pozycje na linie (40% i 60% długości)
t_pos = [0.4, 0.6]; 

for i = 1:2
    % Środek ręcznika na linie
    tx_c = p1(1) + t_pos(i)*(p2(1)-p1(1));
    ty_c = p1(2) + t_pos(i)*(p2(2)-p1(2));
    tz_c = p1(3) + t_pos(i)*(p2(3)-p1(3));
    
    % Siatka w płaszczyźnie Y-Z (wzdłuż liny)
    [tY, tZ] = meshgrid(linspace(-towel_len/2, towel_len/2, t_res), ...
                        linspace(-towel_h, 0, t_res));
    
    finalX = ones(size(tY)) * tx_c; % Stałe X (lina jest w X=0)
    finalY = tY + ty_c;            % Rozpiętość wzdłuż liny (oś Y)
    finalZ = tZ + tz_c;            % Zwis w dół (oś Z)
    
    % Zapis punktów i wizualizacja
    new_towel_pts = [finalX(:), finalY(:), finalZ(:)];
    radar_points = [radar_points; new_towel_pts];
    
    surf(finalX, finalY, finalZ, 'FaceColor', 'w', 'EdgeColor', edge_col);
end


% --- Module: Cylindrical Swimming Pool ---
pool_pos = [6, 6, 0];  % Pozycja środka basenu na działce
pool_diam = 5.0;       % Średnica 5 m
pool_h = 1.2;          % Wysokość 1.2 m
pool_res = 30;         % Rozdzielczość cylindra
edge_col = [0.5 0.5 0.5]; 

% 1. POOL WALLS (Cylinder)
height_res = 15;       % MODIFICATION: Liczba punktów/warstw w pionie dla radaru
r_vector = ones(height_res, 1) * (pool_diam/2); 
[pX, pY, pZ] = cylinder(r_vector, pool_res);    % Generuje pasujące wymiary macierzy
pZ = pZ * pool_h; % Skalowanie do wysokości basenu

% Przesunięcie do pozycji końcowej
X_wall = pX + pool_pos(1);
Y_wall = pY + pool_pos(2);
Z_wall = pZ + pool_pos(3);

% Zapis punktów ścian do macierzy radarowej
radar_points = [radar_points; X_wall(:), Y_wall(:), Z_wall(:)];

% Wizualizacja ścian basenu (zewnętrzna powłoka)
surf(X_wall, Y_wall, Z_wall, 'FaceColor', [0.8 0.8 0.8], 'EdgeColor', edge_col);

% 2. WATER SURFACE (Circular top)
theta = linspace(0, 2*pi, pool_res + 1);
rho = linspace(0, pool_diam/2, 10);
[R, T] = meshgrid(rho, theta);

X_water = R.*cos(T) + pool_pos(1);
Y_water = R.*sin(T) + pool_pos(2);
Z_water = ones(size(X_water)) * (pool_h - 0.1) + pool_pos(3); % Woda lekko poniżej krawędzi

% Zapis punktów tafli wody
radar_points = [radar_points; X_water(:), Y_water(:), Z_water(:)];

% Wizualizacja wody
surf(X_water, Y_water, Z_water, 'FaceColor', [0 0.5 1], 'FaceAlpha', 0.6, 'EdgeColor', edge_col);




% --- Module: Badminton Net (Along X Axis) ---
net_pos = [6, 0, 0];   % Środek siatki (ustawiony np. na trawniku)
net_width = 6.0;        % Szerokość siatki (rozpiętość w osi X)
net_h_top = 1.55;       % Wysokość słupków
net_mesh_h = 0.76;      % Wysokość samej siatki (od góry w dół)
edge_col = [0.5 0.5 0.5];
res_net = 15;           

% 1. POSTS (Two thin vertical cuboids at X = +/- 3)
post_thick = 0.05;
post_offs = [-net_width/2, net_width/2];

for off = post_offs
    [pm1, pm2] = meshgrid(linspace(-1,1,3));
    % Słupek jako cienki prostopadłościan
    p_fcs = {pm1*post_thick/2, pm2*post_thick/2, -net_h_top/2*ones(3); ...
             pm1*post_thick/2, pm2*post_thick/2,  net_h_top/2*ones(3); ...
             pm1*post_thick/2, -post_thick/2*ones(3), pm2*net_h_top/2; ...
             pm1*post_thick/2,  post_thick/2*ones(3), pm2*net_h_top/2; ...
             -post_thick/2*ones(3), pm1*post_thick/2, pm2*net_h_top/2; ...
              post_thick/2*ones(3), pm1*post_thick/2, pm2*net_h_top/2};
    
    for f = 1:6
        X = p_fcs{f,1} + net_pos(1) + off; % Słupki przesunięte w osi X
        Y = p_fcs{f,2} + net_pos(2);
        Z = p_fcs{f,3} + net_pos(3) + net_h_top/2;
        radar_points = [radar_points; X(:), Y(:), Z(:)];
        surf(X, Y, Z, 'FaceColor', [0.2 0.2 0.2], 'EdgeColor', edge_col);
    end
end

% 2. MESH (The actual net in X-Z plane)
% Tworzymy pionową płaszczyznę w osi X-Z
[mX, mZ] = meshgrid(linspace(-net_width/2, net_width/2, res_net*2), ...
                    linspace(net_h_top - net_mesh_h, net_h_top, res_net));

mY = ones(size(mX)) * net_pos(2);
mX = mX + net_pos(1);
mZ = mZ + net_pos(3);

% Zapis punktów siatki
radar_points = [radar_points; mX(:), mY(:), mZ(:)];

% Wizualizacja siatki
surf(mX, mY, mZ, 'FaceColor', [0.8 0.8 0.2], 'FaceAlpha', 0.3, 'EdgeColor', edge_col);

% 3. TOP TAPE (Biała taśma na górze siatki w osi X)
[tX, tY] = meshgrid(linspace(-net_width/2, net_width/2, res_net*2), ...
                    linspace(-0.03, 0.03, 2));
tZ = ones(size(tX)) * net_h_top + net_pos(3);
finalTX = tX + net_pos(1);
finalTY = tY + net_pos(2);

radar_points = [radar_points; finalTX(:), finalTY(:), tZ(:)];
surf(finalTX, finalTY, tZ, 'FaceColor', 'w', 'EdgeColor', edge_col);


% --- Module: Half-Pipe Greenhouse ---
gh_pos = [4, -6, 0];  % Pozycja środka podstawy szklarni
gh_len = 5.0;          % Długość szklarni (wzdłuż osi X)
gh_rad = 2.0;          % Promień łuku (szerokość to 2*rad = 4m)
gh_res = 15;           % Rozdzielczość siatki
edge_col = [0.5 0.5 0.5];

% 1. ARC SURFACE (Half-cylinder)
% Tworzymy siatkę kątową od 0 do pi (półkole)
theta = linspace(0, pi, gh_res);
x_val = linspace(-gh_len/2, gh_len/2, gh_res);
[Theta, X_local] = meshgrid(theta, x_val);

% Współrzędne lokalne łuku (łuk rozpięty w płaszczyźnie Y-Z)
% sin(Theta) daje wysokość (Z), cos(Theta) daje szerokość (Y)
Y_local = gh_rad * cos(Theta);
Z_local = gh_rad * sin(Theta);

% Przesunięcie do pozycji globalnej
finalX = X_local + gh_pos(1);
finalY = Y_local + gh_pos(2);
finalZ = Z_local + gh_pos(3);

% Zapis punktów do macierzy radarowej
radar_points = [radar_points; finalX(:), finalY(:), finalZ(:)];

% Wizualizacja szklarni (półprzezroczysta jak szkło/folia)
surf(finalX, finalY, finalZ, 'FaceColor', [0.8 1.0 1.0], 'FaceAlpha', 0.3, 'EdgeColor', edge_col);

% 2. FRAME (Opcjonalnie: wzmocnienia krawędzi łuku - przód i tył)
for end_x = [-gh_len/2, gh_len/2]
    arc_y = gh_rad * cos(theta) + gh_pos(2);
    arc_z = gh_rad * sin(theta) + gh_pos(3);
    arc_x = (end_x + gh_pos(1)) * ones(size(theta));
    
    radar_points = [radar_points; arc_x', arc_y', arc_z'];
    plot3(arc_x, arc_y, arc_z, 'Color', [0.2 0.2 0.2], 'LineWidth', 2);
end



% --- Module: Big Tree (Cylinder Trunk & Sphere Canopy) ---
tree_pos = [-6, -6, 0];   % Pozycja drzewa
trunk_rad = 0.4;        % Promień pnia
trunk_h = 2.5;          % Wysokość pnia
canopy_rad = 2.5;       % Promień korony (kuli)
res_tree = 20;          % Rozdzielczość sfery i cylindra
edge_col = [0.5 0.5 0.5];

% 1. TRUNK (Cylinder)
[tx, ty, tz] = cylinder(trunk_rad, res_tree);
tz = tz * trunk_h; % Skalowanie wysokości

% Przesunięcie i zapis punktów pnia
fX_t = tx + tree_pos(1);
fY_t = ty + tree_pos(2);
fZ_t = tz + tree_pos(3);

radar_points = [radar_points; fX_t(:), fY_t(:), fZ_t(:)];
surf(fX_t, fY_t, fZ_t, 'FaceColor', [0.4 0.2 0.1], 'EdgeColor', edge_col);

% 2. CANOPY (Sphere)
[sx, sy, sz] = sphere(res_tree);

% Skalowanie korony i przesunięcie na szczyt pnia
% Korona zaczyna się w środku kuli, więc przesuwamy ją o trunk_h + canopy_rad
fX_s = sx * canopy_rad + tree_pos(1);
fY_s = sy * canopy_rad + tree_pos(2);
fZ_s = sz * canopy_rad + tree_pos(3) + trunk_h + canopy_rad/2;

% Zapis punktów korony
radar_points = [radar_points; fX_s(:), fY_s(:), fZ_s(:)];

% Wizualizacja korony
surf(fX_s, fY_s, fZ_s, 'FaceColor', [0.1 0.5 0.1], 'EdgeColor', [0, 0.4, 0]);

%saving to a file
  save('radar_points.mat', 'radar_points');




% --- DRONE ---
drone_group = hgtransform('Parent', gca);
drone_color='w';
% Rama i parametry (bez zmian)
L = 0.35; W = 0.045; H = 0.015;
v = [-0.5 -0.5 -0.5; 0.5 -0.5 -0.5; 0.5 0.5 -0.5; -0.5 0.5 -0.5; ...
     -0.5 -0.5 0.5; 0.5 -0.5 0.5; 0.5 0.5 0.5; -0.5 0.5 0.5];
f = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];

% Rotacja ramion do układu X
theta_X = pi/4;
R_X = [cos(theta_X) -sin(theta_X) 0; sin(theta_X) cos(theta_X) 0; 0 0 1];
v_arm1 = (v .* [L W H]) * R_X';
v_arm2 = (v .* [W L H]) * R_X';
patch('Vertices', v_arm1, 'Faces', f, 'FaceColor', drone_color, 'Parent', drone_group); % Ciemnoszary
patch('Vertices', v_arm2, 'Faces', f, 'FaceColor', drone_color, 'Parent', drone_group);

% Śmigła - PRZEDNIE są BIAŁE, TYLNE są CZERWONE
[Xc, Yc, Zc] = cylinder(0.06, 20);
d = L/2 * cos(pi/4); 
pos_X = [d d; d -d; -d d; -d -d]; % 1,2 to przód (dodatnie X), 3,4 to tył

for i = 1:4
    p_color = 'w'; % Domyślnie czerwone (tył)
    if i <= 2, p_color = 'w'; end % Przednie białe
    
    surf(Xc + pos_X(i,1), Yc + pos_X(i,2), Zc*0.02 + H, ...
        'FaceColor', p_color, 'EdgeColor', 'none', 'Parent', drone_group);
end

% Kadłub
S = 0.12; H_f = 0.045;
patch('Vertices', v.*[S S H_f], 'Faces', f, 'FaceColor', drone_color, 'FaceAlpha', 0.8, 'Parent', drone_group);

% --- WYRÓŻNIENIE PRZODU: "OCZY" / KAMERA ---
[Xs, Ys, Zs] = sphere(10);
eye_size = 0.015;
% Lewe oko
surface(eye_size*Xs + S/2, eye_size*Ys + S/4, eye_size*Zs, ...
    'FaceColor', 'y', 'EdgeColor', 'none', 'Parent', drone_group); 
% Prawe oko
surface(eye_size*Xs + S/2, eye_size*Ys - S/4, eye_size*Zs, ...
    'FaceColor', 'y', 'EdgeColor', 'none', 'Parent', drone_group);

% --- PĘTLA ANIMACJI Z PEŁNĄ ROTACJĄ (3 OSIE) ---
t = linspace(0, 2*pi, 300);
radius_flight = 2;

view(3);            % Widok 3D
daspect([1 1 1]);   % Zachowanie proporcji 1:1:1
camproj('perspective'); % Perspektywa zamiast widoku ortogonalnego (wygląda naturalniej)
set(gca, 'Color', [0.95 0.95 0.95]); % Lekko szare tło zwiększa kontrast

% === PRZED PĘTLĄ (Inicjalizacja wykresu linii - wywołaj TYLKO RAZ) ===
% Tworzymy pusty obiekt linii o czerwonym kolorze i pobieramy jego uchwyt
trail_handle = plot3(NaN, NaN, NaN, 'r', 'LineWidth', 1.5); 
plan_handle = plot3(NaN, NaN, NaN, 'y', 'LineWidth', 1.5);
hold on; % Zapobiega nadpisywaniu sceny przez inne elementy graficzne


for i = 1:size(bb.x_bb, 2)
    % 1. Position
    x = bb.x_bb(7,i);
    y = -bb.x_bb(8,i);
    z = -bb.x_bb(9,i);
    
    % 2. Definition of rotations (in radians)
    yaw = -bb.x_bb(12,i);       % Rotation around Z (flight direction)
    pitch = -bb.x_bb(11,i);            % Rotation around Y (forward pitch)
    roll = bb.x_bb(10,i); % Rotation around X (dynamic side-to-side roll)
    
    % 3. Creating component matrices
    T = makehgtform('translate', [x, y, z]);
    Rz = makehgtform('zrotate', yaw);
    Ry = makehgtform('yrotate', pitch);
    Rx = makehgtform('xrotate', roll);
    
    % 4. Combining transformations (order matters!)
    % First we rotate around own axes, then we translate into space
    set(drone_group, 'Matrix', T * Rz * Ry * Rx);

    %visualizing the flight data
     % === OPTIMIZED FLIGHT PATH VISUALIZATION ===
    % Instead of redrawing from scratch using plot3, we only update the data of the existing line.
    % Corrected indices: 7->X, 8->Y, 9->Z (along with the sign inversion for Z just like in position)
    set(trail_handle, 'XData', bb.x_bb(7, 1:i), ...
                      'YData', -bb.x_bb(8, 1:i), ...
                      'ZData', -bb.x_bb(9, 1:i));
    set(plan_handle, 'XData', bb.xp_bb(7, 1:i), ...
                      'YData', -bb.xp_bb(8, 1:i), ...
                      'ZData', -bb.xp_bb(9, 1:i));
    
    az = -45 - i*0.1; % Slow camera rotation at each step
    view(az, 20);
    drawnow;
    %pause(0.05);
end
