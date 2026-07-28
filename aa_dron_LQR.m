% (C) W.J.Zagozdon


clear 

load('radar_points.mat');
n = 16;  % With engines
m = 4;  % dimension of u

%wind and turbulence
  ax_turbulence=0;
  ay_turbulence=0;
  az_turbulence=0;
  turb=[ax_turbulence; ay_turbulence; az_turbulence];

  %typical wind in Mazovia
  ax_wind=0.0;
  ay_wind=10.0;
  az_wind=0.0;
  wind=[ax_wind; ay_wind; az_wind];

%useful data
rad2deg = 180/pi;
deg2rad = pi/180;
RPM_max=8500;
mass = 1.388;
g = 9.81;

%initial state
x = zeros(n,1); %state vector
%start from the window of the house
x(7)=-3;
x(8)=-1;
x(9)=-2;
%nose of the drone rotated by 180 degrees
x(12)=pi;
ind=1; %first index for waypoint
thresh = 1.0; %threshold for a waypoint
num_wpts = 8;
xp = x; %plan vector starting from the initial position
xt = x; %trajectory vector
x0 = x;
t0=0.0;
%initial input
u = [0.0;0.0;0.0;0.0];
% 1. Calculate the raw thrust required to balance gravity
u_hover = [1.380 * 9.81; 0; 0; 0]; % mass * g

%simulation loop
dt = 0.005;
t  = 0.0;
t_end = 25;
N = ceil(t_end/dt);

%black box preallocation
x_bb=zeros(n,N);
xp_bb=zeros(n,N);
xt_bb=zeros(n,N);
t_bb=zeros(1,N);
u_bb=zeros(m,N);
turb_bb=zeros(3,N);
wind_bb=zeros(3,N);

for i = 1 : N
  
  %black box record
  x_bb(:,i)=x;
  xp_bb(:,i)=xp;
  xt_bb(:,i)=xt;
  t_bb(:,i)=t;
  u_bb(:,i)=u;
  turb_bb(:,i)=turb;
  wind_bb(:,i)=wind;

  %saving to a file for later visualization
  save('black_box.mat', 'x_bb', 'xp_bb', 'xt_bb', 't_bb', 'u_bb', 'turb_bb', 'wind_bb');
  
  %turbulence
  turb=turbulence(turb);
  
  %system linearization
  if mod(i,10) == 0 || i == 1
    [A,B] = aa_matrices_AB( "aa_rhs" , x , t , u, turb, wind, n , m );
  end
      
  
  %LQR control
    
  %control weights
  R = 1*eye(m,m); %m is the dimension of u
  %state weights
  Q = eye(n,n);
    %v_c
  Q(1,1) = 10;      
  Q(2,2) = 10;      
  Q(3,3) = 5000;         
    %omega
  Q(4,4) = 5000;    
  Q(5,5) = 5000;  
  Q(6,6) = 5000;
    %x, y, z
  Q(7,7) = 1;      
  Q(8,8) = 1;      
  Q(9,9) = 1;         
    %phi, theta, psi
  Q(10,10) = 1000;     
  Q(11,11) = 1000;  
  Q(12,12) = 0; %numerical issue
    %rotational speed is not controlled
  Q(13,13) = 0;     
  Q(14,14) = 0;  
  Q(15,15) = 0; 
  Q(16,16) = 0; 
 
  %preplanned waypoint switch
    if abs(x(7)-xp(7))<=thresh && abs(x(8)-xp(8))<=thresh && ind<num_wpts
        ind=ind+1;
    end
  xp=plan(ind);

% Initialize alpha (smoothing factor between 0 and 1)
% Near 1 = fast tracking (no smoothing); Near 0 = highly smooth (slower tracking)
alpha = 0.3; 

% 1. Get the waypoint every 10 steps and apply alpha to the jump
if mod(i,10) == 0 || i == 1
    wp_new = waypoints(x, xp, radar_points);
    
    if i == 1
        wp_target = wp_new; % Snap instantly on first frame
        wp = wp_new;
    else
        wp_target = alpha * wp_new + (1 - alpha) * wp_target; % Alpha 1: Smooths raw data
    end
end

% 2. Frame-by-frame smoothing (Runs EVERY step to eliminate the staircase effect)
% Moves 'wp' smoothly toward 'wp_target' by a percentage of the distance
wp = wp + 0.1 * (wp_target - wp); % Alpha 2: Smooths the 10-step gap

% 3. Calculate trajectory
xt = trajectory(xp, wp, x, t, radar_points);
  
  %errors - LQR references
  e = zeros(n,1); 
    
    %v_c
  e(1) = x(1)-xt(1);
  e(2) = x(2)-xt(2);
  e(3) = x(3)-xt(3);
    %omega
  e(4) = x(4)-xt(4);
  e(5) = x(5)-xt(5);
  e(6) = x(6)-xt(6);
    %x,y,z
  e(7) = x(7)-xt(7);
  e(8) = x(8)-xt(8);
  e(9) = x(9)-xt(9);
    %phi,theta,psi
  e(10) = x(10)-xt(10);
  e(11) = x(11)-xt(11);
  %e(12) = x(12)-xt(12);
    %rotational speed is not controlled
  %e(13) = x(13);
  %e(14) = x(14);
  %e(15) = x(15);
  %e(16) = x(16);
  
   
  %computing Optimal Gain
     [K,P] = lqr_m( A , B , Q , R );
      
     u = -K * e;
    
  %control input saturation
  
    T_col_max = 4*0.8*9.81; %31.4 N
    
    u(1) = max( 0.2*T_col_max , min( T_col_max , u(1) ) ); 

    M_roll_max = u(1)*e(4)/50;
    M_pitch_max = u(1)*e(5)/50;
    M_yaw_max = u(1)/300; %max 0.1 Nm

    u(2) = max( -M_roll_max , min( M_roll_max , u(2) ) );
    u(3) = max( -M_pitch_max , min( M_pitch_max , u(3) ) ); 
    u(4) = max( -M_yaw_max , min( M_yaw_max , u(4) ) ); 
  
  %selection of the integration scheme  
  x = aa_rk45( "aa_rhs" , x , t , dt , u, turb, wind);
  %x = aa_euler( "aa_rhs" , x , t , dt , u, turb, wind);
    
  %Flight Data plots
   if mod(i,100) == 0 
    tiledlayout(5,4)
    %v_c
    nexttile
    plot(t_bb(1:i),x_bb(1,1:i))
    hold on
    plot(t_bb(1:i),xt_bb(1,1:i))
    hold off
    title('v_{xi}')
    ylim([-10 10])
    
    nexttile
    plot(t_bb(1:i),x_bb(2,1:i))
    hold on
    plot(t_bb(1:i),xt_bb(2,1:i))
    hold off
    title('v_{eta}')
    ylim([-10 10])
    
    nexttile
    plot(t_bb(1:i),x_bb(3,1:i))
    hold on
    plot(t_bb(1:i),xt_bb(3,1:i))
    hold off
    title('v_{zeta}')
    ylim([-10 10])
    
    nexttile
    plot(t_bb(1:i),x_bb(13,1:i))
    title('omega_1')
    ylim([0 RPM_max*pi/30])
    
    %omega
    nexttile
    plot(t_bb(1:i),rad2deg*x_bb(4,1:i))
    hold on
    plot(t_bb(1:i),rad2deg*xt_bb(4,1:i))
    hold off
    title('omega_{xi}')
    ylim([-90 90])
    
    nexttile
    plot(t_bb(1:i),rad2deg*x_bb(5,1:i))
    hold on
    plot(t_bb(1:i),rad2deg*xt_bb(5,1:i))
    hold off
    title('omega_{eta}')
    ylim([-90 90])
    
    nexttile
    plot(t_bb(1:i),rad2deg*x_bb(6,1:i))
    hold on
    plot(t_bb(1:i),rad2deg*xt_bb(6,1:i))
    hold off
    title('omega_{zeta}')
    ylim([-500 500])
    
    nexttile
    plot(t_bb(1:i),x_bb(14,1:i))
    title('omega_2')
    ylim([0 RPM_max*pi/30])
    
    %r_c
    nexttile
    plot(t_bb(1:i),x_bb(7,1:i))
    hold on
    plot(t_bb(1:i),xt_bb(7,1:i))
    plot(t_bb(1:i),xp_bb(7,1:i), 'k')
    hold off
    title('x')
    ylim([-10 10])
    
    nexttile
    plot(t_bb(1:i),x_bb(8,1:i))
    hold on
    plot(t_bb(1:i),xt_bb(8,1:i))
    plot(t_bb(1:i),xp_bb(8,1:i), 'k')
    hold off
    title('y')
    ylim([-10 10])
    
    nexttile
    plot(t_bb(1:i),x_bb(9,1:i))
    hold on
    plot(t_bb(1:i),xt_bb(9,1:i))
    plot(t_bb(1:i),xp_bb(9,1:i), 'k')
    hold off
    title('z')
    ylim([-10 10])
    
    nexttile
    plot(t_bb(1:i),x_bb(15,1:i))
    title('omega_3')
    ylim([0 RPM_max*pi/30])
    
    %euler angles
    nexttile
    plot(t_bb(1:i),rad2deg*x_bb(10,1:i))
    hold on
    plot(t_bb(1:i),rad2deg*xt_bb(10,1:i))
    hold off
    title('roll')
    ylim([-45 45])
    
    nexttile
    plot(t_bb(1:i),rad2deg*x_bb(11,1:i))
    hold on
    plot(t_bb(1:i),rad2deg*xt_bb(11,1:i))
    hold off
    title('pitch')
    ylim([-45 45])
    
    nexttile
    plot(t_bb(1:i),rad2deg*wrapToPi(x_bb(12,1:i)))
    hold on
    plot(t_bb(1:i),rad2deg*xt_bb(12,1:i))
    hold off
    title('yaw')
    ylim([-180 180])
    
    nexttile
    plot(t_bb(1:i),x_bb(16,1:i))
    title('omega_4')
    ylim([0 RPM_max*pi/30])
    

    nexttile
    plot(t_bb(1:i),turb_bb(1,1:i))
    title('turb_x')
    ylim([-3 3])
    
    nexttile
    plot(t_bb(1:i),turb_bb(2,1:i))
    title('turb_y')
    ylim([-3 3])
        
    nexttile
    plot(t_bb(1:i),turb_bb(3,1:i))
    title('turb_z')
    ylim([-3 3])
        
    nexttile
    plot(t_bb(1:i),turb_bb(1,1:i))
    title('turb_x')
    ylim([-3 3])
    
    refresh;
    
    %rotor speeds in the right column
    
    drawnow;
   end
        

%end of the simulation
  if( t >= t_end )
    break;
  end

  t = t + dt;
end
