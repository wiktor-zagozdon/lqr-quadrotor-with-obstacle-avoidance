function [ dx_dt ] = aa_rhs( x , t , u, turb, wind)
 
  n = size(x,1);
  dx_dt = zeros( n , 1 );

  deg2rad = pi/180.0;
  
  %drone parameters basing on DJI Phantom 4
  mass = 1.388;
  g = 9.81;
    %reference surface areas for the drag
  S_xi = 0.02; %up to 0.03
  S_eta = 0.02;
  S_zeta = 0.035; %up to 0.05

  C_D_xi = 0.8; %up to 1.2
  C_D_eta = 0.8; %up to 1.2
  C_D_zeta = 0.3;

  rho_0 = 1.225;
  rho = rho_0 * ( 1.0 - abs(x(9))/44300.0 )^4.256;
    %inertia matrix of the drone
  I_xi_xi=0.0178;
  I_xi_eta=0;
  I_xi_zeta=0;
  I_eta_xi=0;
  I_eta_eta=0.0178;
  I_eta_zeta=0;
  I_zeta_xi=0;
  I_zeta_eta=0;
  I_zeta_zeta=0.022;

  I_c=[I_xi_xi I_xi_eta I_xi_zeta; I_eta_xi I_eta_eta I_eta_zeta;I_zeta_xi I_zeta_eta I_zeta_zeta];
    %pitch and roll arm of force
  a=0.175/sqrt(2);
  b=a;

  %rotor parameters based on the Blade Element Theory
    %rotor thrust
  c_T=0.0137; %requires more research
  R=0.12; %rotor blade length
  
  K_t=c_T*rho*pi*R*R*R*R;
  %K_t=1.08e-5;
    
    %rotor torque
  c_Q=0.0018; %to be specified later
  K_Q=c_Q*rho*pi*R*R*R*R*R;
  %K_Q=1.762e-7;
  
  %inertia matrix of the totor
    %This value includes the 10.8x5.4 inch propellers and the 2312S motor bell, which are designed for high efficiency and quick response (8500 rpm max).
  I_blade=5.28e-5;
  I_bell=2.5e-6;
  I_xi_xi_i=0.5*(I_blade+I_bell); 
  I_xi_eta_i=0;
  I_xi_zeta_i=0;
  I_eta_xi_i=0;
  I_eta_eta_i=0.5*(I_blade+I_bell); 
  I_eta_zeta_i=0;
  I_zeta_xi_i=0;
  I_zeta_eta_i=0;
  I_zeta_zeta_i= I_blade+I_bell; %only this component is important

  I_i=[I_xi_xi_i I_xi_eta_i I_xi_zeta_i; I_eta_xi_i I_eta_eta_i I_eta_zeta_i;I_zeta_xi_i I_zeta_eta_i I_zeta_zeta_i];

    %rotor delayed response
  time_constant=0.07; %average value of the control loop delay without downlink delay
  
  
  %state vector components
  v_c = [x(1) ; x(2); x(3)];
  omega = [x(4) ; x(5); x(6)];
  r_c = [x(7) ; x(8); x(9)];
  Theta = [x(10) ; x(11); x(12)];
  omega_i = [x(13) ; x(14); x(15); x(16)]; %actual angular speed of the rotors
  
  %input vector
  T_col=u(1);
  M_roll=u(2);
  M_pitch=u(3);
  M_yaw=u(4);

    
  %Mapping Matrix M
  
  M=[K_t K_t K_t K_t; a*K_t -a*K_t -a*K_t a*K_t; b*K_t b*K_t -b*K_t -b*K_t; -K_Q K_Q -K_Q K_Q];

  %relation between the virtual input and desired angular speeds
  omega_ui_sqr = M\u; %desired squared angular speed of the rotors
  
  omega_ui=[sqrt(omega_ui_sqr(1)); sqrt(omega_ui_sqr(2)); sqrt(omega_ui_sqr(3)); sqrt(omega_ui_sqr(4))];

 
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

  %U matrix
  U=[1 sin(phi)*tan(theta) cos(phi)*tan(theta); 0 cos(phi) -sin(phi); 0 sin(phi)/cos(theta) cos(phi)/cos(theta)];

  %external forces
  T_FC_BC=T_BC_FC'; %the inverse is a transpose here
  Q = T_FC_BC*[0;0;mass*g];
      %relative air velocity in the drone reference frame is a trasformed sum
      %of wind turbulence and drone speed with respect to the ground (FC).

  v_air=v_c+T_FC_BC*(turb+wind);
    %rotor thrust
  
  T_1=K_t*omega_i(1)*omega_i(1);
  T_2=K_t*omega_i(2)*omega_i(2);
  T_3=K_t*omega_i(3)*omega_i(3);
  T_4=K_t*omega_i(4)*omega_i(4);

    %rotor torque
  
  tau_1=K_Q*omega_i(1)*omega_i(1);
  tau_2=K_Q*omega_i(2)*omega_i(2);
  tau_3=K_Q*omega_i(3)*omega_i(3);
  tau_4=K_Q*omega_i(4)*omega_i(4);
  
  F_E=[0;0;(-T_1 -T_2 -T_3 -T_4)];
  D = [-0.5*rho*v_air(1)*abs(v_air(1))*C_D_xi*S_xi;
     -0.5*rho*v_air(2)*abs(v_air(2))*C_D_eta*S_eta;
     -0.5*rho*v_air(3)*abs(v_air(3))*C_D_zeta*S_zeta]; %added turbulence
  F_A=D; %lift force is neglected
  
  F = Q + F_E + F_A;

  %external moments about the center of mass of the drone
  M_c_zeta=[0;0;-tau_1+tau_2-tau_3+tau_4];
  M_c_xi=a*[T_1+T_4-T_2-T_3;0;0];
  M_c_eta=b*[0;T_1+T_2-T_3-T_4;0];
    %aerodynamic moments are neglected
    
    
    
  
  
    
  M_c=M_c_zeta+M_c_eta+M_c_xi; 
 
  %ODE equations of motion
  dv_c_dt = (-cross(omega,mass*v_c)+F)/mass;
  domega_i_dt=(omega_ui-omega_i)/time_constant;
    %gyroscopic moments acting on the rotors
  domega_dt = (I_c+4*I_i)\(-cross(omega, I_c*omega) + M_c -I_i*([0;0;domega_i_dt(1)]-[0;0;domega_i_dt(2)]+[0;0;domega_i_dt(3)]-[0;0;domega_i_dt(4)])-cross(omega, I_i*([0;0;omega_i(1)]-[0;0;omega_i(2)]+[0;0;omega_i(3)]-[0;0;omega_i(4)])));
  dr_c_dt = T_BC_FC*v_c; %which transformation should be here
  dTheta_dt = U*omega;
  
  
  dx_dt = [dv_c_dt; domega_dt; dr_c_dt; dTheta_dt; domega_i_dt];
end
