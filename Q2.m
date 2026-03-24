%{  
ENG417 Project - Question 2
Determine the following:
a. The open-loop response of the aircraft to a 1% step in thrust.
b. The open-loop response of the aircraft to a 1° step in elevator position.
c. The open-loop response of the aircraft to a 1° step in aileron position.
d. The open-loop response of the aircraft to a 1° step in rudder position.

Include a description of the responses that you observe, and explain the differences between the two aircraft. 
Identify the short-period, phugoid, roll subsidence, spiral and Dutch roll modes. 
Without a controller, would you consider these responses to be acceptable?
%}

%% Define System Variables
% Aircraft Specifications
% Longtudinal Dynamics
X_u = -0.0016;
X_w = 0.0103;
Z_u = -0.1179;
Z_w = -0.3907;
M_wdot = 0.0023;
M_u = 0.0085;
M_w = 0.0049;
M_q = -0.500;
Z_delta_e = 23.5329;
M_delta_e = -12.7322;
X_delta_e = 3.651;
X_delta_T = 4.2711;
% Lateral Dynamics
Y_beta = -27.38;
L_beta = -28.08;
N_beta = 5.481;
Y_p = 0.389;
Y_r = 0.538;
L_p = -1.644;
L_r = 0.333;
N_p = -0.081;
N_r = -1.504;
Y_delta_r = 4.588;
L_delta_r = 9.591;
L_delta_a = 47.92;
N_delta_r = -1.908;
N_delta_a = -1.429;
% Trim point and aircraft specifications
h_e = 30000; % Equilibirum altitude, ft
alpha_e = 5; % Equilibirum angle of attach, deg
V_0 = 182; % Velocity, m/s
M_ref = 0.6; % Mach number
m = 9295; % Mass, kg
I_x = 12875; % Roll moment of inertia, kgm^2
I_y = 75674; % Pitch moment of inertia, kgm^2
I_z = 85554; % Yaw moment of inertia, kgm^2
I_xz = 0.460; % Inertia product, kgm^2
p = 0.460; % Air density, kg/m^3
S = 27.871; % Wing area, m^2
b = 9.144; % Wing span, m
cbarbar = 3.450; % Mean aerodynamic chord, m



% Calculate 
mprime = m/(0.5*p*V_0*S);
I_xprime = I_x/(0.5*p*V_0*S);
I_yprime = I_y/(0.5*p*V_0*S*cbarbar);
I_zprime = I_z/(0.5*p*V_0*S*b);
I_xzprime = I_xz/(0.5*p*V_0*S*b);
% Concise longitudinal aerodynamic stability derivatives
x_u = X_u/mprime + (cbarbar/V_0*0*Z_u)/(mprime*(mprime-cbarbar/V_0*0));
z_u = Z_u/(mprime-cbarbar/V_0*0);
m_u = M_u/I_yprime + (c_barbar/V_0*M_wdot*Z_u)/(I_yprime*(mprime-cbarbar/V_0*0));
x_w = X_w/mprime + (cbarbar_V_0*X_wdot*Z_w)/(mrpime*(mprime-cbarbar/V_0*0));
x_q = 

% Concise lateral aerodynamic stability derivatives

% Lineraised, decoupled state space model - xdot = Ax + Bu
% Longtudinal Direction 
xdot_long = [udot; wdot; qdot; thetadot];
x_long =[u; w; q; theta];
u_long = [delta_e; delta_T];
A_long = [x_u x_w x_q x_theta; z_u z_w z_q z_theta; m_u m_w m_q m_theta; 0 0 1 0];
B_long = [x_delta_e x_delta_T; z_delta_e z_delta_T; m_delta_e m_delta_T; 0 0];
% Lateral Direction
xdot_lat = [betadot; pdot; rdot; phidot];
x_lat =[beta; p; r; phi];
u_lat = [delta_a; delta_r];
A_lat = [y_beta y_p y_r y_phi; l_beta l_p l_r l_phi; n_beta n_p n_r n_phi; 0 1 0 0];
B_lat = [y_delta_a y_delta_r; l_delta_a l_delta_r; n_delta_a n_delta_r; 0 0];
%% a. 1% step in thrust

%% b. 1° step in elevator position

%% c. 1° step in aileron position

%% d. 1° step in rudder position

