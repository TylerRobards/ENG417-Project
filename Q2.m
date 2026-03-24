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
alpha_e = deg2rad(5); % Equilibirum angle of attach, rad
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
g=9.81;

%% Calculate Values for Linearised, decoupled state space model
mprime = m/(0.5*p*V_0*S);
I_xprime = I_x/(0.5*p*V_0*S);
I_yprime = I_y/(0.5*p*V_0*S*cbarbar);
I_zprime = I_z/(0.5*p*V_0*S*b);
I_xzprime = I_xz/(0.5*p*V_0*S*b);
theta_e = alpha_e;  % Due to steady level flight (equilibrium)
U_e = V_0*cos(theta_e);
W_e = V_0*sin(theta_e);
X_wdot = 0;
Z_wdot = 0;
X_q = 0;
Z_q = 0;
Z_delta_T = 0;
M_delta_T = 0;
Y_delta_a = 0;
den_lat = I_xprime*I_zprime - I_xzprime^2;
% Concise longitudinal aerodynamic stability derivatives
x_u = X_u/mprime + (cbarbar/V_0*X_wdot*Z_u)/(mprime*(mprime - cbarbar/V_0*Z_wdot));
z_u = Z_u/(mprime - cbarbar/V_0*Z_wdot);
m_u = M_u/I_yprime + (cbarbar/V_0*M_wdot*Z_u)/(I_yprime*(mprime - cbarbar/V_0*Z_wdot));
x_w = X_w/mprime + (cbarbar/V_0*X_wdot*Z_w)/(mprime*(mprime - cbarbar/V_0*Z_wdot));
z_w = Z_w/(mprime - cbarbar/V_0*Z_wdot);
m_w = M_w/I_yprime + (cbarbar/V_0*M_wdot*Z_w)/(I_yprime*(mprime - cbarbar/V_0*Z_wdot));
x_q = (cbarbar*X_q - mprime*W_e)/mprime + ((cbarbar*Z_q + mprime*U_e)*cbarbar/V_0*X_wdot)/(mprime*(mprime - cbarbar/V_0*Z_wdot));
z_q = (cbarbar*Z_q + mprime*U_e)/(mprime - cbarbar/V_0*Z_wdot);
m_q = cbarbar*M_q/I_yprime + ((cbarbar*Z_q + mprime*U_e)*cbarbar/V_0*M_wdot)/(I_yprime*(mprime - cbarbar/V_0*Z_wdot));
x_theta = -g*cos(theta_e) + (cbarbar/V_0*X_wdot*g*sin(theta_e))/(mprime - cbarbar/V_0*Z_wdot);
z_theta = -(mprime*g*sin(theta_e))/(mprime - cbarbar/V_0*Z_wdot);
m_theta = -(cbarbar/V_0*M_wdot*mprime*g*sin(theta_e))/ (I_yprime*(mprime - cbarbar/V_0*Z_wdot));
% Concise longitudinal control derivatives
x_delta_e = V_0*X_delta_e/mprime + (cbarbar/V_0*X_wdot*V_0*Z_delta_e) / (mprime*(mprime - cbarbar/V_0*Z_wdot));
z_delta_e = V_0*Z_delta_e / (mprime - cbarbar/V_0*Z_wdot);
m_delta_e = V_0*M_delta_e/I_yprime + (cbarbar*M_wdot*Z_delta_e) / (I_yprime*(mprime - cbarbar/V_0*Z_wdot));
x_delta_T = V_0*X_delta_T/mprime + (cbarbar/V_0*X_wdot*V_0*Z_delta_T) / (mprime*(mprime - cbarbar/V_0*Z_wdot));
z_delta_T = V_0*Z_delta_T / (mprime - cbarbar/V_0*Z_wdot);
m_delta_T = V_0*M_delta_T/I_yprime + (cbarbar*M_wdot*Z_delta_T) / (I_yprime*(mprime - cbarbar/V_0*Z_wdot));
% Concise lateral aerodynamic stability derivatives
y_beta = Y_beta/mprime;
y_p = (b*Y_p + mprime*W_e)/mprime;
y_r = (b*Y_r - mprime*U_e)/mprime;
y_phi = g*cos(theta_e);
y_psi = g*sin(theta_e);
l_beta = (I_zprime*L_beta + I_xzprime*N_beta)/den_lat;
l_p = (I_zprime*L_p + I_xzprime*N_p)/den_lat;
l_r = (I_zprime*L_r + I_xzprime*N_r)/den_lat;
l_phi = 0;
l_psi = 0;
n_beta = (I_xprime*N_beta + I_xzprime*L_beta)/den_lat;
n_p = (I_xprime*N_p + I_xzprime*L_p)/den_lat;
n_r = (I_xprime*N_r + I_xzprime*L_r)/den_lat;
n_phi = 0;
n_psi = 0;
% Concise lateral control derivatives
y_delta_a = V_0*Y_delta_a/mprime;
l_delta_a = V_0*(I_zprime*L_delta_a + I_xzprime*N_delta_a)/den_lat;
n_delta_a = V_0*(I_xprime*N_delta_a + I_xzprime*L_delta_a)/den_lat;
y_delta_r = V_0*Y_delta_r/mprime;
l_delta_r = V_0*(I_zprime*L_delta_r + I_xzprime*N_delta_r)/den_lat;
n_delta_r = V_0*(I_xprime*N_delta_r + I_xzprime*L_delta_r)/den_lat;
%% Lineraised, decoupled state space model - xdot = Ax + Bu
% Longtudinal Direction 
% xdot_long = [udot; wdot; qdot; thetadot];
% x_long =[u; w; q; theta];
% u_long = [delta_e; delta_T];
A_long = [x_u x_w x_q x_theta; z_u z_w z_q z_theta; m_u m_w m_q m_theta; 0 0 1 0];
B_long = [x_delta_e x_delta_T; z_delta_e z_delta_T; m_delta_e m_delta_T; 0 0];
C_long = eye(4);
D_long = zeros(4,2);
sys_long = ss(A_long, B_long, C_long, D_long);
% Lateral Direction
% xdot_lat = [betadot; pdot; rdot; phidot];
% x_lat =[beta; p; r; phi];
% u_lat = [delta_a; delta_r];
A_lat = [y_beta y_p y_r y_phi; l_beta l_p l_r l_phi; n_beta n_p n_r n_phi; 0 1 0 0];
B_lat = [y_delta_a y_delta_r; l_delta_a l_delta_r; n_delta_a n_delta_r; 0 0];
C_lat = eye(4);
D_lat = zeros(4,2);
sys_lat = ss(A_lat, B_lat, C_lat, D_lat);
%% Model Verification
lat_poles = pole(sys_long)
long_poles = pole(sys_lat)
% pzplot(sys_long)
pzplot(sys_lat)

t = 0:0.01:10; % Timescale for sims
%% a. 1% step in thrust
u_long_dT = zeros(length(t),2);
u_long_dT(:,1) = 0;
u_long_dT(:,2) = 0.01;   % 1% thrust step

[y_dT, t_dT, x_dT] = lsim(sys_long, u_long_dT, t);

figure;
plot(t_dT, x_dT(:,1), t_dT, x_dT(:,2), t_dT, x_dT(:,3), t_dT, x_dT(:,4));
grid on;
xlabel('Time (s)');
ylabel('State response');
title('Open-loop Longitudinal Response to 1% Thrust Step');
legend('u','w','q','\theta','Location','best');
%% b. 1° step in elevator position
u_long_de = zeros(length(t),2);
u_long_de(:,1) = deg2rad(1);   % delta_e = 1 deg
u_long_de(:,2) = 0;            % delta_T = 0

[y_de, t_de, x_de] = lsim(sys_long, u_long_de, t);

figure;
plot(t_de, x_de(:,1), t_de, x_de(:,2), t_de, x_de(:,3), t_de, x_de(:,4));
grid on;
xlabel('Time (s)');
ylabel('State response');
title('Open-loop Longitudinal Response to 1 Degree Elevator Step');
legend('u','w','q','\theta','Location','best');
%% c. 1° step in aileron position
u_lat_da = zeros(length(t),2);
u_lat_da(:,1) = deg2rad(1);   % delta_a = 1 deg
u_lat_da(:,2) = 0;            % delta_r = 0

[y_da, t_da, x_da] = lsim(sys_lat, u_lat_da, t);

figure;
plot(t_da, x_da(:,1), t_da, x_da(:,2), t_da, x_da(:,3), t_da, x_da(:,4));
grid on;
xlabel('Time (s)');
ylabel('State response');
title('Open-loop Lateral Response to 1 Degree Aileron Step');
legend('\beta','p','r','\phi','Location','best');
%% d. 1° step in rudder position
u_lat_dr = zeros(length(t),2);
u_lat_dr(:,1) = 0;
u_lat_dr(:,2) = deg2rad(1);   % delta_r = 1 deg

[y_dr, t_dr, x_dr] = lsim(sys_lat, u_lat_dr, t);

figure;
plot(t_dr, x_dr(:,1), t_dr, x_dr(:,2), t_dr, x_dr(:,3), t_dr, x_dr(:,4));
grid on;
xlabel('Time (s)');
ylabel('State response');
title('Open-loop Lateral Response to 1 Degree Rudder Step');
legend('\beta','p','r','\phi','Location','best');
