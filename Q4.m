%{  
ENG417 Project - Question 4
Use the major and minor loop design techniques to design a pitch attitude hold control system.
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
%% Design Minor Loop
tau_elevator = 0.02; % Elevator actuator lag time constant
g_a = tf(1,[tau_elevator 1]); % Actuator lag block 
B_E = B_long(:,1);
G_q = tf(ss(A_long, B_E, [0 0 1 0], 0));

% Inner-loop open-loop transfer function
L = minreal(g_a * G_q);
% Get numerator and denominator vectors
[num, den] = tfdata(L, 'v');
% Choose target damping ratio for the short-period poles
zeta_des = 0.7;   
% Function: imaginary part of K2 should be zero for a valid real gain
imagK = @(wn) imag( polyval(den, ...
    (-zeta_des*wn + 1j*wn*sqrt(1-zeta_des^2))) / ...
    polyval(num, ...
    (-zeta_des*wn + 1j*wn*sqrt(1-zeta_des^2))) );

% Solve for wn
wn_sol = fzero(imagK, [1 200]);

% Desired pole
sd = -zeta_des*wn_sol + 1j*wn_sol*sqrt(1-zeta_des^2);

% Compute K2
K2 = polyval(den, sd) / polyval(num, sd);
K2 = -1*real(K2);

fprintf('Chosen damping ratio zeta = %.4f\n', zeta_des);
fprintf('Solved wn = %.4f rad/s\n', wn_sol);
fprintf('Desired pole sd = %.4f %+.4fj\n', real(sd), imag(sd));
fprintf('Computed K2 = %.6f\n', K2);

% Closed-loop inner loop
% Use +1 because your stabilising convention was positive feedback
% with positive K2, equivalent to negative k in feedback(k*L,1)
Tq = feedback(K2 * L, 1);

disp('Closed-loop poles:')
pole(Tq)

disp('Closed-loop damping data:')
damp(Tq)

figure;
step(Tq, 20)
grid on
title(sprintf('Minor Loop response with K2 = %.4f', K2))

info = stepinfo(Tq);
disp(info)

%% Design Major Loop
Ctheta = [0 0 0 1];

% Transfer functions
G_theta = tf(ss(A_long, B_E, Ctheta, 0));  % delta_e -> theta

% Outer-loop plant with inner loop closed
G_theta_in = minreal((g_a*G_theta) / (1 + K2*g_a*G_q));

% Numerator and denominator of outer-loop plant
[num_th, den_th] = tfdata(G_theta_in,'v');

% Desired damping ratio for outer loop
zeta_des = 0.6;

% Function whose zero gives a real K1
imagK1 = @(wn) imag( -polyval(den_th, ...
    (-zeta_des*wn + 1j*wn*sqrt(1-zeta_des^2))) / ...
    polyval(num_th, ...
    (-zeta_des*wn + 1j*wn*sqrt(1-zeta_des^2))) );

% Scan over wn to find sign changes
wn_test = linspace(0.01,50,50000);
vals = arrayfun(imagK1, wn_test);

figure;
plot(wn_test, vals, 'LineWidth', 1.2);
grid on;
xlabel('\omega_n (rad/s)');
ylabel('imag(K1)');
title('Imaginary part of K1 versus \omega_n');
yline(0,'k--');

idx = find(vals(1:end-1).*vals(2:end) < 0);

if isempty(idx)
    error('No sign changes found in the scanned interval. Increase scan range or change zeta_des.');
else
    fprintf('Candidate intervals for fzero:\n');
    for i = idx
        fprintf('[%.6f, %.6f]\n', wn_test(i), wn_test(i+1));
    end
end

% Use first sign-change interval
a = wn_test(idx(2));
b = wn_test(idx(2)+1);

fprintf('\nUsing interval [%.6f, %.6f] for fzero.\n', a, b);

wn_sol = fzero(imagK1, [a b]);

% Desired dominant pole
sd = -zeta_des*wn_sol + 1j*wn_sol*sqrt(1-zeta_des^2);

% Compute K1
K1 = -polyval(den_th, sd) / polyval(num_th, sd);
K1 = real(K1);

fprintf('\nChosen damping ratio zeta = %.4f\n', zeta_des);
fprintf('Solved wn = %.4f rad/s\n', wn_sol);
fprintf('Desired pole sd = %.4f %+.4fj\n', real(sd), imag(sd));
fprintf('Computed K1 = %.6f\n', K1);

% Closed-loop outer loop
T_theta = feedback(K1*G_theta_in, 1);

disp('Closed-loop poles:');
disp(pole(T_theta));

disp('Closed-loop damping data:');
damp(T_theta);

figure;
step(T_theta, 20);
grid on;
title(sprintf('Closed Loop pitch-attitude response with K1 = %.4f', K1));

info = stepinfo(T_theta);
disp('Step info:');
disp(info);

% Optional: root locus of the outer loop
figure;
rlocus(G_theta_in);
grid on;
title('Outer-loop root locus');

%% Step tracking test
theta_ref_deg = 5;
theta_ref_rad = deg2rad(theta_ref_deg);

t = 0:0.001:5;
[y,t] = step(theta_ref_rad*T_theta, t);

info = stepinfo(y, t, theta_ref_rad);

fprintf('Rise time: %.4f s\n', info.RiseTime);
fprintf('Peak time: %.4f s\n', info.PeakTime);
fprintf('Overshoot: %.2f %%\n', info.Overshoot);
fprintf('Settling time: %.4f s\n', info.SettlingTime);

steady_state_error = abs(theta_ref_rad - y(end));
fprintf('Steady-state error: %.6f rad (%.4f deg)\n', ...
    steady_state_error, rad2deg(steady_state_error));

t = 0:0.001:10;
u = deg2rad(2)*sin(2*pi*0.5*t);

y = lsim(T_theta, u, t);

figure;
plot(t, rad2deg(y), 'LineWidth', 1.5)
hold on
plot(t, rad2deg(u), '--', 'LineWidth', 1.2)
grid on
xlabel('Time (s)')
ylabel('\theta (deg)')
title('Sinusoidal reference tracking')
legend('\theta(t)', '\theta_{ref}(t)')