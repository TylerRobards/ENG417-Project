%{  
ENG417 Project - Question 7
Implement a model predictive control system to execute an altitude-adjustment-and-
hold command, taking the plane from its starting altitude of 30 000 ft and ending at an
altitude of 32 000 ft. Note that in practice, the trim state changes here, but the linearised
dynamics still provide a reasonable model of the system. As part of your response, show
that the transfer function between the elevator angle and altitude is given approximately
by thye function in the problem description.

%}
clear; clc; close all;
%% Longtiudiunal Model 
% xdot = A*x+B*u
% y = C*x+D*u

% xdot = [udot; wdot; qdot; thetadot];
% x =[u; w; q; theta];
% u = [delta_e; delta_T];

% A Matrix from project 1 feedback
A = [-0.0002, 0.0013, -15.8623, -9.7727;
    -0.0148, -0.490, 181.3074, -0.8550;
    0.0005, 0.0003, -0.0913, 0.0004;
    0, 0, 1, 0];
% B Matrix from project 1 feedback
B = [83.4038, 97.5694;
    537.876, 0;
    -123.252, 0;
    0, 0];
% Take only elevator control
Be = B(:,1);
C = eye(4);


%% System variables and Constants (need to update)

Ts = 0.05;   % To update as needed
k = 10;     % To update as needed
Nc = 5;     % To update as needed
Np = 80;    % To update as needed
R_weight = 10;       % larger value gives smoother elevator action

climb_time = 20;   % seconds to ramp from 30000 ft to 32000 ft

x0 = [0;0;0];
xm0 = [0;0];

V0 = 182;                  % trim velocity, m/s
ft_to_m = 0.3048;

h0_ft = 30000;             % starting altitude
hf_ft = 32000;             % final altitude

r = (hf_ft - h0_ft) * ft_to_m;    % altitude reference in metres

%% Discrete State Space Model 
ATs = zeros(4,4);
BTs = zeros(4,2);
BTse = zeros(4,2);
for i = 1:k+1
    ATs = ATs + (Ts)^(i-1)*A^(i-1)./factorial(i-1);
    BTs = BTs + (A^(i-1)*Ts^(i)*B/factorial(i));
end

Am = ATs;
Bm = BTs(:,1)  % Take only the elevator control
Cm = C;

% xm[k+1] = Am*xm[k] + Bm*um[k[
% y[k] = Cm*xm[k]

%% Add altitude and altimeter lag states
% x_m = [u_a; w; q; theta; h; h_m]
%
% h[k+1]   = h[k] + Ts*(V0*theta[k] - w[k])
% h_m_dot  = a_alt*(h - h_m)
% h_m[k+1] = h_m[k] + Ts*a_alt*(h[k] - h_m[k])

tau_alt = 0.8;          % altimeter time constant in seconds
a_alt = 1/tau_alt;
Am = [Am, zeros(4,2);
       0, -Ts, 0, Ts*V0, 1, 0;
       0,  0,  0, 0, Ts*a_alt, 1 - Ts*a_alt]
Bm = [Bm;
       0;
       0];
% Output is measured altitude, not true altitude
Cm = [0 0 0 0 0 1];

%% MPC augmentation 
% x[k] = [Delta x_m[k]; y[k]]
%
% x[k+1] = A*x[k] + B*Delta u[k]
% y[k]   = C*x[k]

n = size(Am,1);      % number of plant states, n = 5
ny = size(Cm,1);     % number of outputs, ny = 1

A = [Am, zeros(n,ny);
     Cm*Am, eye(ny)];
B = [Bm;
     Cm*Bm];
C = [zeros(ny,n), eye(ny)];

%% Build prediction matrices
% Y = F*x[k] + Phi*DeltaU


F = zeros(Np*ny, n + ny);
Phi = zeros(Np*ny, Nc);

for i = 1:Np
    % Free response term: C*A^i
    F((i-1)*ny+1:i*ny, :) = C * (A^i);
    % Forced response terms
    for j = 1:Nc
        if i >= j
            Phi((i-1)*ny+1:i*ny, j) = C * (A^(i-j)) * B;
        end
    end
end

%% Cost function weighting
R = R_weight * eye(Nc);

% Reference trajectory
% R_s = r * ones(Np*ny,1); % Now updated in the loop

%% Input constraints
u_max = deg2rad(30);
u_min = -u_max;

du_max = deg2rad(65) * Ts;
du_min = -du_max;

% Rate constraints on DeltaU
% du_min <= DeltaU <= du_max

I_Nc = eye(Nc);

M_1 = [ I_Nc;
       -I_Nc];

gamma_1 = [ du_max * ones(Nc,1);
           -du_min * ones(Nc,1)];

% Matrix that accumulates DeltaU into future U values
H = tril(ones(Nc));

%% Simulation setup
Tfinal = 120;
Nsim = round(Tfinal/Ts);

% Plant state x_m = [u_a, w, q, theta, h]'
x_m = zeros(n,1);
x_m_prev = x_m;

% Control input u[k] = delta_e[k]
u = 0;

% Storage vectors
time = zeros(Nsim,1);
h_true_store_ft = zeros(Nsim,1);
h_meas_store_ft = zeros(Nsim,1);
u_store_deg = zeros(Nsim,1);
theta_store_deg = zeros(Nsim,1);
w_store = zeros(Nsim,1);
q_store = zeros(Nsim,1);
elevator_rate_store = zeros(Nsim,1);
u_prev = u;

% quadprog options
options = optimoptions('quadprog', 'Display', 'off');

%% Simulation loop
for k = 1:Nsim

      % Current simulation time
    t_now = (k-1)*Ts;

    % Output y[k] = h[k]
    y = Cm*x_m;

    % Delta x_m[k]
    delta_x_m = x_m - x_m_prev;

    % MPC state x[k]
    x = [delta_x_m;
         y];

    % Ramped altitude reference
    r_now = min(r, r*t_now/climb_time);
    R_s = r_now * ones(Np,1);

    % QP cost matrices
    E = 2*(Phi'*Phi + R);
    f_qp = -2*Phi'*(R_s - F*x);

    % Magnitude constraints on future u values
    % u_min <= u[k-1] + H*DeltaU <= u_max

    M_2 = [ H;
           -H];

    gamma_2 = [ u_max*ones(Nc,1) - u*ones(Nc,1);
               -u_min*ones(Nc,1) + u*ones(Nc,1)];

    % Combine constraints
    % M*DeltaU <= gamma

    M = [M_1;
         M_2];

    gamma = [gamma_1;
             gamma_2];

    % Solve constrained MPC optimisation
    [DeltaU, ~, exitflag] = quadprog(E, f_qp, M, gamma, [], [], [], [], [], options);

    % If quadprog fails, apply no change in elevator
    if exitflag <= 0
        DeltaU = zeros(Nc,1);
    end

    % Apply only the first control increment
    delta_u = DeltaU(1);

    % u[k] = u[k-1] + Delta u[k]
    u = u + delta_u;

    % Safety clamp against numerical issues
    u = max(min(u, u_max), u_min);

    % Propagate the plant
    x_m_prev = x_m;

    % x_m[k+1] = A_m*x_m[k] + B_m*u[k]
    x_m = Am*x_m + Bm*u;

    % Store results
    time(k) = (k-1)*Ts;
    h_true_store_ft(k) = h0_ft + x_m(5)/ft_to_m;
    h_meas_store_ft(k) = h0_ft + x_m(6)/ft_to_m;
    u_store_deg(k) = rad2deg(u);
    theta_store_deg(k) = rad2deg(x_m(4));
    w_store(k) = x_m(2);
    q_store(k) = x_m(3);
    elevator_rate_store(k) = rad2deg((u - u_prev) / Ts);
    u_prev = u;

end

%% Plot Results
%% Plot altitude response
figure;
plot(time, h_true_store_ft, 'LineWidth', 1.5);
hold on;
plot(time, h_meas_store_ft, '--', 'LineWidth', 1.5);
yline(hf_ft, ':', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Altitude (ft)');
title('Altitude Response with Altimeter Lag');
legend('True altitude', 'Measured altitude', 'Reference', 'Location', 'best');

%% Plot elevator input
figure;
plot(time, u_store_deg, 'LineWidth', 1.5);
hold on;
yline(30, '--');
yline(-30, '--');
grid on;
xlabel('Time (s)');
ylabel('Elevator angle, \delta_e (deg)');
title('Elevator Input');
legend('Elevator', 'Upper limit', 'Lower limit', 'Location', 'best');

%% Plot elevator rate
figure;
plot(time, elevator_rate_store, 'LineWidth', 1.5);
hold on;
yline(65, '--', 'LineWidth', 1.2);
yline(-65, '--', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Elevator rate, \dot{\delta}_e (deg/s)');
title('Elevator Rate of Change');
legend('Elevator rate', 'Upper rate limit', 'Lower rate limit', 'Location', 'best');

%% Plot pitch angle
figure;
plot(time, theta_store_deg, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Pitch angle, \theta (deg)');
title('Pitch Angle Response');

%% Plot vertical velocity perturbation
figure;
plot(time, w_store, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('w (m/s)');
title('Vertical Velocity Perturbation');

%% Print final values
fprintf('Final true altitude: %.2f ft\n', h_true_store_ft(end));
fprintf('Final measured altitude: %.2f ft\n',h_meas_store_ft(end));
fprintf('Final elevator angle: %.2f deg\n', u_store_deg(end));
fprintf('Final pitch angle: %.2f deg\n', theta_store_deg(end));
fprintf('Final vertical velocity perturbation: %.4f m/s\n', w_store(end));