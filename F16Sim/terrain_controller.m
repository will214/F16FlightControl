%% Chapter 8: Design of a terrain following system


clear;
clc;
altitude = 5000;    % ft
velocity = 300;      % ft/s
gravity = 9.80665;   % ft/s^2
FindF16Dynamics;

trimmed_altitude = altitude;    % ft
distance_from_ground = 40;   % m
m_to_ft = 3.28084;   
initial_altitude = 1500; % m
initial_elevator = -4.1891;   % deg
initial_thrust = 2826.8165;    %  lb


A_TC = mat_lo([3 7 8 5 11], [3 7 8 5 11]);
B_TC = mat_lo([3 7 8 5 11], [13 14]);
C_TC = mat_lo([21 25 26 23 29], [3 7 8 5 11]);
D_TC = mat_lo([21 25 26 23 29], [13 14]);

SP_sys = ss(A_TC, B_TC, C_TC, D_TC);

% Q = [1000 0 0 0 0;  % altitude
%     0 0.000001 0 0 0;   % velocity
%     0 0 1000 0 0;   % alpha
%     0 0 0 1000 0;   % theta
%     0 0 0 0 100];  % q
% R = [1 0;        % dt
%     0 1];        % de

Q = [42 0 0 0 0;  % altitude
    0 1 0 0 0;   % velocity
    0 0 1 0 0;   % alpha
    0 0 0 1 0;   % theta
    0 0 0 0 1];  % q
R = [1 0;        % dt
    0 1];        % de

[K, S, e] = lqr(A_TC, B_TC, Q, R);
K(:,2:end) = deg2rad(K(:,2:end));

H_engine_num = [1];
H_engine_den = [1 1];
engine_limits = [1000-initial_thrust 19000-initial_thrust];

H_elevator_num = [20.2];
H_elevator_den = [1 20.2];
elevator_limits = [-25-initial_elevator 25-initial_elevator];

sim('terrain_follower.slx')
maximum_error = max(error_signal);
disp(maximum_error)
