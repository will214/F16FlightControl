save_matrices = 0;
if (save_matrices)
    FindF16Dynamics;
    save('state_space/matrices_FL50_V300.mat', 'A_lo', 'B_lo', 'C_lo', 'D_lo')
    save('state_space/matrices_FL50_V300_trim.mat', 'trim_state_lo', 'trim_thrust_lo', 'trim_control_lo');
end

trim = load('state_space/matrices_FL50_V300_trim.mat');
trim_state_lin = [trim.trim_state_lo; trim.trim_thrust_lo; trim.trim_control_lo];
state_space = load('state_space/matrices_FL50_V300.mat');

A = state_space.A_lo;
B = state_space.B_lo;
C = state_space.C_lo;
D = state_space.D_lo;

% reduce state to x = (h, v, alpha, theta, q), u = (thrust, d_el)
A_red = A([3, 7, 8, 5, 11], [3, 7, 8, 5 11]);
B_red = A([3, 7, 8, 5, 11], [13, 14]);
C_red = eye(5);
D_red = D([3, 7, 8, 5, 11], [1, 2]);

aircraft_long = ss(A_red, B_red, C_red, D_red, ...
                    'StateName', {'h', 'V', 'alpha', 'theta', 'q'}, ... 
                     'InputName', {'thrust', 'd_e'}, ...
                     'OutputName', {'h', 'V', '\alpha', '\theta', 'q'});
                      
Q = diag([30, 1, 1, 1, 1]);   
R = [0.1 0; 0 2];
% states are radiands 
[K, S, E] = lqr(A_red, B_red, Q, R);
K_inner = K(:, 2:end);
K_h = K(:, 1);

sim('lqr_terrain_following.slx', [0, 60]);

aircraft_height = simout.Data(:, 1);
reference_signal = canyon.Data(:, 2);
canyon_height = canyon.Data(:, 2) - 40;
x = canyon.Data(:, 1);

plot(x, reference_signal); hold on;
plot(x, aircraft_height, 'linewidth', 2);
legend('Canyon height', 'Reference track', 'Aircraft height')
figure;
plot(x, (aircraft_height - reference_signal)*0.3048);




                 