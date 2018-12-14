function [aircraft_long, aircraft_lat] = open_loop_response(plot, save_matrices);

if (save_matrices)
    FindF16Dynamics;
    save('state_space/matrices.mat', 'A_lo', 'B_lo', 'C_lo', 'D_lo')
end
% inputs are in radiants while outputs are in degrees
state_space = load('state_space/matrices.mat');
A = state_space.A_lo;
B = state_space.B_lo;
C = state_space.C_lo;
D = state_space.D_lo;

% reduce state to x = (v, alpha, theta, q), u = d_el
A_red_long = A([7, 8, 5, 11], [7, 8, 5 11]);
B_red_long = A([7, 8, 5, 11], [14]);
C_red_long = C([7, 8, 5, 11], [7, 8, 5 11]);
D_red_long = 0;

aircraft_long = ss(A_red_long, B_red_long, C_red_long, D_red_long, ...
                    'StateName', {'V', 'alpha', 'theta', 'q'}, ... 
                     'InputName', {'d_e'}, ...
                     'OutputName', {'V', '\alpha', '\theta', 'q'});
                 
time_phugoid = linspace(0, 200, 1000);
time_short_period = linspace(0, 15, 1000);
u_phugoid = ones(size(time_phugoid));
u_short_period = ones(size(time_short_period));
if (plot)
figure;
lsim(aircraft_long, u_phugoid, time_phugoid);
figure;
lsim(aircraft_long, u_short_period, time_short_period);
end

% reduce state to x = (beta, phi, p, r)
A_red_lat = A([9, 4, 10, 12], [9, 4, 10, 12]);
B_red_lat = A([9, 4, 10, 12], [15, 16]);
C_red_lat = C([9, 4, 10, 12], [9, 4, 10, 12]);
D_red_lat = [0, 0; 0, 0; 0, 0; 0, 0];

aircraft_lat = ss(A_red_lat, B_red_lat, C_red_lat, D_red_lat, ...,
                'StateName', {'beta', 'phi', 'p', 'r'}, ...
                'InputName', {'d_a', 'd_r'}, ...
                'OutputName', {'beta', 'phi', 'p', 'r'});
time = linspace(0, 20, 1000); 
if (plot)
figure;
impulse(aircraft_lat, time);
end