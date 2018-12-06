function [short_period_model] = reduced_2_state_model()
% cap = 9 * w_sp * T_theta/V
[aircraft_long, aircraft_lat] = open_loop_response(0, 0);
A_red = aircraft_long.A([2, 4], [2, 4]);
B_red = aircraft_long.B([2, 4]);
C_red = aircraft_long.C([2, 4], [2, 4]);
D_red = 0;

short_period_model = ss(A_red, B_red, C_red, D_red);
time = linspace(0, 5, 1000);
u = ones(size(time));
[y, t, x] = lsim(aircraft_long, u, time);
% hold on;
[y_s, t, x] = lsim(short_period_model, u, time);
% q = y(:, 4); q_s = y_s(:, 2);
% plot(time, q, 'b');
% hold on;
% plot(time, q_s, 'r');
end