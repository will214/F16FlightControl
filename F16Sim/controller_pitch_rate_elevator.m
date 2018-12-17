state_space = reduced_2_state_model();

% Requirements
V = 600; % ft/s
g = 9.81/0.3048;
omega_req = 0.03 * V * 0.3048; 
T_req = 1/(0.75 * omega_req);
damp_req = 0.5;

% required poles 
p1 = -omega_req * damp_req + omega_req * (damp_req^2 - 1)^0.5;
p2 = -omega_req * damp_req - omega_req * (damp_req^2 - 1)^0.5;

K = place(state_space.A, state_space.B, [p1, p2]);

controlled_system = ss(state_space.A - state_space.B * K, state_space.B, state_space.C, state_space.D);
H = tf(controlled_system);
H_alpha_de = H(1, 1);
H_q_de = H(2, 1);
% check damping and freq
eig_new = eig(controlled_system.A);
[omega, damp_coeff] = freq_and_damp(eig_new(1));

figure;
step(state_space);
% K_alpha and k_q are in degrees/rad.

% check for feasability
vertical_gust = 4.572/0.3048;
new_angle_attack = vertical_gust/V; % rad
elevetor_deflection = - K(1) * new_angle_attack; % acceptable 6 deg approx

% to change T_theta you want a lead lag filter
% (1 + T_new) / (1 + T_old) that multiplies 
% K (1 + T_new)/Den. The filter must stay outside of the control loop
% to avoid modifying the pole location r T_new and T_old would end
% up in the denominator
s = tf('s');
q_controller_num = H_q_de.Numerator{1};
T_theta = q_controller_num(2)/q_controller_num(3);
filter = (1 + T_req * s)/(1 + T_theta * s);
controller_w_filter = minreal(filter * H_q_de);
dc_gain = evalfr(controller_w_filter, 0);
final_controller = controller_w_filter / (-dc_gain);

% overshoot
figure;
[y, t, x] = step(final_controller);
plot(t, y);
overshoot = max(abs(y))/abs(y(end));

% q6 check CAP and Gibson
CAP = omega^2/(V/g * 1/T_req);
db_qs = T_req - 2 * damp_coeff/omega;

% criteria are satisfied;

