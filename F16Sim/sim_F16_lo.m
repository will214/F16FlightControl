clear all;
altitude = 15000;
velocity = 500;
% weird behaviour otherwise
global fi_flag_Simulink
fi_flag_Simulink = 0;
% trim settings
thrust = 5000;          % thrust, lbs
elevator = -0.09;       % elevator, degrees
alpha = 8.49;              % AOA, degrees
rudder = -0.01;             % rudder angle, degrees
aileron = 0.01;            % aileron, degrees

[trim_state, trim_thrust, trim_control, dLEF, UX] = trim_steady_state(thrust, elevator, alpha, aileron, rudder, velocity, altitude);
load_system('Lin_F16Block_lo');

% params for linear model

deltaT = 0.001;
TStart = 0; 
TFinal = 5;
time = linspace(0, TFinal, TFinal/deltaT + 1);
thrust = 0;  % Since this a linear model
accelerometer_pos = [0, 5, 5.9, 6, 7, 15] * 0.3048;
% accelerometer_pos = [5];
tf_list = {};
acc_data = {};

for x_a = accelerometer_pos

    [A, B, C, D] = linmod('LIN_F16Block_lo', [trim_state; trim_thrust; trim_control; dLEF; -trim_state(8)*180/pi], ... 
                                              [trim_thrust; trim_control]);
    % initial states are already set in the model
    system = ss(A, B, C, D);
    H_mimo = tf(system);
    H_an_el = minreal(H_mimo(19, 2));
    tf_list(end + 1) = {H_an_el};
    sim('SS_F16_Block_lo', [TStart ,TFinal]);
    % output from simulink model
    acc_data(end + 1, :) = {sprintf('x_a = %.1f', x_a), a_n_data.Data};
end

zeros_tfs = zeros(6, 4);
for i = 1:length(tf_list)
    zeros_tfs(i, :) = zero(tf_list{i});
end
plot_acc(time, acc_data)



function [] = plot_acc(time, data_cell)
%% change properties
size_data = size(data_cell);
number_datasets = size_data(1);
figure;
for i = 1:number_datasets
    plot(time, data_cell{i, 2}, 'Linewidth', 2);
    hold on;
end
xlabel('Time, t [s]');
ylabel('Normal accelration [m/s^2]');
title('Normal acceleration for different accelerometer positions')

legend_list = {};
for i = 1:number_datasets
    legend_list(end + 1) = {data_cell{i, 1}};
end
legend(legend_list);
saveas(gcf, 'an_changing_xa.png');
end