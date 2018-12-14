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

[trim_state_lin, trim_thrust_lin, trim_control_lin, dLEF, UX] = trim_steady_state(thrust, elevator, alpha, aileron, rudder, velocity, altitude);
load_system('LIN_F16Block');

% params for linear model
deltaT = 0.001;
TStart = 0; 
TFinal = 0.05;
time = linspace(0, TFinal, TFinal/deltaT + 1);
thrust = 0;  % Since this a linear model
accelerometer_pos = [0, 5, 5.9, 6, 7, 15] * 0.3048;
% accelerometer_pos = [0];
tf_list = {};
acc_data = {};

for x_a = accelerometer_pos

    [A, B, C, D] = linmod('LIN_F16Block', [trim_state_lin; trim_thrust_lin; trim_control_lin; dLEF; -trim_state_lin(8)*180/pi], ... 
                                              [trim_thrust_lin; trim_control_lin]);
    % initial states are already set in the model
    system = ss(A, B, C, D);
    H_mimo = tf(system);
    H_an_el = minreal(H_mimo(19, 2));
    tf_list(end + 1) = {H_an_el};
    sim('SS_F16_Block', [TStart ,TFinal]);
    % output from simulink model
    acc_data(end + 1, :) = {sprintf('x_a = %.1f ft', x_a/0.3048), a_n_data.Data};
end

zeros_tfs = zeros(6, 4);
for i = 1:length(tf_list)
    zeros_tfs(i, :) = zero(tf_list{i});
end

plot_acc(time, acc_data, 0)


function [] = plot_acc(time, data_cell, step_response)
%% change properties
size_data = size(data_cell);
number_datasets = size_data(1);
figure;
for i = 1:number_datasets
    plot(time, data_cell{i, 2}, 'Linewidth', 2);
    hold on;
end

grid on;
xlabel('Time, t [s]');
ylabel('Normal accelration [m/s^2]');
if (step_response)
    title('Normal acceleration response for an elevator step')
else
    title('Normal acceleration for different accelerometer positions')
end

legend_list = {};
for i = 1:number_datasets
    legend_list(end + 1) = {data_cell{i, 1}};
end
if (step_response)
    title_name = 'step_response.png';
else
    title_name = 'an_changing_xa.png';
    legend(legend_list);
end
saveas(gcf, title_name);
end