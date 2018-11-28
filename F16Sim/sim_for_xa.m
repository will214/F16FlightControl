function [] = sim_for_xa()
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

deltaT = 0.01;
TStart = 0; 
TFinal = 5;
time = linspace(0, TFinal, TFinal/deltaT + 1);
thrust = 0;  % Since this a linear model

[A, B, C, D] = linmod('LIN_F16Block_lo', [trim_state; trim_thrust; trim_control; dLEF; -trim_state(8)*180/pi], ... 
                                          [trim_thrust; trim_control]);
% initial states are already set in the model
sim('SS_F16_Block_lo', [TStart ,TFinal]);
end