altitude = 15000;
velocity = 500;

fi_flag_Simulink = 0;
% trim settings
thrust = 5000;          % thrust, lbs
elevator = -0.09;       % elevator, degrees
alpha = 8.49;              % AOA, degrees
rudder = -0.01;             % rudder angle, degrees
aileron = 0.01;            % aileron, degrees

[trim_state, trim_thrust, trim_control, dLEF, UX] = trim_F16(thrust, elevator, alpha, aileron, rudder, velocity, altitude);

[A, B, C, D] = linmod('LIN_F16Block_lo', [trim_state; trim_thrust; trim_control; dLEF; -trim_state(8)*180/pi], ... 
                                              [trim_thrust; trim_control]);

thrust = 0;  % Since this a linear model
deltaT = 0.001;
TStart = 0; 
TFinal = 20;

% initial states are already set in the model
sim('SS_F16_Block_lo', [TStart TFinal]);