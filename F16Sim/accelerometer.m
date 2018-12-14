%% Obtain transfer functions of complete low-fidelity model
velocity = 500;
altitude = 15000;
x_a = 0;
FindF16Dynamics;


SS_lo = ss(A_lo,B_lo,C_lo,D_lo); % obtain state space system
% minreal(tf(SS_lo)) % obtain transfer functions for all IO combinations and print them
tf_SS_lo = minreal(tf(SS_lo));
tf_SS_lo_an = tf_SS_lo(19,2);
zeroes = zero(tf_SS_lo_an);
poles = pole(tf_SS_lo_an);

opt = stepDataOptions('StepAmplitude',-1);
t = 0:0.01:100;
[y,t] = step(tf_SS_lo_an,t, opt);
coefficients = polyfit(t(1:2), y(1:2), 1);
slope = coefficients(1);    

figure(1)
plot(t,y)
xlim([0,1])
grid on
hold on
figure(2)
plot(t,y)
xlim([0,100])
grid on
hold on