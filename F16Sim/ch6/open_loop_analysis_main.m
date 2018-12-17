clear;
load('lofiss/Alofiss.mat', 'A_lo')
load('lofiss/Blofiss.mat', 'B_lo')
load('lofiss/Clofiss.mat', 'C_lo')
load('lofiss/Dlofiss.mat', 'D_lo')
%FindF16Dynamics;
mat_lo = [A_lo B_lo; C_lo D_lo];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Longitudal Directional %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Select the components that make up the longitude A matrix
%%
A_longitude_lo = mat_lo([3 5 7 8 11 13 14], [3 5 7 8 11 13 14]);

%% Select the components that make up the longitude B matrix
%%
B_longitude_lo = mat_lo([3 5 7 8 11 13 14], [19 20]);

%% Select the components that make up the longitude C matrix
%%
C_longitude_lo = mat_lo([21 23 25 26 29], [3 5 7 8 11 13 14]);

%% Select the components that make up the longitude D matrix
%%
D_longitude_lo = mat_lo([21 23 25 26 29], [19 20]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Lateral Directional %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Select the components that make up the lateral A matrix
%%
A_lateral_lo = mat_lo([4 6 7 9 10 12 13 15 16], [4 6 7 9 10 12 13 15 16]);

%% Select the components that make up the lateral B matrix
%%
B_lateral_lo = mat_lo([4 6 7 9 10 12 13 15 16], [19 21 22]);

%% Select the components that make up the lateral C matrix
%%
C_lateral_lo = mat_lo([22 24 25 27 28 30], [4 6 7 9 10 12 13 15 16]);

%% Select the components that make up the lateral D matrix
%%
D_lateral_lo = mat_lo([22 24 25 27 28 30], [19 21 22]);

%reducing longitudinal linearized model

A_ac_lo = A_longitude_lo([2,3,4,5],[2,3,4,5]);
B_ac_lo = A_longitude_lo([2,3,4,5],[7]);
C_ac_lo = C_longitude_lo([2,3,4,5],[2,3,4,5]);
D_ac_lo = C_longitude_lo([2,3,4,5],[7]);

%reducing lateral linearized model

A_ac_la = A_lateral_lo([1,4,5,6],[1,4,5,6]);
B_ac_la = A_lateral_lo([1,4,5,6],[8,9]);
C_ac_la = C_lateral_lo([1,4,5,6],[1,4,5,6]);
D_ac_la = C_lateral_lo([1,4,5,6],[8,9]);

%Calculation of the inherent motion characteristics

%Eigenvalues

eig_lo = eig(A_ac_lo);
eig_la = eig(A_ac_la);

%%Response to non-reduced system for verification

% figure;
% sys_n_1= ss(A_longitude_lo,B_longitude_lo,C_longitude_lo,D_longitude_lo);
% step(sys_n_1) 
% title('Short Period and Phugoid non-reduced');
% 
% figure;
% sys_n_2= ss(A_lateral_lo,B_lateral_lo,C_lateral_lo,D_lateral_lo);
% impulse(sys_n_2)
% title('Dutch roll, Aperiodic roll and Spiral non-reduced');

%Short Period

l3=eig_lo(3);
l4=eig_lo(4);
o_sp = sqrt(real(l3)^2+imag(l3)^2); %natural frequency
d_sp = -real(l3)/o_sp; %damping ratio
P_sp = 2*pi/imag(l3);  %period
T_sp = log(0.5)/real(l3); %time half amplitude
%plot
figure;
sys1= ss(A_ac_lo,B_ac_lo,C_ac_lo,D_ac_lo,...
                    'StateName', {'\theta [deg]', 'V_{t} [ft/s]', '\alpha [deg]', 'q [deg/s]'}, ... 
                     'InputName', {'\delta_e'}, ...
                     'OutputName', {'\theta [deg]', 'V_{t} [ft/s]', '\alpha [deg]', 'q [deg/s]'});
%opt = stepDataOptions('StepAmplitude',1);
%step(sys1,15,opt) %step on elevator for 15 s
[time_sp, i_sp]= pulse(15,15,1);
lsim(sys1,i_sp,time_sp)
title('Short Period');


%Phugoid

l1=eig_lo(1);
l2=eig_lo(2);
o_p = sqrt(real(l1)^2+imag(l1)^2); %natural frequency
d_p = -real(l1)/o_p; %damping ratio
P_p = 2*pi/imag(l1);  %period
T_p = log(0.5)/real(l1); %time half amplitude
%plot
figure;
%opt = stepDataOptions('StepAmplitude',1);
%step(sys1,150,opt) %step on elevator
[time_p, i_p]= pulse(250,250,1);
lsim(sys1,i_p,time_p)
title('phugoid')

%Dutch roll

l5=eig_la(1);
l6=eig_la(2);
o_dr = sqrt(real(l5)^2+imag(l5)^2); %natural frequency
d_dr = -real(l5)/o_dr; %damping ratio
P_dr = 2*pi/imag(l5);  %period
T_dr = log(0.5)/real(l5); %time half amplitude
%plot
figure;
sys2= ss(A_ac_la,B_ac_la(:,2),C_ac_la,D_ac_la(:,2),...
                    'StateName', {'\phi', '\beta', 'p', 'r'}, ... 
                     'InputName', {'\delta_r'}, ...
                     'OutputName', {'\phi', '\beta', 'p', 'r'});
[time_d, i_d]= pulse(15,1,1);
lsim(sys2,i_d,time_d)
%impulse(sys2,20) %step on rudder
title('Dutch roll')
%Aperiodic roll

l7=eig_la(3);
o_ar = sqrt(l7^2); %natural frequency
t_ar = -1/l7; %time constant
T_ar = log(0.5)/real(l7); %time half amplitude
%plot
figure;
sys3= ss(A_ac_la,B_ac_la(:,1),C_ac_la,D_ac_la(:,1),...
                    'StateName', {'\phi ', '\beta', 'p', 'r'}, ... 
                     'InputName', {'\delta_a'}, ...
                     'OutputName', {'\phi', '\beta', 'p', 'r'});

[time_a, i_a]= pulse(15,1,1);
lsim(sys3,i_a,time_a)                 
%impulse(sys3,15) %step on aileron
title('Aperiodic roll')

%Spiral

l8=eig_la(4);
o_s = sqrt(l8^2); %natural frequency
t_s = -1/l8; %time constant
T_s = log(0.5)/real(l8); %time half amplitude
%plot
figure;
[time_s, i_s]= pulse(150,1,1);
lsim(sys3,i_s,time_s)

%impulse(sys3) %step on aileron
title('spiral')

