clear;
load('lofiss/Alofiss.mat', 'A_lo')
load('lofiss/Blofiss.mat', 'B_lo')
load('lofiss/Clofiss.mat', 'C_lo')
load('lofiss/Dlofiss.mat', 'D_lo')

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

%Short Period
l3=eig_lo(3);
l4=eig_lo(4);
o_sp = sqrt(real(l3)^2+imag(l4)^2); %natural frequency
d_sp = -real(l3)/o_sp; %damping ratio
P_sp = 2*pi/imag(l3);   %period
T = lo

%Phugoid
l1=eig_lo(1);
l2=eig_lo(2);
