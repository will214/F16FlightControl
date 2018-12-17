%% Chapter 7: Design of a pitch rate command system
% Short period simplified
cd ..
clear;
clc;

altitude = 30000;    % ft
velocity = 600;      % ft/s
gravity = 9.80665;   % ft/s^2
x_a = 0;             % Position of the accelerometer
plot_short_period_comparison = 1;     % Switch to plot the comparison between complete and reduced models
plot_short_period_comparison_cont = 1;% Switch to plot the comparison between the controlled and uncontrolled models
plot_short_period_comparison_filt = 1;% Switch to plot the comparison between the models with and without the prefilter
plot_CAP = 1;                         % Switch to plot the CAP verification requirements
plot_short_period_comparison_filt_G = 1;% Switch to plot the dropback gibson criterion requirements
FindF16Dynamics;                      % Run the F16 trimming file in order to obtain the state-space columns

% Reduced state space model for the short period
A_SP = mat_lo([8 11], [8 11]);       
B_SP = mat_lo([8 11], [14]);
C_SP = mat_lo([26 29], [8 11]);
D_SP = mat_lo([26 29], [14]);

SP_sys = ss(A_SP, B_SP, C_SP, D_SP);


% Input function to reduced short period model
T_final_SP_2 = 25;
time_step_SP_2 = 0.001;
square_length_SP_2 = 1; 
square_size_SP_2 = 1;
time_to_square_SP_2 = 0;
input_type_SP_2 = "Step";  % Options: Single or Doublet or Step
n_inputs_SP_2 = 1;

[t_SP_2, u_SP_2] = input_function(T_final_SP_2, time_step_SP_2, square_length_SP_2, square_size_SP_2, time_to_square_SP_2, input_type_SP_2, n_inputs_SP_2);

% Complete longitudinal F16 model
A_longitude_lo_OL = mat_lo([7 8 5 11], [7 8 5 11]);
B_longitude_lo_OL = mat_lo([7 8 5 11], [14]);
C_longitude_lo_OL = mat_lo([25 26 23 29], [7 8 5 11]);
D_longitude_lo_OL = mat_lo([25 26 23 29], [14]);

sys_longitudinal_lo_OL = ss(A_longitude_lo_OL, B_longitude_lo_OL, C_longitude_lo_OL, D_longitude_lo_OL);

% Input function to complete longitudinal model
T_final_SP = 25;
time_step_SP = 0.001;
square_length_SP = 1;
square_size_SP = 1;
time_to_square_SP = 0;
input_type_SP = "Step";  % Options: Single or Doublet or Step
n_inputs_SP = 1;

[t_SP, u_SP] = input_function(T_final_SP, time_step_SP, square_length_SP, square_size_SP, time_to_square_SP, input_type_SP, n_inputs_SP);

% Plot the comparison between the short period reduced and complete model
if plot_short_period_comparison == 1
    ySP_2 = lsim(SP_sys, u_SP_2, t_SP_2);
    ySP = lsim(sys_longitudinal_lo_OL, u_SP, t_SP);
    figure(8)
    plot(t_SP_2, ySP_2(:,2))
    title("Short period eigenmotion: q comparison")
    xlabel('Time, t [s]')
    ylabel('q [^\circ/s]')
    grid on
    hold on
    plot(t_SP, ySP(:,4))
    plot(t_SP_2, u_SP_2, 'k--')
    plot(t_SP, u_SP, 'k--')
    hold off
    legend('Simplified SP model', 'Complete SP model', 'Step input')
end
    
% Requirements
ft_to_m = 0.3048;
omega = 0.03*velocity*ft_to_m; % rad/s
T_theta2 = 1/(0.75*omega);
dzeta = 0.5;                   % -

% Pole placement

controllability_matrix = ctrb(A_SP, B_SP);
uncontrollable_states = length(A_SP) - rank(controllability_matrix);
p1 = -dzeta*omega + omega*sqrt(1-dzeta^2)*1j; % first pole
p2 = conj(p1);                                % second pole
K = place(A_SP,B_SP,[p1,p2]);                 % controller gain

% Check for gusts
V_gust_v = 4.572;  % m/s
alpha_ind = atan(V_gust_v/(velocity*ft_to_m));  % [rad] induced gust angle of attack
elevator_deflection_gust = -K(1) * alpha_ind;   % deg

% F16 controlled model
s = tf('s');
sys_new = ss(A_SP - B_SP*K, B_SP, C_SP, D_SP);   % Close loop transfer function
H_new = tf(sys_new);                        
H_new_q = zpk(H_new(2));    % zero-pole-gain transfer function form

% Input to the F116 controlled model
T_final_SP_new = 25;
time_step_SP_new = 0.001;
square_length_SP_new = 1;
square_size_SP_new = 1;
time_to_square_SP_new = 0;
input_type_SP_new = "Step";  % Options: Single or Doublet or Step
n_inputs_SP_new = 1;

[t_SP_new, u_SP_new] = input_function(T_final_SP_new, time_step_SP_new, square_length_SP_new, square_size_SP_new, time_to_square_SP_new, input_type_SP_new, n_inputs_SP_new);

% Plot comparison between controlled and uncontrolled models
if plot_short_period_comparison_cont == 1
    ySP = lsim(SP_sys, u_SP_2, t_SP_2);            % Open loop without controller
    ySP_new = lsim(sys_new, u_SP_new, t_SP_new);   % Close loop
    figure(9)
    plot(t_SP_new, ySP_new(:,2))
    title("Short period eigenmotion")
    grid on
    hold on
    plot(t_SP_2, ySP_2(:,2))
    plot(t_SP_new, u_SP_new, 'k--')
    plot(t_SP_2, u_SP_2, 'k--')
    hold off
    legend('Simplified controlled SP model', 'Simplified uncontrolled SP model')
end

% Lead-lag filter

closed_loop_zero = zero(H_new(2));                 % Future pole of the prefilter
filter_numerator = (1+T_theta2*s);                 % Numerator of prefilter
filter_denominator = (s-closed_loop_zero);         % Denominator of the prefilter
filter = zpk(filter_numerator/filter_denominator); % Prefilter transfer function

complete_system = zpk(minreal(filter*H_new_q));    % Prefilter with closed loop model
dc_gain = evalfr(complete_system, 0);              % 1/Gain to reduce steady state error to zero of tracking task
complete_system = complete_system/(-dc_gain);      % Final prefilter with closed loop model

% Input to the closed loop system with prefilter
T_final_SP_filter = 2.5;
time_step_SP_filter = 0.001;
square_length_SP_filter = 1;
square_size_SP_filter = 1;
time_to_square_SP_filter = 0;
input_type_SP_filter = "Step";  % Options: Single or Doublet or Step
n_inputs_SP_filter = 1;

[t_SP_filter, u_SP_filter] = input_function(T_final_SP_filter, time_step_SP_filter, square_length_SP_filter, square_size_SP_filter, time_to_square_SP_filter, input_type_SP_filter, n_inputs_SP_filter);

% Plot the comparison between the models with and without prefilters
if plot_short_period_comparison_filt == 1
    ySP_filter = lsim(complete_system, u_SP_filter, t_SP_filter);
    ySP_new = lsim(sys_new, u_SP_new, t_SP_new);
    figure(10)
    plot(t_SP_new, ySP_new(:,2))
    xlim([0 t_SP_filter(end)])
    title("Pitch rate response for a positive step input")
    xlabel('Time, t [s]')
    ylabel('q [^\circ/s]')
    grid on
    hold on
    plot(t_SP_filter, ySP_filter)
    plot(t_SP_new, u_SP_new, 'k--')
    plot(t_SP_filter, u_SP_filter, 'k--')
    hold off
    legend('Simplified controlled unfiltered SP model', 'Simplified controlled filtered SP model', 'Step input')
end


%% CAP and GIBSON parameters

CAP = omega^2/(velocity/(gravity/ft_to_m*T_theta2));
DB_q = T_theta2 - 2*dzeta/omega;

% Plot CAP criteria requirements
if plot_CAP == 1
    % Category A
    figure(11)
    % Level 3
    loglog([0.15,0.15],[0.1,10], 'k')
    hold on

    % Level 1
    generate_rectangle(0.35,1.3,0.28,3.6)

    %Level 2
    generate_rectangle(0.25,2,0.16,10)

    loglog(dzeta,CAP,'bo', 'DisplayName', 'Current paramater value')

    ylim([0.1,10])
    xlim([0.1,10])
    title("Flight Phase Category A")
    xlabel('Short period damping ratio, \zeta_{sp} [-]')
    ylabel("CAP [1/gsec^2]")
    grid on

    % Category B
    figure(12)
    % Level 3
    loglog([0.15,0.15],[0.01,10], 'k')
    hold on

    % Level 2
    generate_rectangle(0.2,2,0.038, 10)

    %Level 1
    generate_rectangle(0.3,2,0.085,3.6)

    loglog(dzeta,CAP,'bo')

    ylim([0.01,10])
    xlim([0.1,10])
    title("Flight Phase Category B")
    xlabel('Short period damping ratio, \zeta_{sp} [-]')
    ylabel("CAP [1/gsec^2]")
    grid on

    % Category C
    figure(13)
    % Level 3
    loglog([0.15,0.15],[0.01,10], 'k')
    hold on

    % Level 2
    generate_rectangle(0.25,2,0.05, 10)

    %Level 1
    generate_rectangle(0.35,1.3,0.16,3.6)

    loglog(dzeta,CAP,'bo')

    ylim([0.01,10])
    xlim([0.1,10])
    title("Flight Phase Category C")
    xlabel('Short period damping ratio, \zeta_{sp} [-]')
    ylabel("CAP [1/gsec^2]")
    grid on
end

% Study of Gibson dropback criterion for tracking task
% Input for model with prefilter (pulse input) in order to analyze dropback
T_final_SP_filter_G = 20;
time_step_SP_filter_G = 0.01;
square_length_SP_filter_G = 10;
square_size_SP_filter_G = -1;
time_to_square_SP_filter_G = 0;
input_type_SP_filter_G = "Single";  % Options: Single or Doublet or Step
n_inputs_SP_filter_G = 1;

[t_SP_filter_G, u_SP_filter_G] = input_function(T_final_SP_filter_G, time_step_SP_filter_G, square_length_SP_filter_G, square_size_SP_filter_G, time_to_square_SP_filter_G, input_type_SP_filter_G, n_inputs_SP_filter_G);

% Plot q and theta response to pulse input and Gibson dropback criterion
% satisfactory design region
if plot_short_period_comparison_filt_G == 1
    
    % Obtain theta response for pulse input to elevator deflection
    ySP_filter_G = lsim(complete_system, u_SP_filter_G, t_SP_filter_G);
    y_theta = zeros(T_final_SP_filter_G/time_step_SP_filter_G,1);
    for i = 1:length(ySP_filter_G)-1
        y_theta(i+1) = y_theta(i)+ySP_filter_G(i)*time_step_SP_filter_G;
    end
    
    % Retrieve current qm, qs an DB parameters from plot
    qm = max(ySP_filter_G);
    qs = ySP_filter_G(square_length_SP_filter_G/time_step_SP_filter_G);
    qm_qs = qm/qs;
    theta_rc = y_theta(square_length_SP_filter_G/time_step_SP_filter_G);
    final_theta = y_theta(end);
    DB = theta_rc-final_theta;
    DB_qs = DB/qs;
    
     
    % Plot the pitch rate
    figure(14)
    plot(t_SP_filter_G, ySP_filter_G)
    xlim([0 t_SP_filter_G(end)])
    title("Pitch rate to block elevator input of " + square_length_SP_filter_G + " [s]")
    xlabel('time [s]')
    ylabel('q [^\circ/s]')
    grid on
    hold on
    plot(t_SP_filter_G, u_SP_filter_G, 'r--')
    plot([0,T_final_SP_filter_G],[qs, qs], 'k--', 'LineWidth',2)  % final pitch rate
    plot([0,T_final_SP_filter_G],[qm, qm], 'k', 'LineWidth',2)  % maximum pitch rate
    hold off
    text1 = strcat('qs = ', num2str(qs));
    text2 = strcat('qm = ', num2str(qm));
    legend('Pitch rate', 'Pulse input [^\circ]', text1, text2)
    
    % Plot the pitch angle
    figure(15)
    plot(t_SP_filter_G, y_theta)
    xlim([0 t_SP_filter_G(end)])
    title("Pitch angle to block elevator input of " + square_length_SP_filter_G + " [s]")
    xlabel('time [s]')
    ylabel('\theta [^\circ]')
    grid on
    hold on
    plot(t_SP_filter_G, u_SP_filter_G, 'r--')
    plot([0,T_final_SP_filter_G],[final_theta,final_theta], 'k--', 'LineWidth',2)  % final theta
    plot([0,T_final_SP_filter_G],[theta_rc, theta_rc], 'k', 'LineWidth',2)  % theta at removed control
    hold off
    text1 = strcat('\theta final = ', num2str(final_theta));
    text2 = strcat('\theta removed control = ', num2str(theta_rc));
    legend('Pitch angle', 'Pulse input [^\circ]', text1, text2)
    
    % Plot the Gibson dropback criterion
    figure(16)
    fill([0, 0, 0.05, 0.3], [1, 3, 3, 1], [0, 0.75, 0.75])
    xlim([-0.4,0.6])
    ylim([1,4])
    title('Gibson dropback criterion for tracking task')
    ylabel('q_{m}/q_{s} [-]')
    xlabel('$\displaystyle\frac{OS}{q_{s}}$ [s]  \quad \quad  \quad \quad \quad \quad  \quad \quad $\displaystyle\frac{DB}{q_{s}}$ [s]', 'interpreter', 'latex')
    grid on
    hold on
    plot(DB_qs, qm_qs, 'ro')
    plot(DB_q, qm_qs, 'rx')
    text(0.05,1.7,'Satisfactory')
    text(-0.25, 2.25,'Sluggish')
    text(0.28, 2.25,{'Abrupt bobble', 'tendency'})
    text(0.28, 3.25,'Continuous bobbling')
    legend('Allowable region', 'Current value', 'Design point')    
end

% Function to generate the rectangles for the CAP criterion plots
function generate_rectangle(x1,x2,y1,y2)
    loglog([x1,x2],[y1,y1],'k')
    loglog([x1,x2],[y2,y2],'k')
    loglog([x2,x2],[y1,y2],'k')
    loglog([x1,x1],[y1,y2],'k')
end

