state_space = reduced_2_state_model();
eig_A = eig(state_space.A);
[omega_sp, damping] = freq_and_damp(eig_A(1));
