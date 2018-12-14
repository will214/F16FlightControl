function [t,u]= input_function(T_final, time_step, square_length, square_size, time_to_square, input_type, n_inputs)     
    t = 0:time_step:T_final-time_step;
    if input_type == "Single"
        u = [zeros(time_to_square/time_step,1);ones(square_length/time_step,1)*square_size;zeros((T_final-time_to_square-square_length)/time_step,1)];
        if n_inputs == 2
            u = [zeros(T_final/time_step,1) u];
        end
    elseif input_type == "Doublet"
        u = [zeros(time_to_square/time_step,1);ones(square_length/time_step,1)*square_size;ones(square_length/time_step,1)*(-square_size);zeros((T_final-time_to_square-2*square_length)/time_step,1)];
        if n_inputs == 2
            u = [zeros(T_final/time_step,1) u];
        end
    elseif input_type == "Step"
        u = [zeros(time_to_square/time_step,1);ones((T_final-time_to_square)/time_step,1)*square_size];
        if n_inputs == 2
            u = [u zeros(T_final/time_step,1)];
        end
    end
end