function  [t,u]= pulse(T_final, square_length, amplitude) 
    % T_final: the total simulation time
    % square_length: the time length of the pulse
    % amplitude: the amplitude of the pulse
    t = 0:0.0001:T_final-0.0001;
    u = [ones(square_length/0.0001,1)*amplitude;zeros((T_final-square_length)/0.0001,1)];
end