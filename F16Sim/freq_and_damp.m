function [freq, damp] = freq_and_damp(eigenvalue)
freq = (real(eigenvalue)^2 + imag(eigenvalue)^2)^(0.5);
damp = ((real(eigenvalue)^2/(imag(eigenvalue)^2))/(1 + real(eigenvalue)^2/(imag(eigenvalue)^2)))^(0.5);
end