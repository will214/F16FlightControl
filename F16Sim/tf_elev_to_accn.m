s = tf('s');
H_an_el = (0.421*s^9 - 0.6734*s^8 - 23.46* s^7 - 71.02 *s^6 - 229* s^5 - 359.5*s^4 - 2.766* s^3 + 0.034* s^2 + 0.0002239 * s - 7.188e-16)/...
    (s^10 + 24.5 *s^9 + 102.2 *s^8 + 344.8 *s^7 + 765.8 *s^6 + 915.2* s^5 + 685.6*s^4 + 19.19 *s^3 + 4.86 *s^2 + 0.05328 * s - 5.169e-15);
poles_h_an_el = pole(H_an_el);
zeros_h_an_el = zero(H_an_el);
H_min = minreal(H_an_el);
t = linspace(0, 5, 1000);
neg_step = -ones(size(t));
[y, t, u] = lsim(H_approx, neg_step, t);
plot(t, y);