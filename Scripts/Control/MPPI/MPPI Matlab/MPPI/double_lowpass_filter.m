function filtered = double_lowpass_filter(unfilter, filter_param, idx)
persistent p_output p_p_output

if idx == 1
    p_output = 0;
    p_p_output = 0;
end

ts = filter_param.ts;
cutoff_freq = filter_param.cutfreq;
epsilon = filter_param.epsilon;

wn = 2*pi*cutoff_freq;

coeff_b1 = -2*exp(-epsilon*wn*ts)*cos(wn*ts*sqrt(1-epsilon^2));
coeff_b2 = exp(-2*wn*epsilon*ts);
coeff_a1 = 1+coeff_b1 + coeff_b2;

filtered = coeff_a1 * unfilter - coeff_b1 * p_output - coeff_b2 * p_p_output;

p_p_output = p_output;
p_output = filtered;