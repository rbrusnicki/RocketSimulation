%% 4th order Low-pass
function [rate_kp1, value_filt_kp1] = Low_pass2_10msec(D, w0, value_filt_k, rate_k, value_k)

    rate_kp1 = 0.01 * w0*w0 * (value_k - value_filt_k) + rate_k;
    value_filt_kp1 = value_filt_k + 0.01*(rate_kp1 - 2*D*w0 * value_filt_k);
    
end

% D = 1.0;
% w0 = 30.0;