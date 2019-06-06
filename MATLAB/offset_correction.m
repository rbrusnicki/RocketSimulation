%% Simple RC LOW PASS FILTER

f = 0.3;   % [Hz]
R = 1000;   % [Ohms]
RC = 1/(2*pi*f);    % [s]
C = RC/R;   % [F]
s = tf('s');
num = 1/(s*C);
den = 1/(s*C) + R;
LOW_PASS = num/den;     %continuous plant
t = 0.010;      % sample time
LOW_PASSd = c2d(LOW_PASS, t)    %discrete plant

% LOW_PASSd =
%  
%       0.0609 * z_1
%   -------------------
%   z_0 - 0.9391 * z_1

% y[k] = 0.0609 * u[k-1] + 0.9391 * y[k-1]

% for i = 2:5001
%     data(i,6) = 0.0609 * data(i-1,5) + 0.9391 * data(i-1,6);
% end

% data(i,1) = time
% data(i,2) = original
% data(i,3) = received from dmars (with delay)
% data(i,4) = original delaed                                    ->        data(13:5001,4) = data(1:4989,2);
% data(i,5) = offset (original delayed - received from dmars)    ->        data(:,5) = data(:,4) - data(:,3);
% data(i,6) = offset filtered               -> for above
% data(i,7) = original + offset                                  ->        data(:,7) = data(:,2) + data(:,6);
% plot(data(:,1),data(:,2:7))