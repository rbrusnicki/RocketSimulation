function [y] = DMARS_plant(y_1, y_2, u_1, u_2, u_3)

y = 0.2913 * u_1 + 0.2021 * u_2 + 0.000111 * u_3 + 0.7497 * y_1 - 0.2432 * y_2;


% t = 0.01; 
% s = tf('s');
% num = 1 * exp(-0.03 * s);
% den = (1 + 2 * 0.707 * 0.01 * s + 0.01^2 * s^2);
% DMARS = num/den;
% 
% DMARSd = c2d(DMARS, t)
%
% bode(DMARSd, 'g')
% iztrans(DMARSd)


% DMARSd =
%  
%               0.3048 z + 0.1886
%   z^(-3) * -----------------------
%            z^2 - 0.7497 z + 0.2432