function [TVA_b] = TVA_Plant(TVA_b_1, TVA_b_2, TVA_cmd_b_1, TVA_cmd_b_2)

TVA_b = 0.9965 * TVA_b_1 - 0.3422 * TVA_b_2 + 0.2038 * TVA_cmd_b_1 + 0.142 * TVA_cmd_b_2;


% s = tf('s');
% 
% t = 0.01; % sample time [s]
% 
% num = 1 ;%* exp(-0.011 * s * t);
% den = 1 + 2 * 0.697 * 0.013 * s + 0.013^2 * s^2;
% TVA = num/den;
% 
% num = 1 * exp(-0.03 * s * t);
% den = (1 + 2 * 0.707 * 0.01 * s + 0.01^2 * s^2);
% DMARS = num/den;
% 
% % H = feedback(TVA,-1)
% % bode(H)
% 
% opts = bodeoptions('cstprefs');
% opts.MagUnits = 'abs';
% opts.Grid = 'on';
% opts.FreqScale = 'Linear';
% opts.Xlim = [0 100];
% % opts.Ylim = [0 2];
% 
% bode(TVA, opts, 'r')
% hold on;
% bode(DMARS, opts, 'b');
% 
% 
% TVAd = c2d(TVA, t);
% 
% bode(TVAd, 'g')
% 
% iztrans(TVAd)


