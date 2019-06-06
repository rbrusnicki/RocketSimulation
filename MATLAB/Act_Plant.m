%% Act_Plant without delay
% function [Act_b] = Act_Plant(TVA_b_1, TVA_b_2, TVA_cmd_b_1, TVA_cmd_b_2)
% Act_b = 0.9965 * TVA_b_1 - 0.3422 * TVA_b_2 + 0.2038 * TVA_cmd_b_1 + 0.142 * TVA_cmd_b_2;

%% Act_Plant without delay
% function [y] = Act_Plant(y_1, y_2, u_1, u_2)
% y = 0.9965 * y_1 - 0.3422 * y_2 + 0.2038 * u_1 + 0.142 * u_2;


%% Act_Plant with delay
function [Act_b] = Act_Plant(TVA_b_1, TVA_b_2, TVA_cmd_b_2, TVA_cmd_b_3, TVA_cmd_b_4)
Act_b = 0.9965 * TVA_b_1 - 0.3422 * TVA_b_2 + 0.1715 * TVA_cmd_b_2 + 0.1732 * TVA_cmd_b_3 + 0.001049 * TVA_cmd_b_4;


%%
% s = tf('s');
% t = 0.01; % sample time [s]
% 
% % num = 1 * exp(-0.011 * s);
% num = 1;
% den = 1 + 2 * 0.697 * 0.013 * s + 0.013^2 * s^2;
% TVA = num/den;
% TVAd = c2d(TVA, t)

%%
% TVA transfer function with delay
% TVAd =
%  
%            0.1715 z^2 + 0.1732 z + 0.001049
%   z^(-2) * --------------------------------
%                z^2 - 0.9965 z + 0.3422

%%
% TVA transfer function without delay
% TVAd =
%  
%      0.2038 z + 0.142
%   -----------------------
%   z^2 - 0.9965 z + 0.3422

