function [Act315_225] = Command_Conversion_TVA_To_ACT(TVA, TVA_old, roll_angle, roll_rate)
% Função que converte os comandos inerciais (DLR) do TVA em pitch e yaw para o
% triedo do corpo (DLR) do foguete. Inicialmente a função faz uma predição
% de qual será o angulo de roll do foguete no instante que a tubeira tiver
% terminado o movimento, e posteriormente faz a compensação do rolamento.
%
% INPUTS:
% T: 		    dead time           % seconds  
% TVA_p: 	    max new pitch angle (comand)        [rad]
% TVA_p_old:	max old pitch angle (comand)        [rad]
% TVA_y:		max new yaw angle   (comand)        [rad]
% TVA_y_old:	max old yaw angle   (comand)        [rad]
%
% OUTPUTS:
% TVA_p_b:    angulo de deflexão da tubeira em torno do eixo X no triedo do corpo
%             do DLR e seguindo a regra da mão direita para o sinal do angulo              [rad]
% TVA_y_b:    angulo de deflexão da tubeira em torno do eixo Y no triedo do corpo
%             do DLR e seguindo a regra da mão direita para o sinal do angulo              [rad]
% Act315:     angulo de deflexão da tubeira em torno do eixo X=-Y no triedo do corpo
%             do DLR e seguindo a regra da mão direita para o sinal do angulo              [rad]
% Act225:     angulo de deflexão da tubeira em torno do eixo X=Y no triedo do corpo
%             do DLR e seguindo a regra da mão direita para o sinal do angulo              [rad]
% 
% Authors: Josef Ettl - DLR, Roberto Brusnicki - IAE
% Date: 17/12/2018

TVA_p = TVA(1);
TVA_y = TVA(2);
TVA_p_old = TVA_old(1);
TVA_y_old = TVA_old(2);

% Roll Pediction:
roll1 = roll_angle + TVA_time(0.011, TVA_p, TVA_p_old, TVA_y, TVA_y_old) * roll_rate ;
% roll1 = roll_angle;

% Conversion from inertial to body fixed nozzle deflection angles:
[Act315, Act225] = TVAi_r(TVA_p, TVA_y, roll1);

% For debbuging pourpose
% [TVA_p_b, TVA_y_b, Act315, Act225] = TVAi_r(TVA_p, TVA_y, roll_angle);

Act315_225 = [ Act315,  Act225];

end

function [TVA_control_time] = TVA_time( T, TVA_p, TVA_p_old, TVA_y, TVA_y_old)  

	TVA_Vmax = 20 * pi/180;      % 20 degrees per second 

	pitch_time = T + abs( (TVA_p - TVA_p_old) / TVA_Vmax );
	yaw_time   = T + abs( (TVA_y - TVA_y_old) / TVA_Vmax );
    
    if pitch_time > yaw_time
        TVA_control_time = pitch_time;
    else
        TVA_control_time = yaw_time;
    end
    
end

function [Act315, Act225] = TVAi_r(TVA_p, TVA_y, roll1)
% requested inertial pitch and yaw nozzle deflection angle
%     Act315 = - TVA_y * sin(roll1 + pi/4) - TVA_p * cos(-roll1 + pi/4);     
%     Act225 = + TVA_y * cos(roll1 + pi/4) - TVA_p * sin(-roll1 + pi/4); 
    
    Act315 =  - TVA_p * sin(-roll1 + pi/4) - TVA_y * sin(roll1 + pi/4);     
    Act225 =  - TVA_p * cos(-roll1 + pi/4) + TVA_y * cos(roll1 + pi/4);
    
    % Inverse transformation:
    % TVA_y_2 =  - Act315 * cos(-roll1 + pi/4) + Act225 * sin(-roll1 + pi/4);     
    % TVA_p_2 =  - Act315 * cos( roll1 + pi/4) - Act225 * sin(roll1 + pi/4);
    
end
