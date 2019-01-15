function [FE_b] = Thrust_Force_in_DLR_Body_Reference_System(Fe, Beta)
% Função que calcula a Força de Empuxo no Triedo do Corpo para um instante
% de tempo
%
% INPUTS ESCALARES:
% Fe: Magnitude da força de empuxo naquele instante                                              [N]
% Beta: Angulo de comando dos atuadores 315 e 225 no "referencial do Corpo do DLR"              [rad]
%
% OUTPUT VETORIAL:
% FE_B: Força de empuxo representada no referencial do Corpo do DLR
%
% Autor: Roberto Brusnicki
% Data: 21/11/2015


% Beta_pitch: Angulo de comando em pitch do TVA no referencial do Corpo do DLR  [rad]
% Beta_yaw: Angulo de comando em yaw do TVA no referencial do Corpo do DLR      [rad]
 Beta_pitch = sqrt(2)/2 * Beta(1) - sqrt(2)/2 * Beta(2);
 Beta_yaw = sqrt(2)/2 * Beta(1) + sqrt(2)/2 * Beta(2);


% if motor pressure is less than atmospheric pressure
if Fe < 0
    % Return no Thrust
    FE_b = [0 0 0];
else
    
    
    % Longitudinal Thrust Magnitude
    Fez = Fe ./ sqrt( tan(Beta_yaw).^2 + tan(Beta_pitch).^2 + 1 ); 

    % Thrust Vector Force in Body Reference System
    FE_b = [ tan(Beta_yaw).*Fez, -tan(Beta_pitch).*Fez,  Fez];
end