function [FE_b] = Thrust_Force_in_DLR_BRS(Fe, Beta, Nozzle_misalignment)
% Fun��o que calcula a For�a de Empuxo no Triedo do Corpo para um instante
% de tempo
%
% INPUTS ESCALARES:
% Fe: Magnitude da for�a de empuxo naquele instante                                              [N]
% Beta: Angulo de comando dos atuadores 315 e 225 no "referencial do Corpo do DLR"              [rad]
% Nozzle_misalignment: Desalinhamento da tubeira [pitch, yaw]                                   [rad]
%
% OUTPUT VETORIAL:
% FE_B: For�a de empuxo representada no referencial do Corpo do DLR
%
% Autor: Roberto Brusnicki
% Data: 22/03/2019


% Beta_pitch: Angulo de comando em pitch do TVA no referencial do Corpo do DLR  [rad]
% Beta_yaw: Angulo de comando em yaw do TVA no referencial do Corpo do DLR      [rad]
 Beta_pitch =  sqrt(2)/2 * Beta(1) + sqrt(2)/2 * Beta(2);
 
 Beta_yaw   =  sqrt(2)/2 * Beta(1) - sqrt(2)/2 * Beta(2);
 


 % Adi��o do Desalinhamento da Tubeira
 Beta_pitch = Beta_pitch - Nozzle_misalignment(1);      % O sinal de menos aqui � para desalinhamento positivo causar pitch positivo
 Beta_yaw = Beta_yaw - Nozzle_misalignment(2);          % O sinal de menos aqui � para desalinhamento positivo causar yaw positivo
 
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