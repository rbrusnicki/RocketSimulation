function [FA] = Aerodynamic_Force_in_DLR_Navigation_Reference_System(q, Cnalfa, Cnbeta, Cd, Pdin, AoA_pitch, AoA_yaw)
% Fun��o que calcula a For�a Aerodinamica em um determinado momento no Triedo de Navega��o do DLR
%
% INPUT VETORIAL
%  q: quaternion de atitude
%
% INPUTS ESCALARES:
%  Cnalfa: Coeficiente...
%  Cnbeta: Coeficiente...
%  Cd: Coeficiente...
%  Pdin: Press�o Dinamica [Pa]
%  AoA_pitch: Angulo de Ataque no referencial de navega��o do DLR
%  AoA_yaw: Angulo de Derrapagem no referencial de navega��o do DLR
%
% OUTPUT VETORIAL:
%  FA_b: For�a Aerodinamica no triedo do corpo do DLR
%
% Autor: Roberto Brusnicki
% Data: 12/12/18

% Di�metro do S50 [m]
D = 1.46;               %    1.72;

% Area de Refer�ncia
Sref = pi * ( D/2 )^2;  

% DLR Euler Angles
pitch = atan2( -2*q(3)*q(4) + 2*q(1)*q(2) ,  q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2 );
yaw   =  asin(  2*q(2)*q(4) + 2*q(1)*q(3) );
roll  = atan2( -2*q(2)*q(3) + 2*q(1)*q(4), q(1)^2 + q(2)^2 - q(3)^2 - q(4)^2 );

% Magnitude da For�a de arrasto devida ao Angulo de Ataque
FA_pitch = Pdin * Sref * Cnalfa * AoA_pitch;

% Magnitude da For�a de arrasto devida ao Angulo de Derrapagem
FA_yaw   = Pdin * Sref * Cnbeta * AoA_yaw;

% Magnitude da For�a de arrasto na dire��o Longitudinal do Foguete
FA_long  = Pdin * Sref * Cd;

% For�a de arrasto devida ao Angulo de Ataque no triedo de Navega��o do DLR
FAp = [    0                   ;
         FA_pitch * cos(pitch) ;
         FA_pitch * sin(pitch) ];
     
% For�a de arrasto devida ao Angulo de Derrapagem no triedo de Navega��o do DLR
FAy = [ -FA_yaw * cos(yaw) ;
           0                 ;
         FA_yaw * sin(yaw)   ];
     
% For�a de arrasto na dire��o Longitudinal do Foguete, no triedo de Navega��o do DLR 
FAl = [ -FA_long * sin(yaw)            ;
         FA_long * sin(pitch) * cos(yaw)
        -FA_long * cos(pitch) * cos(yaw) ];

% For�a Aerodin�mica total
FA = (FAp + FAy + FAl)';    % Precisa retornar como vetor linha





