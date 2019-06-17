function [FG_d] = Gravitational_Force_in_DLR_NRS(latd, alt, M)
% Função que calcula a Força Gravitacional em um determinado momento no 
% Triedo de navegação do DLR
%
% INPUTS ESCALARES: 
% R_l:   Raio local instantaneo   [m]
% L_C:   Latitude instantânea     [rad]
% 
% M  :   Massa instantânea        [kg]
%
% OUTPUT VETORIAL:
% FG_d: Força gravitacional no triedo de navegação do DLR
%
% Autor: Roberto Brusnicki
% Data: 21/11/18


R_e = 6378137;                                          %Raio equatorial                     [m]
f = 1/298.257223563;                                    %Achatamento polar terrestre         [-]
D_l = f * sin(2*latd) * (1 - f/2 + 2*f*sin(latd)^2);    %latd - latc                         [rad]
L_C = latd - D_l;                                       %geocentric latitude                 [rad]
R_l = R_e / sqrt( 1 + ((1-f)^-2 - 1) * sin(L_C)^2 );    %Raio local do elipsóide terrestre   [m]

R_l = R_l + alt;                    %Total radius from center of earth until the vehicle     [m]

%Constante gravitacional geocêntrica terrestre. [m^3/s^2]
mi_g = 3.986004415e14; 
J2   = 1.0826257e-3;
J3   = -2.532153e-6;
J4   = -1.6109876e-6;

%Componentes vetoriais da ac. gravitacional local no triedro geocêntrico (Inercial)
aux1 = 3/2 * J2 * (R_e/R_l)^2 * (3*sin(L_C)^2 - 1);
aux2 = 2 * J3 * (R_e/R_l)^3 * sin(L_C) * (5*sin(L_C)^2 - 3);
aux3 = 5/8 * J4 * (R_e/R_l)^4 * (35*sin(L_C)^4 - 30*sin(L_C)^2 + 3);

gl_rC = -mi_g/R_l^2 * (1 - aux1 - aux2 - aux3);

aux1 = J2 * sin(L_C);
aux2 = 1/2 * J3 * R_e/R_l * (5*sin(L_C)^2 - 1);
aux3 = 5/6 * J4 * (R_e/R_l)^2 * sin(L_C) * (7*sin(L_C)^2 - 1);

gl_tC = 3 * mi_g/R_l^2 * (R_e/R_l)^2 * cos(L_C) * (aux1 + aux2 + aux3);

FG_d = [ M * gl_tC,     0,   M * gl_rC];