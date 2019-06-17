function [MFA] =  Aerodynamic_Moment_in_DLR_NRS(FA, D_NB, cg, cp)
% Fun��o que calcula o Momento devido a For�a Propulsiva no Triedo de Navega��o do DLR para um 
% instante de tempo
%
% INPUTS ESCALARES:
% cg: Center of Mass in the longitudinal axis of body           [m]
% cp: center of pressure in the longitudinal axis of body       [m]
% 
% INPUTS VETORIAIS:
% q: Quat�rnion de atitude                                      [-]
% FA: For�a Aerodinamica no triedo de Navega��o do DLR          [N] 	
%
% OUTPUT VETORIAL:
% MFA: Aerodynamic Moment in DLR Navigation Reference System    [N.m]
%
% Author: Roberto Brusnicki
% Date: 28/11/2018

% Magnitude of Re
ra = cp - cg;

% Vector Re in Navigation Reference System
Ra = D_NB' * [0; 0; ra];

% Aerodynamic Moment in DLR Navigation Reference System
MFA = cross(Ra, FA')';
