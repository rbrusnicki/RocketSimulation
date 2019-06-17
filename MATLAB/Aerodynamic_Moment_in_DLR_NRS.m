function [MFA] =  Aerodynamic_Moment_in_DLR_NRS(FA, D_NB, cg, cp)
% Função que calcula o Momento devido a Força Propulsiva no Triedo de Navegação do DLR para um 
% instante de tempo
%
% INPUTS ESCALARES:
% cg: Center of Mass in the longitudinal axis of body           [m]
% cp: center of pressure in the longitudinal axis of body       [m]
% 
% INPUTS VETORIAIS:
% q: Quatérnion de atitude                                      [-]
% FA: Força Aerodinamica no triedo de Navegação do DLR          [N] 	
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
