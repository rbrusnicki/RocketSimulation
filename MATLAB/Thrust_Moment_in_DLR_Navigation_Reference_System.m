function [MFE] =  Thrust_Moment_in_DLR_Navigation_Reference_System(FE, D_NB, cg, le)
% Fun��o que calcula o Momento devido a For�a Propulsiva no Triedo de Navega��o do DLR para um 
% instante de tempo
%
% INPUTS ESCALARES:
% cg: Center of Mass in the longitudinal axis of body           [m]
% le: center of mass flow in the longitudinal axis of body      [m]
% 
% INPUTS VETORIAIS:
% q: Quat�rnion de atitude                                      [-]
% FE: For�a Propulsiva no triedo de Navega��o do DLR            [N] 	
%
% OUTPUT VETORIAL:
% MFE: Propulsive Moment in DLR Navigation Reference System     [N.m]
%
% Author: Roberto Brusnicki
% Date: 28/11/2018

% Magnitude of Re
re = le - cg;

% Vector Re in Navigation Reference System
Re = D_NB' * [0; 0; re];

% Propulsive Moment in DLR Navigation Reference System
MFE = cross(Re, FE')';
