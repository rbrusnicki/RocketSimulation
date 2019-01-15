function [Fco] = Coriolis_Force_in_DLR_Navigation_Reference_System(D_NB, M_ponto, W_body, cg, le)
% Função que calcula a Força de Coriolis no Triedo de Navegação para um instante
% de tempo
%
% INPUTS ESCALARES:
% M_ponto: derivada da massa				   [Kg/s]
% cg: Center of Mass in the longitudinal axis of body      [m]
% le: center of mass flow in the longitudinal axis of body [m]
% 
% INPUT VETORIAL:
% q: Quatérnion de atitude				 [-]
% W_body: Angular Velocity in DLR Body Reference System  [rad/s] 	
%
% OUTPUT VETORIAL:
% Fco: Coriolis Force in DLR navigation Reference System [N]
%
% Author: Roberto Brusnicki
% Date: 26/11/2015

% Magnitude of Re
re = le - cg;

% Vector Re in Navigation Reference System
Re = D_NB' * [0; 0; re];

% Mass Flow
m = -M_ponto;

% Angular velocity in DLR Navigation Reference System
OMEGA = D_NB' * W_body';

% Coriolis Force in Navigation Reference System
Fco = -2 * m * cross( OMEGA, Re);


Fco = Fco'; %Change from column vector to line vector



