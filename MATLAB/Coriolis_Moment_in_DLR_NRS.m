function [Mco] = Coriolis_Moment_in_DLR_NRS(Ixx_p, Iyy_p, Izz_p, D_NB, M_ponto, W_body, cg, le)
% Função que calcula o Momento de Coriolis no Triedo de Navegação para um instante
% de tempo
%
% INPUTS ESCALARES:
% M_ponto: derivada da massa                               [Kg/s]
% cg: Center of Mass in the longitudinal axis of body      [m]
% le: center of mass flow in the longitudinal axis of body [m]
% 
% INPUTS VETORIAIS:
% q: Quatérnion de atitude                                 [-]
% W_body: Angular Velocity in DLR Body Reference System    [rad/s] 	
%
% INPUT MATRICIAL:
% I_P : derivative of the moment of Inertia Matrix         [Kg.m^2/s]
%
% OUTPUT VETORIAL:
% Mco: Coriolis Moment in DLR Navigation Reference System  [N.m]
%
% Author: Roberto Brusnicki
% Date: 10/12/2018

% Magnitude of Re
re = le - cg;

% Vector Re in DLR Body Reference System
Re_b = [0; 0; re];

% Mass Flow
m = -M_ponto;

% Derivative of Moment of Inertia in DLR Body Reference System
I_p_b = diag([Ixx_p, Iyy_p, Izz_p]);

% Coriolis Moment in DLR Body Reference System
Mco_b =  -I_p_b * W_body' - m * cross(  Re_b, cross( W_body', Re_b));

% Coriolis Moment in DLR Navigation Reference System
Mco =  D_NB' * Mco_b;

Mco = Mco'; %Change from column vector to line Vector;




