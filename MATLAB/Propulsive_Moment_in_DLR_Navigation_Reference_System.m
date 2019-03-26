function [MFE] =  Propulsive_Moment_in_DLR_Navigation_Reference_System(FE, q, cg, le)
% Função que calcula o Momento devido a Força Propulsiva no Triedo de Navegação do DLR para um 
% instante de tempo
%
% INPUTS ESCALARES:
% cg: Center of Mass in the longitudinal axis of body           [m]
% le: center of mass flow in the longitudinal axis of body      [m]
% 
% INPUTS VETORIAIS:
% q: Quatérnion de atitude                                      [-]
% FE: Força Propulsiva no triedo de Navegação do DLR            [N] 	
%
% OUTPUT VETORIAL:
% MFE: Propulsive Moment in DLR Navigation Reference System     [N.m]
%
% Author: Roberto Brusnicki
% Date: 28/11/2018

q0 = q(1);  % escalar component
q1 = q(2);  % ex
q2 = q(3);  % ey
q3 = q(4);  % ez

% 'Rotates' from Navigation Reference System to Body Reference System
%If r1 is a vector writen in Nav. Ref. Sys., then DCM*r1 is the same vector writen in Body Ref. Sys.
%If r2 is a vector writen in Body Ref. Sys., then DCM'*r2 is the same vector writen in Nav. Ref. Sys.
DCM = [ (q0^2+q1^2-q2^2-q3^2)     2*(q1*q2+q0*q3)     2*(q1*q3-q0*q2)   ;
           2*(q1*q2-q0*q3)      q0^2-q1^2+q2^2-q3^2   2*(q2*q3+q0*q1)   ;
           2*(q1*q3+q0*q2)        2*(q2*q3-q0*q1)   q0^2-q1^2-q2^2+q3^2 ];

% Magnitude of Re
re = le - cg;

% Vector Re in Navigation Reference System
Re = DCM' * [0; 0; re];

% Propulsive Moment in DLR Navigation Reference System
MFE = cross(Re, FE')';
