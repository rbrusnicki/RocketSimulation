function [I] = Moment_of_Inertia_in_DLR_Navigation_Reference_System(Ixx, Iyy, Izz, q)
% Função que calcula o Momento de Inercia no Triedo de Navegação para um instante de tempo
%
% INPUTS ESCALARES:
% Ixx: Moment of inertia along x axis in DLR body Reference System          [Kg.m^2]
% Iyy: Moment of inertia along y axis in DLR body Reference System          [Kg.m^2]
% Izz: Moment of inertia along z axis in DLR body Reference System          [Kg.m^2]
% 
% INPUT VETORIAL:
% q: Quatérnion de atitude                                                  [-]
%
% OUTPUT MATRICIAL:
% I: Moment of inertia Matrix in DLR Navigation Reference System            [Kg.m^2]
%
% Author: Roberto Brusnicki
% Date: 28/11/2018

% Moment of inertia Matrix in DLR Body Reference System            
I_b = diag([Ixx, Iyy, Izz]);

q0 = q(1);  % escalar component
q1 = q(2);  % ex
q2 = q(3);  % ey
q3 = q(4);  % ez

% 'Rotates' from Navigation Reference System to Body Reference System
%If r1 is a vector writen in Nav. Ref. Sys., then DCM*r1 is the same vector writen in Body Ref. Sys.
%If r2 is a vector writen in Body Ref. Sys., then DCM'*r2 is the same vector writen in Nav. Ref. Sys.
DCM = [ (q0^2+q1^2-q2^2-q3^2)     2*(q1*q2+q0*q3)     2*(q1*q3-q0*q2);
           2*(q1*q2-q0*q3)      q0^2-q1^2+q2^2-q3^2   2*(q2*q3+q0*q1);
           2*(q1*q3+q0*q2)        2*(q2*q3-q0*q1)   q0^2-q1^2-q2^2+q3^2];

% Moment of Inertia in DLR Navigation Reference System
I = DCM' * I_b;