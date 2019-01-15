function [MAd] = Aerodynamic_Damping_Moment(q, W_b, Coef, Pdin, V, V_wind)
% Calcula o Momento Aerodinamico de Amortecimento no Triedo de Navegação do DLR
%
% INPUT ESCALAR:
% Pdin: Pressão dinamica                                                        [Pa]
%
% INPUTS VETORIAIS:
% V: Velocidade do Veículo no triedo de navegaçãdo do DLR                       [m/s]
% V_wind: Velocidade do Vento no trideo de navegação do DLR                     [m/s]
% W_b: Velocidade angular no trideo do corpo do DLR                             [rad/s]
%
% INPUT MATRICIAL:
% Coef: Matriz diagonal com coeficientes aerodinamicos Clp, Cmq e Cnr           [1/rad]
%
% OUTPUT VETORIAL:
% MAd: Momento Aerodinamico de Amortecimento no Triedo de Navegação do DLR      [N.m]
% 
%
% Autor: Roberto Brusnicki
% Data: 06/12/2015

% Diâmetro do S50 [m]
D = 1.46;               %    1.72;

% Area de Referência
Sref = pi * ( D/2 )^2;  

V_norm = norm( V - V_wind );

% Momento Aerodinamico de Amortecimento no Triedo do corpo do DLR
if V_norm == 0
    MAd_b = [0; 0; 0];
else
    MAd_b = - (0.5 * Pdin * Sref * D^2 / V_norm) * Coef * W_b';
end
      
q0 = q(1);  % escalar component
q1 = q(2);  % ex
q2 = q(3);  % ey
q3 = q(4);  % ez

% 'Rotates' from Navigation Reference System to Body Reference System
%If r1 is a vector writen in Nav. Ref. Sys., then DCM*r1 is the same vector writen in Body Ref. Sys.
%If r2 is a vector writen in Body Ref. Sys., then DCM'*r2 is the same vector writen in Nav. Ref. Sys.
DCM = [    1-2*q2^2-2*q3^2      2*(q1*q2+q0*q3)     2*(q1*q3-q0*q2);
           2*(q1*q2-q0*q3)      1-2*q1^2-2*q3^2     2*(q2*q3+q0*q1);
           2*(q1*q3+q0*q2)      2*(q2*q3-q0*q1)     1-2*q1^2-2*q2^2];

MAd = ( DCM' * MAd_b )';
       