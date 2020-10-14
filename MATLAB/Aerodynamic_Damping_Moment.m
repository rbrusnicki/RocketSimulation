function [MAd] = Aerodynamic_Damping_Moment(D_NB, W_b, Coef, Pdin, V, V_wind)
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
% Autor: Roberto Brusnicki
% Data: 06/12/2015
%
% Diâmetro do S50 [m]

D = 1.72;               %    1.72;

% Area de Referência
Sref = pi * ( D/2 )^2;  

V_norm = norm( V - V_wind );


% Momento Aerodinamico de Amortecimento no Triedo do corpo do DLR
if V_norm == 0
    MAd_b = [0; 0; 0];
else
    MAd_b = - (0.5 * Pdin * Sref * D^2 / V_norm) * Coef * W_b';
end

% Momento Aerodinamico de Amortecimento no Triedo de Navegação do DLR
MAd = ( D_NB' * MAd_b )';