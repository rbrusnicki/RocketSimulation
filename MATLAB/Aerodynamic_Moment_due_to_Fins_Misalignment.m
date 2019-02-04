function [MA_f] = Aerodynamic_Moment_due_to_Fins_Misalignment(Pdin, D_NB, Cld, dl)
% Calcula o Momento Aerodinamico devido ao desalinhamento das empenas no Triedo de Navegação do DLR
%
% INPUT VETORIAL
% V: Velocidade do veículo no triedo de Navegação do Veículo        [m/s]
% Vwind: Velocidade do vento no triedo de Navegação do Veículo      [m/s]
% q: quaternion de atitude do veículo                               [-]
%
% INPUTS ESCALARES:
%  Cld: Coeficiente...
%  dl: Angulo de Desalinhamento das empenas                         [rd]
%  Pdin: Pressão Dinamica                                           [Pa]
%
% OUTPUT VETORIAL:
%  MA_f: Momento Aerodinamico devido ao desalinhamento das empenas  [N.m]
%
% Autor: Roberto Brusnicki
% Data: 03/12/18

Dref = 1.46;             % Diâmetro de referencia utilizado para levantar os coeficientes       [m]
Sref = 1.674155;         % Area de referencia utilizado para levantar os coeficientes           [m^2]
% Lref = 11.5559;        % Comprimento de referencia utilizado para levantar os coeficientes    [m]

% Magnitude of the moment
ma_f = Pdin * Sref * Dref * Cld * dl ;

% Moment in body reference system
MA_fb = [0; 0; ma_f];

% Moment in DLR Navigation Reference System
MA_f = D_NB' * MA_fb;

