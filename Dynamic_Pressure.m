function [Pdin, Patm] = Dynamic_Pressure(h, Vrocket, Vwind)
% Função que calcula a Força Gravitacional em um determinado momento no 
% Triedo de navegação do DLR
%
% INPUTS ESCALARES:
%  h: Altitude [m]
% 
% INPUTS VETORIAIS:
%  V:  Velocity of the vehicle [m/s]
%  Vwind: Velocity of the wind [m/s]
%
% OUTPUTS ESCALARES:
% Pdin: Dynamic Pressure     [Pa]
% Patm: Atmospheric Pressure [Pa]
%
% Autor: Roberto Brusnicki
% Data: 30/11/18

V = norm(Vrocket - Vwind);

if h > 25000
    T = -131.21 + 0.00299 * h;                          % Temperature [ºC]
    p = 2.488 * ( ( T + 273.1)/216.6 )^(-11.388);       % Pressure    [kPa]
elseif h > 11000
    T = -56.46;                                         % Temperature [ºC]
    p = 22.65 * exp(1.73 - 0.000157 * h);               % Pressure    [kPa]
elseif h > 0    
    T = 15.04 - 0.00649 * h;                            % Temperature [ºC]
    p = 101.29 * ( (T+273.1)/288.08 )^5.256;            % Pressure    [kPa]
else %h=0
    T = 15.04;                                          % Temperature [ºC]
    p = 101.4009;                                       % Pressure    [kPa]
end

Patm = 1000 * p;                        % Pressure    [Pa]
rho = Patm/(286.9 * (T+273.1));         % Density     [Kg/m^3]


% Dynamic Pressure
Pdin = 0.5 * rho * V^2;