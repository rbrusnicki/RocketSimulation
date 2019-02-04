function [FE] = Thrust_Force_in_DLR_Navigation_Reference_System(FE_b, D_NB)
% Função que calcula a Força de Empuxo no Triedo de Navegação do DLR 
%
% INPUTS VETORIAIS:
% FE_b: Força de empuxo naquele instante no trideo do corpo do DLR [N]
% q: Quaternion de atitude	[-]
%
% OUTPUT VETORIAL:
% FE: Força de empuxo representada no triedo de Navegação do DLR
%
% Autor: Roberto Brusnicki
% Data: 26/11/2018

FE_b = FE_b';  %Change from line vector to column vector


% Thrust Vector Force in Navigation Reference System
FE = D_NB' * FE_b;


FE = FE'; %Change from line vector to column vector