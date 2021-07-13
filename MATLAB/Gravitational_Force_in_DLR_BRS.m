function [FG_b] = Gravitational_Force_in_DLR_Body_Reference_System(D_NB, FG)
% Função que calcula a Força Gravitacional em um determinado momento no 
% Triedo do corpo do DLR
%
% INPUT VETORIAL: 
% FG: Força gravitacional no triedo de navegação do DLR [N]
% q:   quaternion de atitude   
%
% OUTPUT VETORIAL:
% FG_b: Força gravitacional no triedo do corpo do DLR [N]
%
% Autor: Roberto Brusnicki
% Data: 21/11/18

FG = FG'; %change from line vector to column vector


FG_b = D_NB * FG;


FG_b = FG_b'; %Change from column vector to line vector