
% load('2018.12.04_Greenland_east_with_DLR_att_100Hz.mat');
% load('2018-07-03_VS50_conf3_with_DLR_att.mat');
% load('2018.12.04_result6DOF_VS50_Greenland_east_massaDLR_alfa1_with_DLR_att.mat');
% load('2018.12.04_result6DOF_VS50_S44ativo_alcantara_WITH_DLR_att_100Hz.mat');

data_100hz = Untitled;
clear('data2');


% REMEMBER TO CHANGE THIS IF THIS VALUE IS DIFERENT IN VS50_DYNAMICS_DLR.M
Nz_d  = 0.820;                   % Nozzle exit diameter                                     [m]
Nzl_S = pi*(Nz_d)^2;             % Nozzle exit area                                         [m^2]
dt = 0.01;
T = 85;

% %%
% tempo = (0:dt:T)';              % Vetor de tempo
% n = length(tempo);              % número de iterações
% data_size =  size(data);        % Tamanho da matriz de dados de entrada
% 
% 
% % Nova matriz de dados amostrada em (1/dt) Hz
% data_100hz = zeros(n, data_size(2));    
% 
% for i=1:data_size(2)
%     data_100hz(:,i) = interp1(data(:,1),data(:,i),tempo);
% end
% 
% 

%% Computes the Thrust of S50 in vaccum, and adds it as the last column of the table
[Temp, a, Patm, rho] = atmosisa(1e3 * data_100hz(:,38));                 % Computes atmospheric pressure given the altitude
clear('a');
clear('T');
clear('rho');
data_100hz( :, 55 ) = data_100hz(:,12) + Nzl_S * Patm';   % Thrust Magnitude in Vaccum    [N]

% save('2018-07-03_VS50_conf3_with_DLR_att_100Hz.mat', 'data_100hz');
% save('2018-07-03_VS50_conf3_with_DLR_att_100Hz.mat', 'data_100hz');
% save('2018.12.04_result6DOF_VS50_Greenland_east_massaDLR_alfa1_with_DLR_att_100Hz.mat', 'data_100hz');
save('2018.12.04_result6DOF_VS50_S44ativo_alcantara_WITH_DLR_att_100Hz_plus_Thrust.mat', 'data_100hz');
save 2018.12.04_result6DOF_VS50_S44ativo_alcantara_WITH_DLR_att_100Hz_plus_Thrust.txt data_100hz -ascii
% csvwrite('file.csv',data_100hz)
