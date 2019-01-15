

load('2018-07-03_VS50_conf3_with_DLR_att.mat');
tempo = (0:dt:T)';              % Vetor de tempo
n = length(tempo);              % número de iterações
data_size =  size(data);        % Tamanho da matriz de dados de entrada


% Nova matriz de dados amostrada em (1/dt) Hz
data_100hz = zeros(n, data_size(2));    

for i=1:data_size(2)
    data_100hz(:,i) = interp1(data(:,1),data(:,i),tempo);
end



%% Computes the Thrust of S50 in vaccum, and adds it as the last column of the table
[T, a, Patm, rho] = atmosisa(1e3 * data_100hz(:,38));                 % Computes atmospheric pressure given the altitude
clear('a');
clear('T');
clear('rho');
data_100hz( :, data_size(2)+1 ) = data_100hz(:,12) + Nzl_S * Patm';   % Thrust Magnitude in Vaccum    [N]

save('2018-07-03_VS50_conf3_with_DLR_att_100Hz.mat', 'data_100hz');
% csvwrite('file.csv',data_100hz)
