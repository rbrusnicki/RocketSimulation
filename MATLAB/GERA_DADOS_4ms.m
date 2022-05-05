% load('2018.12.04_VS50_alcantara.mat');
load('2019.10.14_result6DOF_VS50_S44inert_alcantara_massaDLR_alfa1_100hz.mat');

% dt = 0.005;
dt = 0.01;

% data_5ms = zeros(82.5/dt + 1, 55);
data_10ms = zeros(82.5/dt + 1, 55);

% data_5ms(:,1) = [0:dt:82.5]';
data_10ms(:,1) = [0:dt:82.5]';

for i=2:55
%     data_5ms(:,i) = interp1(data_100hz(:,1), data_100hz(:,i), data_5ms(:,1));
    data_10ms(:,i) = interp1(data_100hz(:,1), data_100hz(:,i), data_10ms(:,1));
end

% data_5ms(1:2/dt,53) = zeros(2/dt,1);
data_10ms(1:2/dt,53) = zeros(2/dt,1);

% dlmwrite('2019.10.14_VS50_alcantara_5ms.csv', data_5ms, 'delimiter', ',', 'precision', 15);
dlmwrite('2019.10.14_VS50_alcantara_10ms.csv', data_10ms, 'delimiter', ',', 'precision', 15);
% csvwrite('2019.10.14_VS50_alcantara_5ms.csv',data_5ms)