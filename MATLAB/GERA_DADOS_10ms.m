load('2018.12.04_VS50_alcantara.mat');

dt = 0.005;

data_5ms = zeros(82/dt + 1,55);

data_5ms(:,1) = [0:0.005:82]';

for i=2:55
    data_5ms(:,i) = interp1(data_100hz(:,1), data_100hz(:,i), data_5ms(:,1));
end

data_5ms(1:2/dt,53) = zeros(2/dt,1);

csvwrite('2018.12.04_VS50_alcantara_5ms.csv',data_5ms)