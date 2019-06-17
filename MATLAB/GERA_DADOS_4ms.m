load('2018.12.04_VS50_alcantara.mat');

data_4ms = zeros(20501,55);

data_4ms(:,1) = [0:0.004:82]';

for i=2:55
    data_4ms(:,i) = interp1(data_100hz(:,1), data_100hz(:,i), data_4ms(:,1));
end

csvwrite('2018.12.04_VS50_alcantara_4ms.csv',data_4ms)