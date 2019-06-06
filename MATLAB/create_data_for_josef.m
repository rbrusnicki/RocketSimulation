load('2018.12.04_Greenland_east_with_DLR_att_100Hz.mat')
data3(:,1) = data2(:,1);
data3(:,2:4) = data2(:,52:54);
data3(:,5) = data2(:,38);
data3(:,6) = data2(:,41);
data3(:,7) = data2(:,43);
data3(:,8) = data2(:,12);

% save 28032019_112221.txt Untitled -ascii
% csvwrite('20181204_Alcantra.csv',data_100hz)
