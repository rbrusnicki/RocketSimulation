% load('2018.12.04_Greenland_east_with_DLR_att_100Hz.mat')
load('2019.10.14_VS50_alcantara_10ms.csv')

%%
data2 = X2019_10_14_VS50_alcantara_10ms;
%%
data3(:,1) = data2(:,1);            % Time [s]
data3(:,2:4) = data2(:,52:54);      % DLR attitude [pitch, yaw, roll]  [°]
data3(:,5) = data2(:,38);           % Altitude of the nominal trajectory [km] 
data3(:,6) = data2(:,41);           % Trajectory longitude reference     [°]
data3(:,7) = data2(:,43);           % Trajectory geodetic latitude reference [°]
data3(:,8) = data2(:,12);           % Thrust Magnitude in nominal trajectory [N]

%%
dlmwrite('2019.10.14_VS50_alcantara_10ms_short.csv', data3, 'delimiter', ',', 'precision', 15);
