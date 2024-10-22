%% AUX 2018
Untitled2018 = Untitled1;

%% AUX 2019
Untitled2019 = Untitled;

%% 1st step
% import numeric data (*.txt);

Nz_d  = 0.820;                 % Nozzle exit diameter                                     [m]
Nzl_S = pi*(Nz_d/2)^2;         % Nozzle exit area                                         [m^2]

%% 2nd step: DLR ATTITUDE APPEND GENERATION

 
Untitled2019(603:end-1, :) = Untitled2019(604:end, :);
Untitled2019(803:end-1, :) = Untitled2019(804:end, :);

Untitled2019 = Untitled2019(1:end-2, :);

data2018 = convert_ASTOS_data(Untitled2018);
data2019 = convert_ASTOS_data(Untitled2019);

%% Just to check if all points are unique
% figure(2018)
% Untitled20182 = unique(Untitled2018(:,1),'stable');
% plot(Untitled2018(:,1),'b.')
% hold
% plot(Untitled20182,'ro')
% grid
% 
% figure(2019)
% Untitled20192 = unique(Untitled2019(:,1),'stable');
% plot(Untitled2019(:,1),'b.')
% hold
% plot(Untitled20192,'ro')
% grid



%% 3rd Computes the Thrust of S50 in vaccum, and adds it as the last column of the table

[~, ~, Patm, ~] = atmosisa(1e3 * data2018(:,38));         % Computes atmospheric pressure given the altitude
data2018( :, 55 ) = data2018(:,12) + Nzl_S * Patm;   % Thrust Magnitude in Vaccum    [N]

[~, ~, Patm, ~] = atmosisa(1e3 * data2019(:,38));         % Computes atmospheric pressure given the altitude
data2019( :, 55 ) = data2019(:,12) + Nzl_S * Patm;   % Thrust Magnitude in Vaccum    [N]

data2019(505:end,55) = 0;

%% 4th step: INTERPOLATION

dt = 0.01;
Tf = data2018(end,1);
m = size(data2018,2);

data_2018_10ms = zeros(round(Tf/dt) + 1, m);

data_2018_10ms(:,1) = [0:dt:Tf]';

for i = 2:m
    data_2018_10ms(:,i) = interp1(data2018(:,1), data2018(:,i), data_2018_10ms(:,1));
end

%%
dt = 0.01;
Tf = data2019(end,1);

m = size(data2019,2);

data_2019_10ms = zeros(round(Tf/dt) + 1, m);

data_2019_10ms(:,1) = [0:dt:Tf]';


for i = 2:m
    data_2019_10ms(:,i) = interp1(data2019(:,1), data2019(:,i), data_2019_10ms(:,1));
end

%% 5th step: Save in file

dlmwrite('2018.12.04_VS50_alcantara_10ms.csv', data_2018_10ms, 'delimiter', ',', 'precision', 15);
dlmwrite('2019.10.14_VS50_alcantara_10ms.csv', data_2019_10ms, 'delimiter', ',', 'precision', 15);

