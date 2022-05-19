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

dt = 0.005;
Tf = data2018(end,1);
m = size(data2018,2);

data_2018_5ms = zeros(round(Tf/dt) + 1, m);

data_2018_5ms(:,1) = [0:dt:Tf]';

for i = 2:m
    data_2018_5ms(:,i) = interp1(data2018(:,1), data2018(:,i), data_2018_5ms(:,1));
end


dt = 0.005;
Tf = data2019(end,1);
m = size(data2019,2);

data_2019_5ms = zeros(round(Tf/dt) , m);

data_2019_5ms(:,1) = [0:dt:Tf]';

for i = 2:m
    data_2019_5ms(:,i) = interp1(data2019(:,1), data2019(:,i), data_2019_5ms(:,1));
end

%% 5th step: Save in file

dlmwrite('2018.12.04_VS50_alcantara_5ms.csv', data_2018_5ms(1:16401,:), 'delimiter', ',', 'precision', 15);
dlmwrite('2019.10.14_VS50_alcantara_5ms.csv', data_2019_5ms(1:16401,:), 'delimiter', ',', 'precision', 15);

%% GANHOS
PID_deg = PID_deg(1:8250,:);

T1  = 0:0.010:82.5-0.01;
T2 =  0:0.005:82.5;

for i = 1:3
    PID_deg_labVIEW(:,i) = interp1(T1',             PID_deg(:,i), T2');
    IPD_deg_labVIEW(:,i) = interp1(T1', scheduled_gains_IPD(:,i), T2');
end



dlmwrite('PID_deg_labVIEW.csv', PID_deg_labVIEW, 'delimiter', ',', 'precision', 15);
dlmwrite('IPD_deg_labVIEW.csv', IPD_deg_labVIEW, 'delimiter', ',', 'precision', 15);


