
data2 = load('2019.10.14_result6DOF_VS50_S44inert_alcantara_massaDLR_alfa1.mat');
data = cell2mat(struct2cell(data2));
clear data2;

latd_ref_deg= data(:,43);          % Trajectory geodetic latitude reference           [º]
lon_ref_deg = data(:,41);          % Trajectory longitude reference                   [º]
alt_ref = data(:,38) * 1e3;        % Trajectory altitude reference                    [m]

n = length(data);

% lla_smooth = [latd_ref_deg, lon_ref_deg, alt_ref];                 % Correction necessary for the current input used
% for i = 70:n-70
%     lla_smooth(i,1) = mean(latd_ref_deg(i-69:i+70));
%     lla_smooth(i,2) = mean(lon_ref_deg(i-69:i+70));
%     lla_smooth(i,3) = mean(alt_ref(i-69:i+70));
% end
% latd_ref_s = lla_smooth(:,1);
% lon_ref_s = lla_smooth(:,2);
% alt_ref_s = lla_smooth(:,3);
% clear('lla_smooth');

%% Create DLR attitude

T   = data(:,1);
N = length(T);

w_t = 7.292115000e-5;

alt   = data(:,38);
long  = data(:,41) * pi/180;
gdlat = data(:,43) * pi/180;

DLR_pitch = zeros(N,1);
DLR_yaw = zeros(N,1);
DLR_roll = zeros(N,1);

theta = data(:,2) * pi/180;
psi = data(:,3) * pi/180;
phi = data(:,4) * pi/180;


initial_azimuth = 50 * pi/180;


for t = 1:N
    %% Computes the IAE attitude matrix
    IAE_DCM = T1(phi(t))*T3(psi(t))*T2(theta(t));
    %% Defines the rotations used for the conversion
    % Rotation around X_DLR
    Rot_Z = T3(pi);
    % Rotation around Y_DLR
    Rot_Y = T2(-pi/2+gdlat(t));
    % Rotation around X_DLR
    Rot_Z2 = T3(long(1) - long(t) - w_t * T(t));
    % Rotation around Y_DLR
    Rot_Y2 = T2(-gdlat(1));
    % Rotation around Z_DLR
    Rot_X = T1(-initial_azimuth);
    %% Attitude given in IAE standard, but with the reference system
    % positioned at the footprint of the rocket (same position of the
    % DLR reference system)
    IAE_DCM_AUX = IAE_DCM * Rot_X * Rot_Y2 * Rot_Z2 * Rot_Y * Rot_Z;
    %% Final conversion from IAE euler angles to DLR euler angles (same
    % conversion used in the function IAE2DLR.m)
    DLR_DCM = [ IAE_DCM_AUX(3,:);  % DLR_X ==  IAE_Z_AUX
               -IAE_DCM_AUX(2,:);  % DLR_Y == -IAE_Y_AUX
                IAE_DCM_AUX(1,:)]; % DLR_Z ==  IAE_X_AUX
    theta_1 = atan2(-DLR_DCM(3,2), DLR_DCM(3,3));
    X1 = sin(theta_1);
    Y1 = cos(theta_1);
    X3 = DLR_DCM(1,3) * X1 + DLR_DCM(1,2) * Y1;
    Y3 = DLR_DCM(2,2) * Y1 + DLR_DCM(2,3) * X1;
    theta_3 = atan2(X3,Y3);
    X2 = DLR_DCM(3,1);
    Y2 = DLR_DCM(1,1) * Y3 - DLR_DCM(2,1) * X3;
    theta_2 = atan2(X2,Y2);
    DLR_pitch(t) = theta_1;
    DLR_yaw(t) = theta_2;
    DLR_roll(t) = theta_3;
end

data(:,52) = DLR_pitch * 180/pi;
data(:,53) = DLR_yaw * 180/pi;
data(:,54) = DLR_roll * 180/pi;

%%  Interpolação

dt = 0.01;
tf = data(end,1);
nc = size(data,2);

data_10ms = zeros(round(tf/dt) + 1,  nc + 1);

data_10ms(:,1) = [0: dt : tf]';


for i=2:nc
    x = data(:,1);
    v = data(:,i);

    [~, ind] = unique(x);

    data_10ms(:,i) = interp1(x(ind), v(ind), data_10ms(:,1));
end


%% Create Vacuum Thrust

Nz_d  = 0.820;                                          % Nozzle exit diameter                                  [m]
Nzl_S = pi*(Nz_d)^2;                                    % Nozzle exit area                                      [m^2]

[~, ~, Patm, ~] = atmosisa(1e3 * data_10ms(:,38));      % Computes atmospheric pressure given the altitude
data_10ms( :, 55 ) = data_10ms(:,12) + Nzl_S * Patm;    % Thrust Magnitude in Vaccum                            [N]


%% save
save('2018.12.04_result6DOF_VS50_S44ativo_alcantara_WITH_DLR_att_100Hz_plus_Thrust.mat', 'data_10ms');
save 2018.12.04_result6DOF_VS50_S44ativo_alcantara_WITH_DLR_att_100Hz_plus_Thrust.txt data_10ms -ascii


%% Auxiliar functions
function [T] = T1(arg)
T = [1 0 0 ;
    0 cos(arg) sin(arg);
    0 -sin(arg) cos(arg)];
end
function [T] = T2(arg)
T = [ cos(arg) 0 -sin(arg);
    0 1 0 ;
    sin(arg) 0 cos(arg)];
end
function [T] = T3(arg)
T = [ cos(arg) sin(arg) 0;
    -sin(arg) cos(arg) 0;
    0 0 1];
end