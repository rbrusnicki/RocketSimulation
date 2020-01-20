 function[] = convert_ASTOS_data()
% Function that converts euler attitude angles used by IAE to the euler
% attitude angles used by DLR during flight.
%
% INPUTS: IAE's pitch, yaw and roll euler angles
% OUTPUTS: DLR's pitch, yaw and roll euler angles
%
%%%%%%% DLR REFERENCE SYSTEM SUMMARY %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% DLR reference system is NWU;
% X points to North, Y points to West and Z points Up;
% Pitch represents right rotations around X;
% Yaw represents right rotations around Y;
% Roll represents right rotations around Z;
% Order or rotation is Pitch-Yaw-Roll (same as 1-2-3 or X-Y-Z);
% DLR reference system follows the footprint of the rocket in the ellipsoid
% model WGS-84
%
%%%%%%% IAE REFERENCE SYSTEM SUMMARY %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% IAE reference system is ENU;
% X points to Up, Y points to East and Z points North;
% Pitch represents right rotations around Y;
% Yaw represents right rotations around Z;
% Roll represents right rotations around X;
% Order or rotation is Pitch-Yaw-Roll (same as 2-3-1 or Y-Z-X);
% IAE reference system is fixed at the moment of lift-of
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%
% Authors:
% 1st Lt Roberto BRUSNICKI - IAE/ACE-C - rbrusnicki@gmail.com
% Guilherme Silveira - IAE/ACE-V
%
% 31/10/2018
    close all;
    clear all;
    % Earth rotation velocity [rd/s]
    w_t = 7.292115000e-5;
%     load('2018-07-03_VS50_conf3.mat'); 
    load('result6DOF_VS50_S44inert_alcantara_massaDLR_alfa1_2019_10_14.mat');
    data = result6DOF_VS50_S44inert_alcantara_massaDLR_alfa1_2019_10_14;
    
    
    T   = data(:,1);   N = length(T);
    emp = data(:,12);
    Ixx = data(:,13);
    Iyy = data(:,14);
    Izz = data(:,15);
    massa = data(:,25);
    Xcg   = data(:,28);
    alt   = data(:,38);
    long  = data(:,41) * pi/180;
    gdlat = data(:,43) * pi/180;
    DLR_pitch = zeros(N,1);
    DLR_yaw = zeros(N,1);
    DLR_roll = zeros(N,1);
    
    theta = data(:,2) * pi/180;
    psi = data(:,3) * pi/180;
    phi = data(:,4) * pi/180;
    
    
    N = length(T);
    initial_azimuth = 50 * pi/180;

%       TEST
%       T = [0:6*3600/51:6*3600-6*3600/51]';
%       N = length(T);
%       alt = zeros(51,1);
%       lon = zeros(51,1);
%       latd = zeros(51,1);
%       theta = zeros(51,1);
%       psi = [0:pi/2/51:pi/2-pi/2/51];
%       phi = zeros(51,1);
%       initial_azimuth = 0 * pi/180;

    %%
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
    DLR_pitch = DLR_pitch * 180/pi;
    DLR_yaw = DLR_yaw * 180/pi;
    DLR_roll = DLR_roll * 180/pi;
    
    
%    save('2018-07-03_VS50_conf3_with_DLR_att.mat', 'data');
%    save('2018.12.04_result6DOF_VS50_Greenland_east_massaDLR_alfa1_with_DLR_att.mat', 'data');
%    save 2018.12.04_result6DOF_VS50_Greenland_east_massaDLR_alfa1_with_DLR_att.txt data -ascii

                    
    fileID = fopen('result6DOF_VS50_S44inert_alcantara_massaDLR_alfa1_2019_10_14.txt','w');
    fprintf(fileID,'t[s]\temp[N]\tIxx[kgm2]\tIyy[kgm2]\tIzz[kgm2]\tmassa[kg]\tXcg[m]');
    fprintf(fileID,'\taltitude[km]\tlong[deg]\tgdLat[deg]\tDLR_pitch[deg]\tDLR_yaw[deg]\tDLR_roll[deg]\r\n');
    for t = 1:N
        fprintf(fileID,'%9.9f\t%9.9f\t',T(t),emp(t));
        fprintf(fileID,'%9.9f\t%9.9f\t%9.9f\t',Ixx(t),Iyy(t),Izz(t));
        fprintf(fileID,'%9.9f\t%9.9f\t',massa(t),Xcg(t));
        fprintf(fileID,'%9.9f\t%9.9f\t%9.9f\t',alt(t),long(t),gdlat(t));
        fprintf(fileID,'%9.9f\t%9.9f\t%9.9f\r\n',DLR_pitch(t),DLR_yaw(t),DLR_roll(t));
    end
    fclose(fileID);

    

    figure('pos',[100 100 1500 800]);
    subplot(2,1,1);
    hold on;
    plot(T,DLR_pitch, 'blue')
    plot(T,DLR_yaw, 'red')
    plot(T,DLR_roll,'green')
    title('DLR Euler Angles')
    legend('DLR-pitch','DLR-yaw','DLR-roll')
    xlabel('Time [s]')
    ylabel('Angles [º]');
    grid on;
    hold off;
    subplot(2,1,2);
    hold on;
    plot(T,theta*180/pi, 'blue')
    plot(T,psi*180/pi, 'red')
    plot(T,phi*180/pi,'green')
    title('IAE Euler Angles')
    legend('IAE-pitch','IAE-yaw','IAE-roll')
    xlabel('Time [s]')
    ylabel('Angles [º]');
    grid on;
    hold off;
    % Rough trajectoy plot for better visualization/understanding
    figure();
    distance = 6378*sqrt((long-long(1)).^2+(gdlat-gdlat(1)).^2);
    plot(distance,alt);
    title('Trajectory plane');
    xlabel('Distance from launch-pad [Km]');
    ylabel('Altitude [Km]');
    grid on;
    
    figure()
    plot(long,gdlat);
    hold
    grid on;
    xlabel('long [º]')
    ylabel('gdlat [º]');
    title('footprint')
    axis equal
    
end
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