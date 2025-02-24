% function [lon, latd, alt] = VS50_dynamics_DLR()
% cd C:\Users\rbrus\Documents\RocketSimulation\MATLAB
clc;
clear all;
close all;
set(0,'DefaultAxesXGrid','on')
set(0,'DefaultAxesYGrid','on')
set(0,'DefaultLineLineWidth', 2);
set(0,'defaultFigurePosition', [10 200 1600 800])
colordef white
grid minor
close;

%% Dados do ve�culo

% load('2018.12.04_VS50_alcantara.mat');
load('2019.10.14_result6DOF_VS50_S44inert_alcantara_massaDLR_alfa1_100hz.mat');
% load('vento_1.mat');
% load('vento_2.mat');
load('vento_4.mat');

% load('scheduled_gains_PID.mat');
load('PID_tuned_10ms');                     % THE CHOOSEN ONE!!!
% load('IPD_robust_10ms.csv'); % HORRIVEL

load('pid_smooth.mat');
% load('GUI_PD_tuned.mat');
% load('scheduled_gui_IPD.mat');
% load('GUI_PID_10ms.csv');
% GUI_PID_10ms = X2GUI_PID_10ms;

% Constant values

n = length(data_100hz);          % number of iterations
le    = -10.9332;			 	 % Position of the 'Nozzle Throat' in Longitudinal Axis     [m]
Nz_d  = 0.820;                   % Nozzle exit diameter                                     [m]
Nzl_S = pi*(Nz_d/2)^2;           % Nozzle exit area                                         [m^2]
dt = 0.01;                       % Incremento de tempo utilizado                            [s]
Dref = 1.72;                     % Vehicle reference diameter                               [m]
Sref = pi * (Dref/2)^2;          % Vehicle reference area                                   [m^2]


% Disturbances
Nozzle_misalignment = pi/180 * [ 0  0.0 ];  % Desalinhamento da tubeira [X-pitch, Y-yaw]      [rad]
Nozzle_eccentricity = 1e-3 * [ 0  0 ];      % Ecentricidade da tubeira [X, Y]                 [m]
dl = 0.0  * pi/180;                         % Fins Misalignment                              [rad]


% Control data
e_1_int = 0;
e_2_int = 0;
e_3_int = 0;



%% Leitura dos dados invariantes de voo
time    = data_100hz(:,1);
Fe_traj = data_100hz(:,12);              % Thrust Magnitude in nominal trajectory           [N]  
alt_traj= 1e3 * data_100hz(:,38);        % Altitude of the nominal trajectory               [m]   
Fe      = data_100hz(:,55);              % Thrust Magnitude of S50 in vacuum                [N]
M       = data_100hz(:,25);              % Mass                         			        [Kg]
Cnalfa  = data_100hz(:,10);              % Aerodynamic Coeficient       			        [1/rad] 
Cnbeta  = data_100hz(:,11);              % Aerodynamic Coeficient       			        [1/rad] 
Cd      = data_100hz(:, 5);              % Cd - coeficiente de arrasto 				        [-]
Cld     = data_100hz(:, 7);              % Cld - coeficiente de 'desalinhamento'           	[-]
Clp     = data_100hz(:, 6);              % Coef. de Amortecimento Aerod. em x_b             [1/rad]
Cmq     = data_100hz(:, 8);              % Coef. de Amortecimento Aerod. em y_b             [1/rad]
Cnr     = data_100hz(:, 9);              % Coef. de Amortecimento Aerod. em z_b             [1/rad]
M_p     = data_100hz(:,34);              % d(Mass)/dt                                       [Kg/s]
CoG	    = data_100hz(:,28);              % Center of gravity in longitudinal axis	        [m]
CoP	    = data_100hz(:,31);              % Center of Pressure in longitudinal axis	        [m]
Ixx     = data_100hz(:,15);              % Moment of inertia along x axis                   [Kg.m^2]
Iyy     = data_100hz(:,14);              % Moment of inertia along y axis                   [Kg.m^2]
Izz     = data_100hz(:,13);              % Moment of inertia along z axis                   [Kg.m^2]
Ixx_p   = data_100hz(:,21);              % d(Ixx)/dt                                        [Kg.m^2/s]
Iyy_p   = data_100hz(:,20);              % d(Iyy)/dt                                        [Kg.m^2/s]
Izz_p   = data_100hz(:,19);              % d(Izz)/dt                                        [Kg.m^2/s]

latd_ref_deg= data_100hz(:,43);          % Trajectory geodetic latitude reference           [�]
lon_ref_deg = data_100hz(:,41);          % Trajectory longitude reference                   [�]
alt_ref = data_100hz(:,38) * 1e3;        % Trajectory altitude reference                    [m]

lla_smooth = [latd_ref_deg, lon_ref_deg, alt_ref];                 % Correction necessary for the current input used
for i = 1:4
    lla_smooth(i,3) = mean(alt_ref(1:i+5));
end
for i = 5:n-5
    lla_smooth(i,3) = mean(alt_ref(i-4:i+5));
end
alt_ref = lla_smooth(:,3);
for i = 70:n-70
    lla_smooth(i,1) = mean(latd_ref_deg(i-69:i+70));
    lla_smooth(i,2) = mean(lon_ref_deg(i-69:i+70));
    lla_smooth(i,3) = mean(alt_ref(i-69:i+70))-1.81017;
end
latd_ref_deg = lla_smooth(:,1);
lon_ref_deg = lla_smooth(:,2);
alt_ref = lla_smooth(:,3);
% clear('lla_smooth');

mach_number   = data_100hz(:,39);        % Velocity in Mach number for nominal trajectory   [-] 
pitch_ref_deg = data_100hz(:,52);        % DLR Pitch reference                              [�]
yaw_ref_deg   = data_100hz(:,53);        % DLR Yaw reference                                [�]

% pitch_ref_deg(1:253,1) = ones(253,1);
% pitch_ref_deg   = zeros(8201,1);
% yaw_ref_deg   = zeros(8201,1);
% pitch_ref_deg(1310:8201,1) = pitch_ref_deg(1310:8201,1) + ones(8201-1310+1,1);
% yaw_ref_deg(1310:8201,1) = yaw_ref_deg(1310:8201,1) + ones(8201-1310+1,1);
% pitch_ref_deg(4501:8201,1) = 0.4*ones(3701,1);

roll_ref_deg  = data_100hz(:,54);        % DLR Roll reference                               [�]
pitch_ref = pitch_ref_deg * pi/180;      % DLR Pitch reference                              [rad]
yaw_ref   = yaw_ref_deg * pi/180;        % DLR Yaw reference                                [rad]
roll_ref  = roll_ref_deg * pi/180;       % DLR Roll reference                               [rad]

latd_ref = latd_ref_deg * pi/180;        % Trajectory geodetic latitude reference           [rad]
lon_ref  =  lon_ref_deg * pi/180;        % Trajectory longitude reference                   [rad]

%% Dados variantes de voo (LOGDATA)
n = 8250;


TVA_cmd     = zeros(n,2);               % TVA nozzle pitch and yaw angles commands in DLR NRS. [rad]    
TVAf        = zeros(n,2);               % Filtered TVA nozzle commands in DLR NRS.             [rad]    
Act_cmd_b   = zeros(n,2);               % Nozzle angle command for actuators at 315� and  225� [rad]
Act_b       = zeros(n,2);               % Nozzle angle for actuators at 315� and at 225�       [rad]
latd        = zeros(n,1);               % Geodetic Latitude of the Vehicle during flight       [rad]
alt         = zeros(n,1);               % Geodetic Altitude of the Vehicle during flight       [m]
lon         = zeros(n,1);               % Geodetic Longitude of the Vehicle during flight      [rad]
q           = zeros(n,4);               % Quaternion of attitude                               [-]
angles      = zeros(n,3);               % Euler angles in DLR Standard: 1-2-3                  [rad]
angles_deg  = zeros(n,3);               % Euler angles in DLR Standard: 1-2-3                  [�]
acc         = zeros(n,3);               % Aceleration vector in DLR Navigation Ref. Sys.       [m/s^2]
acc_b       = zeros(n,3);               % Aceleration vector in DLR Navigation Ref. Sys.       [m/s^2]
ang_acc     = zeros(n,3);               % Angular Aceleration vector in DLR Nav. Ref. Sys.     [rad/s^2]
ang_acc_b   = zeros(n,3);               % Angular Aceleration vector in DLR Body Ref. Sys.     [rad/s^2]
V           = zeros(n,3);               % Velocity vector in DLR Navigation Ref. Sys.          [m/s]
Pdin        = zeros(n,1);               % Dynamic Pressure                                     [Pa]
Patm        = zeros(n,1);               % Atmospheric Pressure                                 [Pa]
FE          = zeros(n,3);               % Thrust Force Vector in DLR Navigation Ref. Sys.      [m]
FE_b        = zeros(n,3);               % Thrust Force Vector in DLR Body Reference System     [m]
FG          = zeros(n,3);               % Gravitational Force in DLR Navigation Ref. Sys.      [m]
FG_b        = zeros(n,3);               % Gravitational Force in DLR Body Ref. Sys.            [m]
FA          = zeros(n,3);               % Aerodynamic Force in DLR Navigation Reference System [m]
FA_b        = zeros(n,3);               % Aerodynamic Force in DLR Body Reference System       [m]
FCo         = zeros(n,3);               % Coriolis Force in DLR Navigation Reference System    [m]
FCo_b       = zeros(n,3);               % Coriolis Force in DLR Body Reference System          [m]
W           = zeros(n,3);               % Angular Velocity in DLR Navigation Reference System  [rad/s]
W_b         = zeros(n,3);               % Angular Velocity in DLR Body Reference System        [rad/s]
AoA         = zeros(n,2);               % Angle of Attack [XZ plane, YZ plane] of DLR_NRS      [rad]
AoA_deg     = zeros(n,2);               % Angle of Attack [XZ plane, YZ plane] of DLR_NRS      [�]
AoA_comp    = zeros(n,2);               % Computed AoA [XZ plane, YZ plane] of DLR_NRS         [rad]
AoA_comp_deg= zeros(n,2);               % Computed AoA [XZ plane, YZ plane] of DLR_NRS         [�]
Speed_Att   = zeros(n,2);               % Euler angles of the Speed Vector Attitude (XYZ)      [rad]
Speed_Att_comp = zeros(n,2);            % Computed Speed_Att considering no wind  [pitch, yaw] [rad]
MCo         = zeros(n,3);               % Coriolis Moment in DLR Navigation Reference System   [N.m]
MCo_b       = zeros(n,3);               % Coriolis Moment in DLR Body Reference System         [N.m]
MFE         = zeros(n,3);               % Propulsive Moment in DLR Navigation Reference Sys.   [N.m]
MFE_b       = zeros(n,3);               % Propulsive Moment in DLR Body Reference System       [N.m]
MFA         = zeros(n,3);               % Aerodynamic Moment in DLR Navigation Reference Sys.  [N.m]
MFA_b       = zeros(n,3);               % Aerodynamic Moment in DLR Body Reference Sys.        [N.m]
MA_f        = zeros(n,3);               % Moment due to Fins Misalignment in DLR Nav. Ref. Sys.[N.m]
MA_f_b      = zeros(n,3);               % Moment due to Fins Misalignment in DLR Body Ref. Sys.[N.m]
MA_d        = zeros(n,3);               % Aerodynamic Damping Moment in DLR Nav. Ref. Sys.     [N.m]
MA_d_b      = zeros(n,3);               % Aerodynamic Damping Moment in DLR Body Ref. Sys.     [N.m]
Temperature = zeros(n,1);               % Temperature                                          [K]
Oil_cons    = zeros(n,1);               % Oil Consumption                                      [L]
Velocity_in_Mach = zeros(n,1);

M_alpha     = zeros(n,1);
M_beta      = zeros(n,1);
PID_deg     = zeros(n,3);               % Attitude PID gains computed as a function of M_beta in degrees

k_acc       = zeros(n,2);               % Normalized sideways/front-back acelerations
GUI_PID     = zeros(n,3);               % Guidance PID gains computed as a function of acc_k

pitch_error = zeros(n,1);
pitch_error_int = zeros(n,1);
pitch_error_dev = zeros(n,1);

yaw_error   = zeros(n,1);
yaw_error_int = zeros(n,1);
yaw_error_dev = zeros(n,1);

latd_error  = zeros(n,1);
lon_error   = zeros(n,1);

e_1         = zeros(n,1);
e_2         = zeros(n,1);
e_3         = zeros(n,1);

delta_azi   = zeros(n,1);
delta_ele   = zeros(n,1);

ax_b        = zeros(n,1);
ay_b        = zeros(n,1);
az_b        = zeros(n,1);
ax          = zeros(n,1);
ay          = zeros(n,1);
az          = zeros(n,1);

coor = zeros(n,1);

TVA_rec     = zeros(n,2);

DMARS_data  = zeros(n,16);              % DMARS data sent in the protocol (output of the DMARS_plant)
                                        % [W_b(i,:), ang_acc_b(i,:), q(i,:), V(i,:), latd(i), lon(i), alt(i)];

% Initial Position --------------------------------------------------------

XYZ   = zeros(n,3);

alt(1,1) = 50.0;                   % Initial Altitude                 [m]
latd(1,1) = -2.3159948 * pi/180;    % Initial Geodetic Latitud         [rad]
lon(1,1) = -44.367775 * pi/180;    % Initial Longitude                [rad]

% Initial Atitude ---------------------------------------------------------

angles_deg(1,1)  = 0;
angles_deg(1,2)  = 0;
angles_deg(1,3)  = 0;

angles(1,:) = angles_deg(1,:) * pi/180;  

cang1 = cos(angles(1,1)/2);
cang2 = cos(angles(1,2)/2);
cang3 = cos(angles(1,3)/2);

sang1 = sin(angles(1,1)/2);
sang2 = sin(angles(1,2)/2);
sang3 = sin(angles(1,3)/2);

q0 =  cang1.*cang2.*cang3 - sang1.*sang2.*sang3;
q1 =  cang1.*sang2.*sang3 + sang1.*cang2.*cang3; 
q2 =  cang1.*sang2.*cang3 - sang1.*cang2.*sang3; 
q3 =  cang1.*cang2.*sang3 + sang1.*sang2.*cang3;

q(1,:) = [q0 q1 q2 q3];  

% Wind conditions ---------------------------------------------------------
% n=8201;
% V_wind      = zeros(n,3);           % Wind Velocity vector in DLR NRS [m/s]
% V_wind(1:n,1:2) = vento_4(1:n,2:3); % Wind from files

V_mod = 16; % [m/s]
V_azi = 50; % [�]
V_wind    = [-V_mod*cos(V_azi * pi/180)*ones(n,1) V_mod*sin(V_azi * pi/180)*ones(n,1)  zeros(n,1)];

%V_wind = [zeros(n,1) 10*ones(n,1)  zeros(n,1)];

% Malfa and Mbeta disturbances gains
Gain_a = 1.0;
Gain_b = 1.0;

%%
t_off = 700;
aux_flag = 0;
n = 8200;

for i = 1:(n-1)     
    %% Press�o Din�mica e Press�o Atmosf�rica local
    [Pdin(i), Patm(i), Temperature(i)] = Dynamic_Pressure( alt(i), V(i,:), V_wind(i,:) );
%     [Pdin(i), Patm(i), Temperature(i)]
    %% Direct Cossine Matrix
    D_NB = DCM_NRS_to_BRS( q(i,:) );
%     D_NB
    %% Real Angles of Attack no Triedo de Navega��o do DLR
    [AoA(i,:), Speed_Att(i,:)] =  Angle_Of_Attack_in_DLR_NRS( V(i,:), V_wind(i,:), q(i,:));
    AoA_deg(i,:) = AoA(i,:) * 180/pi;
%     aoa = AoA_deg(i,:)

    %% Low Pass Filter
    % auxiliar variables
    if i > 1
        TVA_1 = TVA_cmd(i-1,:);
        TVAf_1 = TVAf(i-1,:);
    end
    if i > 2
        TVA_2 = TVA_cmd(i-2,:);
        TVAf_2 = TVAf(i-2,:);
    end
    if i > 3
        TVA_3 = TVA_cmd(i-3,:);
        TVAf_3 = TVAf(i-3,:);
    end
    if i > 4
        TVA_4 = TVA_cmd(i-4,:);
        TVAf_4 = TVAf(i-4,:);
    end
    if i == 1
        [TVAf(i,:)] = LPF2d([0,0], [0,0], [0,0], [0,0], [0,0], [0,0], [0,0], [0,0]);
    elseif i == 2
        [TVAf(i,:)] = LPF2d(TVA_1, [0,0], [0,0], [0,0], TVAf_1, [0,0], [0,0], [0,0]);
    elseif i == 3
        [TVAf(i,:)] = LPF2d(TVA_1, TVA_2, [0,0], [0,0], TVAf_1, TVAf_2, [0,0], [0,0]);
    elseif i == 4
        [TVAf(i,:)] = LPF2d(TVA_1, TVA_2, TVA_3, [0,0], TVAf_1, TVAf_2, TVAf_3, [0,0]);
    else
        [TVAf(i,:)] = LPF2d(TVA_1, TVA_2, TVA_3, TVA_4, TVAf_1, TVAf_2, TVAf_3, TVAf_4);
    end
    %% Command Conversion from TVA Comand in DLR NRS To Actuator Command
    if i == 1
        Act_cmd_b(i,:) = Command_Conversion_TVA_To_ACT(TVAf(i,:),      [0, 0], angles(i,3),        0);
    else
        Act_cmd_b(i,:) = Command_Conversion_TVA_To_ACT(TVAf(i,:), TVAf(i-1,:), angles(i,3), W_b(i,3));
    end
%     act_cmd_b = Act_cmd_b(i,:)
    %% Act's command input to Act's plant output
    if i == 1
        Act_b(i,:) = Act_Plant(        [0,0],        [0,0],            [0,0],            [0,0],            [0,0]);
    elseif i == 2
        Act_b(i,:) = Act_Plant( Act_b(i-1,:),        [0,0],            [0,0],            [0,0],            [0,0]);
    elseif i == 3
        Act_b(i,:) = Act_Plant( Act_b(i-1,:), Act_b(i-2,:), Act_cmd_b(i-2,:),            [0,0],            [0,0]);
    elseif i == 4
        Act_b(i,:) = Act_Plant( Act_b(i-1,:), Act_b(i-2,:), Act_cmd_b(i-2,:), Act_cmd_b(i-3,:),            [0,0]);
    else
        Act_b(i,:) = Act_Plant( Act_b(i-1,:), Act_b(i-2,:), Act_cmd_b(i-2,:), Act_cmd_b(i-3,:),  Act_cmd_b(i-4,:));
    end
%     
%      Act_b(i,:) = Act_cmd_b(i,:);
%     act_b = Act_b(i,:)

    %% Oil Consuption
    if i == 1
        Oil_cons(i) = Oil_Consumption(Act_b(i,:), [0 0], 0);
    else 
        Oil_cons(i) = Oil_Consumption(Act_b(i,:), Act_b(i-1,:), Oil_cons(i-1));
    end

     %% Finds the coeficients 
    Sound_Velocity = 343 * sqrt(Temperature(i) / 293.1 );
    Velocity_in_Mach(i) = norm(V(i,:)-V_wind(i,:)) / Sound_Velocity;
    
    ii=1;
    while(mach_number(ii) < Velocity_in_Mach(i))
        ii = ii+1;
    end
    
    Cld(i) = Gain_a * data_100hz(ii, 7);
    Clp(i) = Gain_a * data_100hz(ii, 6);
    Cmq(i) = Gain_a * data_100hz(ii, 8);
    Cnr(i) = Gain_a * data_100hz(ii, 9);
    
    Coef = diag([ Cmq(i,:), Cnr(i,:), Clp(i,:)]);
    
%% FORCES

    %% Thrust Force 
    
    % In DLR Body Reference System
    FE_b(i,:) = Thrust_Force_in_DLR_BRS(Gain_b * Fe(i)-Patm(i)*Nzl_S , Act_b(i,:), Nozzle_misalignment);
    
    if i*dt > t_off 
        FE_b(i,:) = 0;
        if aux_flag == 0
            n_iip = i; 
            aux_flag = 1;
        end
    end
    

    % In DLR Navigation Reference System
    FE(i,:) = (D_NB' * FE_b(i,:)')';

    %% Gravitational Force 
    
    % In DLR Navigation Reference System
    FG(i,:) = Gravitational_Force_in_DLR_NRS(latd(i), alt(i), M(i));    
%     FG(i,:) = Test_Gravitational_Force_in_DLR_NRS(latd(i), alt(i), M(i));   

    % In DLR Body Reference System
    FG_b(i,:) = (D_NB * FG(i,:)')';

    %% For�a Aerodin�mica 
    
    % In DLR Navigation Reference System
    FA(i,:) = Aerodynamic_Force_in_DLR_NRS(q(i,:), Gain_a * Cnalfa(i), Gain_a * Cnbeta(i), Gain_a * Cd(i), Pdin(i), AoA(i,:), Sref);

    % In DLR Body Reference System
    FA_b(i,:) = (D_NB * FA(i,:)')';
    
    %% For�a de Coriolis 
    
    % In DLR Navigation Reference System
    FCo(i,:) = Coriolis_Force_in_DLR_NRS(D_NB, M_p(i), W_b(i,:), CoG(i), le);

    % In DLR Body Reference System
    FCo_b(i,:) = (D_NB * FCo(i,:)')';
    
%% MOMENTS

    %% Propulsive Moment 
    
    % In DLR Navigation Reference System 
    MFE(i,:) = Thrust_Moment_in_DLR_NRS(FE(i,:), D_NB, CoG(i), le, Nozzle_eccentricity);

    % In DLR Body Reference System
    MFE_b(i,:) = (D_NB * MFE(i,:)')';
    
    %% Aerodynamic Moment
    
    % In DLR Navigation Reference System
    MFA(i,:) = Aerodynamic_Moment_in_DLR_NRS(FA(i,:), D_NB, CoG(i), CoP(i));

    % In DLR Body Reference System
    MFA_b(i,:) = (D_NB * MFA(i,:)')';
        
    %% Aerodynamic Moment due to Fins Misalignment
    
    % In DLR Navigation Reference System
    MA_f(i,:) = Aerodynamic_Moment_due_to_Fins_Misalignment(Pdin(i), D_NB, Cld(i), dl);

    % In DLR Body Reference System
    MA_f_b(i,:) = (D_NB * MA_f(i,:)')';
    
    %% Aerodynamic Damping Moment
    
    % No triedo de Navega��o do DLR
    MA_d(i,:) =  Aerodynamic_Damping_Moment( D_NB, W_b(i,:), Coef, Pdin(i,:), V(i,:), V_wind(i,:) );

    % No triedo do Corpo do DLR
    MA_d_b(i,:) = (D_NB * MA_d(i,:)')';
    
    %% Coriolis Moment

    % In DLR Navigation Reference System
    MCo(i,:) = Coriolis_Moment_in_DLR_NRS(Ixx_p(i), Iyy_p(i), Izz_p(i), D_NB, M_p(i), W_b(i,:), CoG(i), le);

    % In DLR Body Reference System
    MCo_b(i,:) = (D_NB * MCo(i,:)')';
    
%% ACELERATIONS

    %% Translation Aceleration 
    
    % In DLR Navigation Reference System
    if norm(FE(i,:)) < norm(FG(i,:)) && i*dt < 10 
        acc(i,:) = [0, 0, 0];
    else
        acc(i,:) = ( FG(i,:) + FA(i,:) + FE(i,:) + FCo(i,:) )/ M(i,:);
    end
    
    % In DLR Body Reference System
    acc_b(i,:) = (D_NB * acc(i,:)')'; 

    
    %% Angular Aceleration 
    In = diag([Ixx(i), Iyy(i), Izz(i)]); %BRS

    % In DLR Body Reference System
    ang_acc_b(i,:) = Angular_Aceleration_in_DLR_BRS(In, W_b(i,:), D_NB, 0*MCo(i,:), MFE(i,:), MFA(i,:), MA_f(i,:), MA_d(i,:) );
%    ang_acc_b(i,3) = 0; % OVERWRITING TO TEST >>>>> FIXED ROLL RATE 
%    ang_acc_b(i,:) = [0, 0, 0];

    
    % In DLR Navigation Reference System
    ang_acc(i,:) = (D_NB' * ang_acc_b(i,:)')'; 
   
    %% Cross-coupling acelerations 
    %( aceleration due to -M_extra inside Angular_Aceleration_in_DLR_BRS function)
    
    ax_b(i) = W_b(i,2) * W_b(i,3) * ( Iyy(i) - Izz(i) ) / Ixx(i);
    ay_b(i) = W_b(i,3) * W_b(i,1) * ( Izz(i) - Ixx(i) ) / Iyy(i);
    az_b(i) = W_b(i,1) * W_b(i,2) * ( Ixx(i) - Iyy(i) ) / Izz(i);
    
    dead_time = 1*0.041;
    roll = angles(i,3) + dead_time * W_b(i,3);
    
    ax(i) = ax_b(i) * cos(roll) - ay_b(i) * sin(roll);
    ay(i) = ax_b(i) * sin(roll) + ay_b(i) * cos(roll);
    
    % Inverse transformation:
    roll = angles(i,3);
    TVA_rec(i,1) = - Act_b(i,1) * cos( roll + pi/4) - Act_b(i,2) * sin( roll + pi/4);
    TVA_rec(i,2) = - Act_b(i,1) * cos(-roll + pi/4) + Act_b(i,2) * sin(-roll + pi/4);
    
    
%     a = (D_NB' * [ax_b(i) ay_b(i) az_b(i)]')';
%     if M_beta(i) ~= 0
%         ax(i) = a(1) / M_beta(i);
%         ay(i) = a(2) / M_beta(i);
%         az(i) = a(3) / M_beta(i);
%     else
%         ax(i) = a(1);
%         ay(i) = a(2);
%         az(i) = a(3);
%     end
%     
%% M_ALPHA & M_BETA
    
    %% M_alpha
    M_alpha(i) = Pdin(i,:) * Cnalfa(i) * Sref * (CoP(i) - CoG(i)) * (1 * pi / 180) / Ixx(i);

    %% M_beta
    M_beta(i) = norm(FE(i,:)) * (le - CoG(i)) * (-1 * pi / 180) / Ixx(i);

%% acc_k : Normalized lateral & vertical accelerations (due to 1� of delta in attitude)
    q_aux = angle2quat(pitch_ref(i), yaw_ref(i), roll_ref(i), 'XYZ');
    
%     k_acc(i,:) = lateral_and_vertical_acc(q_aux, acc(i,:), i, dt, FE_b(i,:), FG(i,:), Cnalfa(i), ...
    k_acc(i,:) = lateral_and_vertical_acc(q(i,:), acc(i,:), i, dt, FE_b(i,:), FG(i,:), Gain_a * Cnalfa(i), ...
        Gain_a * Cnbeta(i), Gain_a * Cd(i), Pdin(i), AoA(i,:), Sref, M(i), M_p(i), W_b(i,:), CoG(i), le);
    
    
%% GAINS - @TO DO: Should be removed from this simulation!!
    M_beta_deg = M_beta(i) * 180 / pi;

    PID_deg(i,:) =  1.0/M_beta_deg  * [ 2  1  3 ];           %  <--  VS50 Control System database doc
%     PID_deg(i,:) =  1.0/M_beta_deg  * [ 2  1  3 ];           %  <--  GNC cdr doc
%     PID_deg(i,:) =  1.0/M_beta_deg  * [ 1.821  0.1184  4.613 ];  %  <--  pidTuner
%     PID_deg(i,:) =  5.1/M_beta_deg  * [ 1  0.5  1 ];     <-- old 2019 version that works
    
    for j = 1:3
        if PID_deg(i,j) > 0.3
            PID_deg(i,j) = 0.3;
        end
    end
    

%% INTEGRATIONS

    %% Translation Integration using 4th order Runge-Kutta

%     [XYZ(i+1,:), ~] = Flat_Earth_Translation_Integration(XYZ(i,:), V(i,:), acc(i,:), dt);
%     [position, V(i+1,:)] = Test_of_Translation_Integration(latd(i), lon(i), alt(i), V(i,:), acc(i,:), dt);
    [position, V(i+1,:), XYZ(i,:)] = Translation_Integration(latd(i), lon(i), alt(i), V(i,:), acc(i,:), dt);
    
    latd(i+1) = position(1);
    lon(i+1)  = position(2);
    alt(i+1)  = position(3);


    %% Attitude Integration using Quaternion Cinematics  
    % OVERWRITING TO TEST >>>>  FIXED ROLL RATE 
%      W_b(i,:) = [0, 0, 0] * pi/180;
    
    [W(i+1,:), W_b(i+1,:), q(i+1,:), angles(i+1,:)] = Rotation_Integration( W_b(i,:), ang_acc(i,:), q(i,:), dt );
    
    angles_deg(i+1,:) = angles(i+1,:) * 180/pi;
%     anglesdeg = angles_deg(i+1,:)
%     quat = q(i+1,:)  

%% TERMINATION
    if alt(i+1) <= 0
        break;
    end


%% GUIDANCE CONTROL

    liftoffcounter = i/100;     % the numbers indicate flight time in seconds

    latd_error = latd_ref(i+1) - latd(i+1);
    lon_error = lon_ref(i+1) - lon(i+1);

    % translate latd_error and lon_error to left_right_error and front_back_error
    LAT_error = 6378137                * latd_error;            % [m]
    LON_error = 6378137*cos(latd(i+1)) *  lon_error;            % [m]

    POS = [LON_error, LAT_error, 0];    % [m]  ENU ref.
    VEL = [-V(i+1,2), V(i+1,1), 0 ];    %footprint velocity. Axis direction:(+lon, +latd, up)
    VEL_Z = V(i+1,3);

    crox = cross(VEL/norm(VEL), POS/norm(POS) );
    guin_error = asin(crox(3));                     % [rad] signal standart right-hand side rotation around +z-axis
    % positive values for guin_error means that the rocket is at the
    % current moment to the right side of the trajectory when seen from
    % above, facing future positions

    % negative values for guin_error means that the rocket is at the
    % current moment to the left side of the trajectory when seen from
    % above, facing future positions

    e_1(i+1) = norm(POS) * sin(guin_error);                      % sideways error in meters
    e_2(i+1) = norm(POS) * cos(guin_error) * sign(dot(POS,VEL)) * sign(VEL_Z);% front-back error in meters
    e_3(i+1) = alt_ref(i+1) - alt(i+1);
        
    e_1_int = e_1_int + e_1(i+1) * dt; 
    e_2_int = e_2_int + e_2(i+1) * dt; 
    e_3_int = e_3_int + e_3(i+1) * dt; 

    e_1_dev = ( e_1(i+1) - e_1(i) ) / dt;       %not used
    e_2_dev = ( e_2(i+1) - e_2(i) ) / dt;       %not used
    e_3_dev = ( e_3(i+1) - e_3(i) ) / dt;       %not used
    
    %e_1 left_right_error
    %positive values means the rocket should rotate positively around up-axis
    %to correct its trajectory
    %negative values means the rocket should rotate negatively around up-axis
    %to correct its trajectory

    %e_2 front_back_error
    %Positive values means the rocket should decrease its elevation angle  
    %during an acendent flight (or increase its elevation angle in an decendent
    %flight) to correct its trajectory
    %Negative values means the rocket should increase its elevation angle  
    %during an acendent flight (or decrease its elevation angle in an decendent
    %flight) to correct its trajectory

    %gains in degrees
    GUI_PID(i,1:3) =  1/norm(k_acc(i,1))  * [ 3,  0,  60 ] * 1e-1;
    if(i <= 756)
            GUI_PID(i,1:3) =  [ 28,  0,  540 ] * 1e-1;
    end
    if(i >= 7500)
            GUI_PID(i,1:3) = GUI_PID(7500,1:3);
    end
    
%     GUI_PID(i,1:3) =  1/norm(k_acc(i,1))  * [ 41,  0.46,  582 ] * 1e-3;
%     if(GUI_PID(i,3) > 5.82)
%             GUI_PID(i,1:3) =  [ 0.41,  0.0046,  5.82 ];
%     end
    
%     GUI_PID(i,1:3) =  1/norm(k_acc(i,1))  * [ 10,  1,  200 ] * 1e-3;     %k_acc is almost the same for both axes
%     if(GUI_PID(i,3) > 4)
%         GUI_PID(i,1:3) =  2*[ 0.1,  0.001,  2 ];
%     end

%         Gui_PID(i,1:3) =  15.6/norm(acc_b(i,:))  * [ 0.8e-3,  8.9e-6,  18e-3 ]; 

%     guiP = GUI_PID(i,1);
%     guiI = GUI_PID(i,2);
%     guiD = GUI_PID(i,3);


%         guiP = GUI_PD_tuned(i,1);
%         guiI = GUI_PD_tuned(i,2);
%         guiD = GUI_PD_tuned(i,3);

% %     Tuned PID GUI GAINS
%     guiP = GUI_PID_10ms(i,1);
%     guiI = GUI_PID_10ms(i,2);
%     guiD = GUI_PID_10ms(i,3);
%     guiT = GUI_PID_10ms(i,4);

%     guiP = scheduled_gui_IPD(i,2);
%     guiI = scheduled_gui_IPD(i,1);
%     guiD = scheduled_gui_IPD(i,3);


    %deltas in degrees
%     delta_azi(i) = (guiP * e_1(i+1) + guiI * e_1_int  + guiD * e_1_dev);
%     delta_ele(i) = (guiP * e_2(i+1) + guiI * e_2_int  + guiD * e_2_dev);  
% %     delta_ele(i) = -(guiP * e_3(i+1) + guiI * e_3_int +  e_3_dev); 


    guiP = pid_smooth(i,1);
    guiI = pid_smooth(i,2);
    guiD = pid_smooth(i,3);
    guiT = pid_smooth(i,4);

    if(guiT > 0)
        delta_azi(i) = guiP * e_1(i+1) + (guiD/guiT) * (e_1(i+1) - e_1(i)) - delta_azi(i-1)* (dt - guiT)/guiT;
        delta_ele(i) = guiP * e_2(i+1) + (guiD/guiT) * (e_2(i+1) - e_2(i)) - delta_ele(i-1)* (dt - guiT)/guiT;
    else
        delta_azi(i) = 0;
        delta_ele(i) = 0;
    end
    
    corr_angle_deg = 3;
    if ( liftoffcounter <= 15)               
        corr_angle_deg = 3;          % maximum guidance correction allowed
    elseif (liftoffcounter > 45 && (liftoffcounter <= 50))
        corr_angle_deg = 3;
    elseif (liftoffcounter > 50 && (liftoffcounter <= 55))
        corr_angle_deg = 6;
    elseif (liftoffcounter > 60 && liftoffcounter <= 65)
       corr_angle_deg = 10;
    elseif (liftoffcounter > 65 && liftoffcounter <= 70)
       corr_angle_deg = 5;
    elseif (liftoffcounter > 70 )
       corr_angle_deg = 3;
    end   
    
    if(delta_azi(i) >= corr_angle_deg)
        delta_azi(i) = corr_angle_deg;
    elseif(delta_azi(i) < -corr_angle_deg)
        delta_azi(i) = -corr_angle_deg;
    end

    if(delta_ele(i) >= corr_angle_deg)
        delta_ele(i) = corr_angle_deg;
    elseif(delta_ele(i) < -corr_angle_deg)
        delta_ele(i) = -corr_angle_deg;
    end
     
    if( 1 )%mod(i,10) == 1)
        if((liftoffcounter > 5 && liftoffcounter < 15) || (liftoffcounter > 45 && liftoffcounter < 75))
     
            for ii = 0:0%9
                %---------------------------------------------------------------------- OK
                % This section computes the reference azimuth and reference elevation
                DCM = angle2dcm(pitch_ref(i+1+ii), yaw_ref(i+1+ii), roll_ref(i+1+ii), 'XYZ');
                [p, y, r] = dcm2angle(DCM, 'ZYZ');

                azi_ref = -p;
                ele_ref = pi/2 - y;
                til_ref = r;

                %---------------------------------------------------------------------- OK
                new_azi_ref = azi_ref - delta_azi(i) * pi/180;
                if ele_ref > 0
                    new_ele_ref = ele_ref - delta_ele(i)* pi/180;             
                else
                    new_ele_ref = ele_ref + delta_ele(i)* pi/180;
                end
                DCM_new = angle2dcm( -new_azi_ref, pi/2-new_ele_ref, til_ref, 'ZYZ');

                [p, y, r] = dcm2angle(DCM_new, 'XYZ');
                new_pitch_ref_deg(i+1+ii) = p * 180/pi;
                new_yaw_ref_deg(i+1+ii)   = y * 180/pi;
                new_roll_ref_deg(i+1+ii)  = r * 180/pi;
            end
        else
            for ii = 0:0%9
                new_pitch_ref_deg(i+1+ii) = pitch_ref(i) * 180/pi;
                new_yaw_ref_deg(i+1+ii)   = yaw_ref(i) * 180/pi;
                new_roll_ref_deg(i+1+ii)  = roll_ref(i) * 180/pi;
            end
            e_1_int = 0;
            e_2_int = 0;
        end
    end

%     if((liftoffcounter > 5 && liftoffcounter < 15) || (liftoffcounter > 45 && liftoffcounter < 75))
% 
%         %---------------------------------------------------------------------- OK
%         % This section computes the reference azimuth and reference elevation
%         DCM = angle2dcm(pitch_ref(i+1), yaw_ref(i+1), roll_ref(i+1), 'XYZ');
% 
%         [p, y, r] = dcm2angle(DCM, 'ZYZ');
% 
%         azi_ref = -p;
%         ele_ref = pi/2 - y;
%         til_ref = r;
% 
%         %---------------------------------------------------------------------- OK
% 
%         new_azi_ref = azi_ref - delta_azi(i) * pi/180;
% 
%         if ele_ref > 0
%             new_ele_ref = ele_ref - delta_ele(i)* pi/180;             
%         else
%             new_ele_ref = ele_ref + delta_ele(i)* pi/180;
%         end
% 
%         DCM_new = angle2dcm( -new_azi_ref, pi/2-new_ele_ref, til_ref, 'ZYZ');
% 
%         [p, y, r] = dcm2angle(DCM_new, 'XYZ');
%         new_pitch_ref_deg(i+1) = p * 180/pi;
%         new_yaw_ref_deg(i+1)   = y * 180/pi;
%         new_roll_ref_deg(i+1)  = r * 180/pi;
%      
%     else
%       
%         new_pitch_ref_deg(i+1) = pitch_ref(i) * 180/pi;
%         new_yaw_ref_deg(i+1)   = yaw_ref(i) * 180/pi;
%         new_roll_ref_deg(i+1)  = roll_ref(i) * 180/pi;
%     
%         e_1_int = 0;
%         e_2_int = 0;
%     end
    
    %To turn guidance on/off, comment/uncoment the next 3 lines:
    
    pitch_ref_deg(i+1) = new_pitch_ref_deg(i+1);
    yaw_ref_deg(i+1)   = new_yaw_ref_deg(i+1);
    roll_ref_deg(i+1)  = new_roll_ref_deg(i+1);
     
    
%% ATTITUDE CONTROL - @TO DO: Should be removed from this simulation!!   
    
    % Computed Angles of Attack no Triedo de Navega��o do DLR
    [AoA_comp(i+1,:), Speed_Att_comp(i+1,:)] =  Angle_Of_Attack_in_DLR_NRS( V(i+1,:), [0 0 0], q(i+1,:));
    AoA_comp_deg(i+1,:) = AoA_comp(i+1,:) * 180/pi;
        
    liftoffcounter = i/100;     % the numbers indicate flight time in seconds
 
    if ( liftoffcounter <= 10)               
        corr_angle_deg = 20;                                % maximo AoA permitido
    elseif (liftoffcounter > 10 && (liftoffcounter <= 15))
        corr_angle_deg = 20 - 3 * (liftoffcounter - 10);
    elseif (liftoffcounter > 15 && (liftoffcounter <= 20))
        corr_angle_deg = 5;
    elseif (liftoffcounter > 20 && (liftoffcounter <= 25))
        corr_angle_deg = 5 - (2/5) * (liftoffcounter - 20);
    elseif (liftoffcounter > 25 && (liftoffcounter <= 45))
       corr_angle_deg = 3;                                     
    elseif (liftoffcounter > 45 && (liftoffcounter <= 50))
       corr_angle_deg = 3 + (7/5) * (liftoffcounter - 45); 
    elseif (liftoffcounter > 50 && (liftoffcounter <= 70))  % 70
       corr_angle_deg = 10;    
    elseif (liftoffcounter > 70 && (liftoffcounter <= 75))  % 70 - 75
       corr_angle_deg = 10 - (1/0.5) * (liftoffcounter - 70);     % 10 - 2 * (...-70);
    else
       corr_angle_deg = 0;
    end    
    
    coor(i,1) = corr_angle_deg;
    
    pitch_desired_deg = pitch_ref_deg(i+1);
    Speed_pitch_deg = Speed_Att_comp(i+1,1) * 180/pi;
    
    
    if( abs(AoA_comp_deg(i+1,1)) > corr_angle_deg )
       if( AoA_comp_deg(i+1,1) > 0 )
             pitch_desired_deg = Speed_pitch_deg - corr_angle_deg;
       elseif ( AoA_comp_deg(i+1,1) < 0 )
             pitch_desired_deg = Speed_pitch_deg + corr_angle_deg;
       end
    end


    yaw_desired_deg = yaw_ref_deg(i+1);
    Speed_yaw_deg = Speed_Att_comp(i+1,2) * 180/pi;
    
    if( abs(AoA_comp_deg(i+1,2)) > corr_angle_deg )
       if( AoA_comp_deg(i+1,2) > 0 )
             yaw_desired_deg = Speed_yaw_deg - corr_angle_deg;
       elseif (AoA_comp_deg(i+1,2) < 0 )
             yaw_desired_deg = Speed_yaw_deg + corr_angle_deg;
       end
    end
    
    if (liftoffcounter <= 1)
        pitch_desired_deg = 0;
        yaw_desired_deg = 0;
    end


    
    % pitch control comand
%     pitch_error_old = pitch_error(i);
    pitch_error(i+1) = pitch_desired_deg - angles_deg(i+1,1);    
    pitch_error_int(i+1) = pitch_error_int(i) + ( pitch_error(i) * dt );
    pitch_error_dev(i+1) = (pitch_error(i+1) - pitch_error(i))/dt;
    
    % yaw control comand    
%     yaw_error_old  = yaw_error(i);
    yaw_error(i+1) = yaw_desired_deg - angles_deg(i+1,2);
    yaw_error_int(i+1)  = yaw_error_int(i) + ( yaw_error(i) * dt );
    yaw_error_dev(i+1)  = (yaw_error(i+1) - yaw_error(i))/dt;
    
    % rewrite the attitude reference
    pitch_ref_deg(i+1) = pitch_desired_deg;
    yaw_ref_deg(i+1) = yaw_desired_deg;

% %     Josef Ettl Gains
    P = PID_deg(i,1);
    I = PID_deg(i,2);
    D = PID_deg(i,3);
    
%     % PID tuned gains
%     P = -PID_tuned_10ms(i,1);
%     I = -PID_tuned_10ms(i,2);
%     D = -PID_tuned_10ms(i,3);

% %     segunda tentative PID robusto - HORRIVEL!
%      I = -IPD_robust_10ms(i,1);
%      P = -IPD_robust_10ms(i,2);
%      D = -IPD_robust_10ms(i,3);

    
    % TESTE DO PID ROBUSTO - TESE DE MESTRADO ################################################################ TESE DE MESTRADO
%    I = -scheduled_gains_PID(i,1);
%    P = -scheduled_gains_PID(i,2);
%    D = -scheduled_gains_PID(i,3);

    % TVA_cmd in degrees here
    TVA_cmd(i+1,1) = P * pitch_error(i+1) + I * pitch_error_int(i+1) + D * pitch_error_dev(i+1);
    TVA_cmd(i+1,2) = P *   yaw_error(i+1) + I *   yaw_error_int(i+1) + D *   yaw_error_dev(i+1);

    if (  liftoffcounter < 15 || ( liftoffcounter > 30 && liftoffcounter < 85 )  )
        if (M_beta(i) > 0)
            % for some time try to compensate the impact of malpha as an acceleraton offset
            TVA_cmd(i+1,1) = TVA_cmd(i+1,1) + AoA_comp_deg(i,1) * M_alpha(i)/M_beta(i) + ax(i)/M_beta(i);
            TVA_cmd(i+1,2) = TVA_cmd(i+1,2) + AoA_comp_deg(i,2) * M_alpha(i)/M_beta(i) + ay(i)/M_beta(i);
        else
            TVA_cmd(i+1,2) = 0;
            TVA_cmd(i+1,1) = 0;
        end
    end
    
    if norm(TVA_cmd(i+1,:)) > 3
        TVA_cmd(i+1,:) = 3 * TVA_cmd(i+1,:)/norm(TVA_cmd(i+1,:));
    end
     
    % Low_pass2_10msec(D, w0, &max_x_filt, &pitch_ar, max_x);
    % Low_pass2_10msec(D, w0, &max_y_filt, &yaw_ar, max_y);
    
    TVA_cmd(i+1,:) =  TVA_cmd(i+1,:) * pi/180;        % change back to [rd]

    % OVERWRITING TO TEST fixed nozzle
%       TVA_cmd(i+1,:) = [0 0] * pi/180;
    
%% DMARS PROTOCOL
    
      %No protocolo real, V � passado na seguinte ordem: [Veast, Vnorth, Vup]
      %Aqui, a ordem de V � [Vnorth, Veast, Vup]
    
    DMARS_u = [W_b(i,:), ang_acc_b(i,:), q(i,:), V(i,:), 180/pi*latd(i), 180/pi*lon(i), alt(i)];
    DMARS_0 = [W_b(1,:), ang_acc_b(1,:), q(1,:), V(1,:), 180/pi*latd(1), 180/pi*lon(1), alt(1)];
      
    if i == 1
        DMARS_y = DMARS_plant( DMARS_0  , DMARS_0  , DMARS_0  , DMARS_0  , DMARS_0);
    elseif i == 2
        DMARS_y = DMARS_plant( DMARS_y_1, DMARS_0  , DMARS_u_1, DMARS_0  , DMARS_0);
    elseif i == 3
        DMARS_y = DMARS_plant( DMARS_y_1, DMARS_y_2, DMARS_u_1, DMARS_u_2, DMARS_0);
    else 
        DMARS_y = DMARS_plant( DMARS_y_1, DMARS_y_2, DMARS_u_1, DMARS_u_2, DMARS_u_3);
    end
      
    DMARS_data(i,:) = DMARS_y;
    DMARS_y_1 = DMARS_y;
    DMARS_y_2 = DMARS_y_1;
    DMARS_u_1 = DMARS_u;
    DMARS_u_2 = DMARS_u_1;
    DMARS_u_3 = DMARS_u_2;
    
end

%% Organize
% new_pitch_deg = new_pitch * 180/pi;
% new_yaw_deg = new_yaw * 180/pi;

In        = [Ixx  , Iyy  , Izz  ]; 
I_p       = [Ixx_p, Iyy_p, Izz_p];
% xyz       = [x(:,1), y(:,1), z(:,1)];
Act_cmd_b_deg = Act_cmd_b * 180/pi;
Act_b_deg = Act_b * 180/pi;

lon_deg = 180/pi * lon;
latd_deg = 180/pi * latd;

M_alpha_beta_deg = [M_alpha(:), M_beta(:)] * 180/pi;

W_b_deg = W_b * 180/pi;


Speed_Att(n,:) = Speed_Att(n-1,:);
Speed_Att_deg = Speed_Att * 180/pi;
Speed_Att_comp_deg = Speed_Att_comp * 180/pi;


TVA_cmd_deg = TVA_cmd * 180/pi;
TVA_rec_deg = TVA_rec * 180/pi;

%% Guidance PLOTS

% Errors
figure;
% subplot(2,1,1);
plot(time(1:n),e_1(1:n));
hold;
plot(time(1:n),e_2(1:n));
plot(time(1:n),e_3(1:n));
legend('Sideways error (e_1)','Front-back error (e_2)','Up-down error (e_3)',...
        'Location','best');
title('Position errors');
xlabel('Time [s]')
ylabel('Errors [m]')
frame_h = get(handle(gcf),'JavaFrame');
set(frame_h,'Maximized',1);
grid on;
grid minor;


% subplot(2,1,2);
% plot(time(1:n),latd_deg(1:n)-latd_ref_deg(1:n));
% hold;
% plot(time(1:n),lon_deg(1:n)-lon_ref_deg(1:n));
% plot(time(1:n),alt(1:n)-alt_ref(1:n));
% legend('latd error','lon error','alt error');
% title('Latitude, Longitude & Altitude errors');

%plot(azimuth_error);
% plot(elevation_error);


%% Normalized aceleration
figure;
plot(time(1:n), k_acc(1:n,:));
hold;
legend('up_{acc}', 'lat_{acc}');
title('Normalized Accelerations');
xlabel('Time [s]')
ylabel('Acc [m/s^2]')
frame_h = get(handle(gcf),'JavaFrame');
set(frame_h,'Maximized',1);
grid on;
grid minor;


%% Footprint versus Altitude - SCATTER
figure()
subplot(1,2,1);
scatter(lon(1:n), latd(1:n), 3, 0.001*alt(1:n), 'filled')
hold on
plot(lon(1),latd(1),'rx');
title('Footprint X Altitude');
xlabel('Lon [rad]');
ylabel('Latd [rad]');
cb = colorbar;
cb.Label.String = 'Altitude (km)';
axis('equal');
grid on;
grid minor
xlim([lon(1)-0.0002, lon(n)+0.0002])

% Rough trajectory plane - SCATTER
subplot(1,2,2);
distance = 6378.137*sqrt((lon-lon(1)).^2+(latd-latd(1)).^2);
scatter(distance(1:n), alt(1:n)/1e3, 3, 1:length(alt(1:n)), 'filled');
title('Trajectory plane over time');
xlabel('Distance from launch-pad [km]');
ylabel('Altitude [km]');
% axis('equal');
grid on;
grid minor
frame_h = get(handle(gcf),'JavaFrame');
set(frame_h,'Maximized',1);

%% Footprint versus Altitude - PLOTS
figure();
subplot(1,2,1);
plot(lon_ref_deg(1:n), latd_ref_deg(1:n), 'b', 'LineWidth', 1)
hold on;
plot(lon_deg(1:n), latd_deg(1:n), 'r', 'LineWidth', 1)
plot(lon_deg(1), latd_deg(1),'rx');

%     plot(lon_deg(1:n_iip), latd_deg(1:n_iip), 'r', 'LineWidth', 1)     %impact point
%     plot(lon_deg(n_iip), latd_deg(n_iip), 'kx')
title('Footprint');
xlabel('Lon [�]');
ylabel('Latd [�]');
axis('equal');
grid on;
grid minor
legend('reference','executed','Launchpad', 'Location','East');
xlim([lon_deg(1)-0.01 lon_deg(n)+0.01])

% Rough trajectory plane
% subplot(1,2,2);
% distance = 6378.137*sqrt((lon-lon(1)).^2+(latd-latd(1)).^2);
% distance_ref = 6378.137*sqrt((lon_ref-lon_ref(1)).^2+(latd_ref-latd_ref(1)).^2);
% plot(distance_ref, alt_ref/1e3, 'b', 'LineWidth', 1);
% hold on;
% plot(distance, alt/1e3,'r', 'LineWidth', 1);
% title('Trajectory plane over time');
% xlabel('Distance from launch-pad [km]');
% ylabel('Altitude [km]');
% legend('reference','executed');
% axis('equal');
% grid on;

% Altitude
subplot(1,2,2);
time = data_100hz(:,1);
plot(time(1:n,1), alt_ref(1:n,1)/1e3, 'b', 'LineWidth', 1);
hold on;
plot(time(1:n,1), alt(1:n,1)/1e3,'r', 'LineWidth', 1);
title('Altitude over time');
xlabel('time [s]');
ylabel('Altitude [km]');
legend('reference','executed', 'Location','east');
grid on;
frame_h = get(handle(gcf),'JavaFrame');
set(frame_h,'Maximized',1);
grid minor

%% Latd & Lon 

% figure();
% subplot(2,1,1);
% plot(time(1:n,1),latd_deg(1:n)-latd_deg(1))
% title('Delta Latitude');
% xlabel('Time [s]');
% ylabel('Latd [�]');
% % ylim([-2.32 -2.28])
% grid on;
% 
% subplot(2,1,2);
% plot(time(1:n,1),lon_deg(1:n)-lon_deg(1));
% title('Delta Longitude');
% xlabel('time [s]');
% ylabel('Lon [�]');
% % ylim([-44.37 -44.33])
% grid on;

%% X & Y Terra plana 
% 
% figure();
% subplot(2,1,1);
% plot(time(1:n,1),XYZ(1:n,1))
% title('X');
% xlabel('Time [s]');
% ylabel('X[m]');
% % ylim([-2.32 -2.28])
% grid on;
% 
% subplot(2,1,2);
% plot(time(1:n,1),XYZ(1:n,2));
% title('Y');
% xlabel('time [s]');
% ylabel('Y[m]');
% % ylim([-44.37 -44.33])
% grid on;


%% AoA and TVA_cmd
% figure();
% plot(TVA_cmd_deg(1:3000,1),'b')
% hold;
% % plot(pitch_error(1:3000,1),'r');
% % plot(pitch_error_int(1:3000,1),'g');
% % plot(pitch_error_dev(1:3000,1),'color',[0.9 0.9 0.0]);
% plot( AoA_deg(1:3000,1),'g')
% % plot( AoA_deg(1:3000,1) .* M_alpha_beta_deg(1:3000,1)./M_alpha_beta_deg(1:3000,2),'cyan')
% % legend('TVA-cmd','pitch-error','pitch-error-int','pitch-error-dev'); %'AoAp','AoAp * Ma/Mb',
% legend('TVA-cmd','AoAp');

%% Attitude and others
figure();
pause(0.00001);
frame_h = get(handle(gcf),'JavaFrame');
set(frame_h,'Maximized',1);
subplot(2,1,1);
% plot(time(1:n,1), angles_deg(1:n,1), 'k')
% hold
plot(time(1:n,1), Speed_Att(1:n,1)*180/pi,  'Color',[0.0, 0.85, 1.0]);
hold
plot(time(1:n,1), pitch_ref_deg(1:n,1),'Color',[0.0, 0.0, 0.66])
plot(time(1:n,1), angles_deg(1:n,1), 'k')
legend('Speed-pitch', 'Pitch-Nom','Pitch', 'Location','east')
low = min( [ min(angles_deg(1:n,1)), min(pitch_ref_deg(1:n,1)) ] );
high = max(max(angles_deg(1:n,1)),max(pitch_ref_deg(1:n,1)));
ylim([low-1 high+1]);
ylabel("Pitch-axis [�]")
title("Attitude");
grid minor

subplot(2,1,2);
plot(time(1:n,1), Speed_Att(1:n,2)*180/pi,'Color',[0.1, 0.9, 0.1]);
hold
plot(time(1:n,1), yaw_ref_deg(1:n,1),'Color',[0.0, 0.33, 0.0])
plot(time(1:n,1), angles_deg(1:n,2), 'Color','k')
legend('Speed-yaw', 'Yaw-Nom','Yaw', 'Location','east')
low = min(min(angles_deg(1:n,2)),min(yaw_ref_deg(1:n,1)));
high = max(max(angles_deg(1:n,2)),max(yaw_ref_deg(1:n,1)));
ylim([low-1 high+1]);
ylabel("Yaw-axis [�]")
xlabel("Time (s)")
grid minor

%% Compara��o para averiguar a elimina��o do cross-couple

figure()
frame_h = get(handle(gcf),'JavaFrame');
set(frame_h,'Maximized',1);
plot(TVA_cmd);
hold on;
plot(ax./M_beta);
plot(ay./M_beta);
xlim([3000 6000]);
ylim([-0.02 0.02]);
legend;
plot(TVA_cmd(:,1)+ax./M_beta);
plot(TVA_cmd(:,2)+ay./M_beta);


% 
% figure()
% plot( AoA_deg(:,1), 'b')
% hold
% plot( AoA_deg(:,2), 'r')
% plot( AoA_comp_deg(:,1),'Color',[0.0, 0.0, 0.66])
% plot( AoA_comp_deg(:,2),'Color',[0.66, 0.0, 0.0])
% legend('AoA-pitch','AoA-yaw','AoA-pitch-comp','AoA-yaw-comp')
% title('Angles of Attack');

% figure()
% plot(W_b_deg,'DisplayName','W_b_deg')
% title('Angular rates in Body Reference System');
% legend('pitch-rate','yaw-rate','roll-rate');
% xlabel('Time [0.01s]')
% ylabel('Rate [�/s]')

% figure()
% plot(MFA)
% title('Aerodynamic Moment due to AoA in Nav. Ref. Sys.');
% legend('pitch-moment','yaw-moment','roll-moment');
% xlabel('Time [0.01s]')
% ylabel('Moment [N.m]')
% 
% figure()
% plot(MFA_b)
% title('Aerodynamic Moment due to AoA in Body Ref. Sys.');
% legend('pitch-moment','yaw-moment','roll-moment');
% xlabel('Time [0.01s]')
% ylabel('Moment [N.m]')
% 
% figure()
% plot(MA_f)
% title('Aerodynamic Moment due to Fins Misalignment in Nav. Ref. Sys.');
% legend('pitch-moment','yaw-moment','roll-moment');
% xlabel('Time [0.01s]')
% ylabel('Moment [N.m]')
% 
% figure()
% plot(MA_f_b)
% title('Aerodynamic Moment due to Fins Misalignment in Body Ref. Sys.');
% legend('pitch-moment','yaw-moment','roll-moment');
% xlabel('Time [0.01s]')
% ylabel('Moment [N.m]')

% figure()
% plot(MA_d)
% title('Aerodynamic Damping Moment in Nav. Ref. Sys.');
% legend('pitch-moment','yaw-moment','roll-moment');
% xlabel('Time [0.01s]')
% ylabel('Moment [N.m]')

% figure()
% plot(MA_d_b)
% title('Aerodynamic Damping Moment in Body Ref. Sys.');
% legend('pitch-moment','yaw-moment','roll-moment');
% xlabel('Time [0.01s]')
% ylabel('Moment [N.m]')


% figure()
% plot(Cld)
% hold
% plot(Clp)
% title('Aerodynamic Roll Moment Coeficients');
% legend('driving-coeficient','damping-coeficient');
% xlabel('Time [0.01s]')
% ylabel('Coeficients [1/rad]')


% figure()
% plot(angles_deg(1:N,1)-pitch_ref_deg(1:N), 'Color',[0.0, 0.0, 1.0])
% hold
% plot(angles_deg(1:N,2) - yaw_ref_deg(1:N), 'Color',[0.0, 0.75, 0.0])
% plot(Speed_pitch(1:N)*180/pi - pitch_ref_deg(1:N),  'Color',[0.0, 0.85, 1.0]);
% plot(Speed_yaw(1:N)*180/pi - yaw_ref_deg(1:N),'Color',[0.0, 1.0, 0.0]);
% legend('pitch - Pitch-ref','yaw - Yaw-ref','Speed-pitch - Pitch-ref','Speed-yaw - Yaw-ref')


% figure
% plot(Act_b(1:8000,:),'DisplayName','Act_b')
% legend()
% figure
% plot(TVA_cmd(1:8000,:),'DisplayName','TVA_cmd')
% legend()

% figure()
% plot( MA_f_b(:,3) + MA_d_b(:,3))
% hold on
% plot(MCo_b(:,3) )
% plot( MA_f_b(:,3) + MA_d_b(:,3) + MCo_b(:,3) )
% legend('Driving Moment + Damping Moment', 'Coriolis Moment', 'Driving Moment + Damping Moment + Coriolis Moment')
% xlabel('Time [0.01s]')
% ylabel('Moment [N.m]')
% title('Roll Moments')

%% Malfa versus Mbeta over time
% set(0,'defaultAxesFontSize',20);
% set(0,'defaultFigurePosition', [10 200 800 800])
% figure()
% scatter(M_alpha_beta_deg(1:n,1), M_alpha_beta_deg(1:n,2), 3, time(1:n), 'filled')
% title('M_{\alpha} versus M_{\beta} over time');
% xlabel('M_{\alpha} [�/s^2]');
% ylabel('M_{\beta}[�/s^2]');
% c = colorbar;
% c.Label.String = 'time (s)';
% % axis('equal');
% grid on;

 %% variable Malfa versus variable Mbeta over time
 
% M_alpha_deg_plus  = 1.1 * M_alpha_beta_deg(1:n,1);
% M_alpha_deg_minus = 0.9 * M_alpha_beta_deg(1:n,1);
% 
% M_beta_deg_plus  = 1.2 * M_alpha_beta_deg(1:n,2);
% M_beta_deg_minus = 0.8 * M_alpha_beta_deg(1:n,2);
% 
% 
% set(0,'defaultAxesFontSize',20);
% set(0,'defaultFigurePosition', [10 200 800 800])
% figure()
% 
% scatter(M_alpha_beta_deg(1:n,1), M_alpha_beta_deg(1:n,2), 'filled')
% hold on
% scatter(M_alpha_deg_plus, M_beta_deg_plus, 'filled');
% scatter(M_alpha_deg_minus, M_beta_deg_plus, 'filled');
% scatter(M_alpha_deg_plus,  M_beta_deg_minus, 'filled');
% scatter(M_alpha_deg_minus, M_beta_deg_minus, 'filled');
% 
% title('M_{\alpha} versus M_{\beta} over time');
% xlabel('M_{\alpha} [�/s^2]');
% ylabel('M_{\beta}[�/s^2]');
% 
% % axis('equal');
% 
% legend('1,0.M_{\alpha} x 1,0.M_{\beta}', ...
%        '1,1.M_{\alpha} x 1,2.M_{\beta}', ...
%        '0,9.M_{\alpha} x 1,2.M_{\beta}', ...
%        '1,1.M_{\alpha} x 0,8.M_{\beta}', ...
%        '0,9.M_{\alpha} x 0,8.M_{\beta}')
% grid on;
% 
% 
% %% variable Malfa versus variable Mbeta over time with polytope
% 
% M_alpha_deg_plus  = 1.1 * M_alpha_beta_deg(1:n,1);
% M_alpha_deg_minus = 0.9 * M_alpha_beta_deg(1:n,1);
% 
% M_beta_deg_plus  = 1.2 * M_alpha_beta_deg(1:n,2);
% M_beta_deg_minus = 0.8 * M_alpha_beta_deg(1:n,2);
% 
% 
% set(0,'defaultAxesFontSize',20);
% set(0,'defaultFigurePosition', [10 200 800 800])
% figure()
% 
% scatter(M_alpha_beta_deg(1:n,1), M_alpha_beta_deg(1:n,2), 'filled')
% hold on
% scatter(M_alpha_deg_plus, M_beta_deg_plus, 'filled');
% scatter(M_alpha_deg_minus, M_beta_deg_plus, 'filled');
% scatter(M_alpha_deg_plus,  M_beta_deg_minus, 'filled');
% scatter(M_alpha_deg_minus, M_beta_deg_minus, 'filled');
% 
% fill([0.5,  5.5,  5.5,  3.5,   1, -19, -19], ...
%      [-1, 20, 34, 39, 40,  23,  14.5], 'blue','FaceAlpha',0.1);
% 
% 
% 
% title('M_{\alpha} versus M_{\beta} over time');
% xlabel('M_{\alpha} [�/s^2]');
% ylabel('M_{\beta}[�/s^2]');
% 
% % axis('equal');
% 
% % legend('1,0.M_{\alpha} x 1,0.M_{\beta}', ...
% %        '1,1.M_{\alpha} x 1,2.M_{\beta}', ...
% %        '0,9.M_{\alpha} x 1,2.M_{\beta}', ...
% %        '1,1.M_{\alpha} x 0,8.M_{\beta}', ...
% %        '0,9.M_{\alpha} x 0,8.M_{\beta}', ...
% %        'Location', 'northwest')
% grid on;
% 
% %% variable Malfa versus variable Mbeta over time with polytope
% 
% M_alpha_deg_plus  = 1.1 * M_alpha_beta_deg(1:n,1);
% M_alpha_deg_minus = 0.9 * M_alpha_beta_deg(1:n,1);
% 
% M_beta_deg_plus  = 1.2 * M_alpha_beta_deg(1:n,2);
% M_beta_deg_minus = 0.8 * M_alpha_beta_deg(1:n,2);
% 
% 
% set(0,'defaultAxesFontSize',20);
% set(0,'defaultFigurePosition', [10 200 800 800])
% figure();
% 
% 
% subplot(2,2,1);
% scatter(M_alpha_beta_deg(1:n,1), M_alpha_beta_deg(1:n,2), 'filled')
% hold on
% scatter(M_alpha_deg_plus, M_beta_deg_plus, 'filled');
% scatter(M_alpha_deg_minus, M_beta_deg_plus, 'filled');
% scatter(M_alpha_deg_plus,  M_beta_deg_minus, 'filled');
% scatter(M_alpha_deg_minus, M_beta_deg_minus, 'filled');
% % % Vertical
% fill([-0.5,  0.5, 0.5, -0.5], ...
%      [-0.5, -0.5, 24,  24], 'blue','FaceAlpha',0.2);
% 
% title({'                                                      M_{\alpha} versus M_{\beta} over time',' '});
%  
% subplot(2,2,2);
% scatter(M_alpha_beta_deg(1:n,1), M_alpha_beta_deg(1:n,2), 'filled')
% hold on
% scatter(M_alpha_deg_plus, M_beta_deg_plus, 'filled');
% scatter(M_alpha_deg_minus, M_beta_deg_plus, 'filled');
% scatter(M_alpha_deg_plus,  M_beta_deg_minus, 'filled');
% scatter(M_alpha_deg_minus, M_beta_deg_minus, 'filled');
% % % Horizontal
% fill([ 0.5, 0.5, -19, -19], ...
%      [ 14, 23.5, 23.5,  14.5], 'blue','FaceAlpha',0.2);
%  
% subplot(2,2,3);
% scatter(M_alpha_beta_deg(1:n,1), M_alpha_beta_deg(1:n,2), 'filled')
% hold on
% scatter(M_alpha_deg_plus, M_beta_deg_plus, 'filled');
% scatter(M_alpha_deg_minus, M_beta_deg_plus, 'filled');
% scatter(M_alpha_deg_plus,  M_beta_deg_minus, 'filled');
% scatter(M_alpha_deg_minus, M_beta_deg_minus, 'filled');
% % % Diagonal
% fill([ 0.5, 0.5, -19, -19], ...
%      [ 18, 29, 23.5,  14], 'blue','FaceAlpha',0.2);
% 
% xlabel('                                               M_{\alpha} [�/s^2]');
% ylabel('                                               M_{\beta} [�/s^2]');
% 
% subplot(2,2,4);
% scatter(M_alpha_beta_deg(1:n,1), M_alpha_beta_deg(1:n,2), 'filled')
% hold on
% scatter(M_alpha_deg_plus, M_beta_deg_plus, 'filled');
% scatter(M_alpha_deg_minus, M_beta_deg_plus, 'filled');
% scatter(M_alpha_deg_plus,  M_beta_deg_minus, 'filled');
% scatter(M_alpha_deg_minus, M_beta_deg_minus, 'filled');
% % % Loop
% fill([-0.5, 5.5,  5.5,  0], ...
%      [ 17.5,   20, 38, 40], 'blue','FaceAlpha',0.2);
% 
% % scatter(M_alpha_beta_deg(30:2100,1), M_alpha_beta_deg(30:2100,2), 'filled', 'k');
% % scatter(M_alpha_deg_plus(30:2100) , M_beta_deg_plus(30:2100), 'filled', 'k');
% % scatter(M_alpha_deg_minus(30:2100), M_beta_deg_plus(30:2100), 'filled', 'k');
% % scatter(M_alpha_deg_plus(30:2100) , M_beta_deg_minus(30:2100), 'filled', 'k');
% % scatter(M_alpha_deg_minus(30:2100), M_beta_deg_minus(30:2100), 'filled', 'k');
% 
% 
% 
% 
% % axis('equal');
% 
% % legend('1,0.M_{\alpha} x 1,0.M_{\beta}', ...
% %        '1,1.M_{\alpha} x 1,2.M_{\beta}', ...
% %        '0,9.M_{\alpha} x 1,2.M_{\beta}', ...
% %        '1,1.M_{\alpha} x 0,8.M_{\beta}', ...
% %        '0,9.M_{\alpha} x 0,8.M_{\beta}', ...
% %        'Location', 'northwest')
% grid on;
% 

%% Dados para a ACE-V

% n = 8200;
% parametros = zeros(n,23);
% 
% parametros(:, 1) = time(1:n,1);
% parametros(:, 2) = pitch_ref_deg(1:n,1);
% parametros(:, 3) = yaw_ref_deg(1:n,1);
% parametros(:, 4) = angles_deg(1:n,1);
% parametros(:, 5) = angles_deg(1:n,2);
% parametros(:, 6) = angles_deg(1:n,3);
% parametros(:, 7) = M_alpha_beta_deg(1:n,1);
% parametros(:, 8) = M_alpha_beta_deg(1:n,2);
% parametros(:, 9) = PID_deg(1:n,1);
% parametros(:,10) = PID_deg(1:n,2);
% parametros(:,11) = PID_deg(1:n,3);
% parametros(:,12) = pitch_error(1:n,1);
% parametros(:,13) = yaw_error(1:n,1);
% parametros(:,14) = pitch_error_int(1:n,1);
% parametros(:,15) = yaw_error_int(1:n,1);
% parametros(:,16) = pitch_error_dev(1:n,1);
% parametros(:,17) = yaw_error_dev(1:n,1);
% parametros(:,18) = TVA_cmd_deg(1:n,1);
% parametros(:,19) = TVA_cmd_deg(1:n,2);
% parametros(:,20) = Act_cmd_b_deg(1:n,1);
% parametros(:,21) = Act_cmd_b_deg(1:n,2);
% parametros(:,22) = TVA_rec_deg(1:n,1);
% parametros(:,23) = TVA_rec_deg(1:n,2);
% 
% csvwrite('parametros_31_05_2021.csv',parametros);


%% Clean up 
% clear('Coef'); clear('x');     clear('y');   clear('z');     clear('ans');   clear('I_times_ang_acc');
% clear('Ixx');  clear('Iyy');   clear('Izz'); clear('Ixx_p'); clear('Iyy_p'); clear('Izz_p');            clear('angles');
% clear('D_NB'); clear('D_l');   clear('L_C'); clear('S');     clear('data');  clear('AoA_pitch');        clear('AoA_yaw');
% clear('Nz_d'); clear('Nzl_S'); clear('R_e'); clear('R_l');   clear('R_lh');  clear('data_size');        clear('Exp_Omega_k_T');
% clear('dt');   clear('f');     clear('fx');  clear('fy');    clear('fz');    clear('Fe_traj');          clear('liftoffcounter');
% clear('i');    clear('le');    clear('T');   clear('pitch'); clear('yaw');   clear('roll');             clear('position');
% clear('q0');   clear('q1');    clear('q2');  clear('q3');    clear('w_b');   clear('alt_traj');         clear('M_alpha');       
% clear('k1');   clear('k2');    clear('k3');  clear('k4');    clear('dl');    clear('M_beta');           clear('data_100hz');
% clear('Dref'); clear('Sref');  clear('Mach');clear('P');     clear('D');     clear('M_beta_deg');       clear('n');     
% clear('ii');   clear('N');     clear('j');   clear('distance_ref');   clear('corr_angle_deg');
% clear('');  clear('');
% 
% clear('DMARS_0');     clear('Sound_Velocity');      clear('distance');
% clear('DMARS_u');     clear('Nozzle_eccentricity'); clear('Nozzle_misalignment');   
% clear('DMARS_u_1');   clear('pitch_error');         clear('yaw_error');            
% clear('DMARS_u_2');   clear('Speed_pitch_deg');     clear('Speed_yaw_deg');
% clear('DMARS_u_3');   clear('pitch_error_dev(i+1)');clear('yaw_error_dev(i+1)');
% clear('DMARS_y');     clear('pitch_error_int');     clear('yaw_error_int');
% clear('DMARS_y_1');   clear('pitch_error_old');     clear('yaw_error_old');
% clear('DMARS_y_2');   clear('pitch_desired_deg');   clear('yaw_desired_deg'); 
% clear('XYZ');