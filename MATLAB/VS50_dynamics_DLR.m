% function [lon, latd, alt] = VS50_dynamics_DLR()
clc;
clear all;
close all;
set(0,'DefaultAxesXGrid','on')
set(0,'DefaultAxesYGrid','on')
set(0,'DefaultLineLineWidth', 2);
set(0,'defaultFigurePosition', [10 200 1600 800])
colordef white

%% Dados do veículo

load('2018.12.04_VS50_alcantara.mat');
% load('vento_1.mat');
% load('vento_2.mat');
% load('vento_4.mat');

% Constant values

n = length(data_100hz);          % number of iterations
le    = -10.9332;			 	 % Position of the 'Nozzle Throat' in Longitudinal Axis     [m]
Nz_d  = 0.820;                   % Nozzle exit diameter                                     [m]
Nzl_S = pi*(Nz_d)^2;             % Nozzle exit area                                         [m^2]
dt = 0.01;                       % Incremento de tempo utilizado                            [s]
Dref = 1.72;                     % Vehicle reference diameter                               [m]
Sref = pi * (Dref/2)^2;          % Vehicle reference area                                   [m^2]


% Disturbances

Nozzle_misalignment = pi/180 * [ 0.0  0 ];   % Desalinhamento da tubeira [X-pitch, Y-yaw]        [rad]  
Nozzle_eccentricity = 1e-3 * [ 0  0 ];        % Ecentricidade da tubeira [X, Y]                   [m]  
dl = 0.05  * pi/180;                          % Fins Misalignment                                 [rad]

% Control data

pitch_error_old = 0;
pitch_error_int = 0;
yaw_error_old   = 0;
yaw_error_int   = 0;



%% Leitura dos dados invariantes de voo

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
latd_ref_deg= data_100hz(:,43);          % Trajectory geodetic latitude reference           [º]
lon_ref_deg = data_100hz(:,41);          % Trajectory longitude reference                   [º]
alt_ref = data_100hz(:,38) * 1e3;        % Trajectory altitude reference                    [m]
mach_number   = data_100hz(:,39);        % Velocity in Mach number for nominal trajectory   [-] 
pitch_ref_deg = data_100hz(:,52);        % DLR Pitch reference                              [º]
yaw_ref_deg   = data_100hz(:,53);        % DLR Yaw reference                                [º]
roll_ref_deg  = data_100hz(:,54);        % DLR Roll reference                               [º]

latd_ref = latd_ref_deg * pi/180;        % Trajectory geodetic latitude reference           [rad]
lon_ref  =  lon_ref_deg * pi/180;        % Trajectory longitude reference                   [rad]

%% Dados variantes de voo (LOGDATA)

TVA_cmd     = zeros(n,2);               % TVA nozzle pitch and yaw angles commands in DLR NRS. [rad]    
Act_cmd_b   = zeros(n,2);               % Nozzle angle command for actuators at 315º and  225º [rad]
Act_b       = zeros(n,2);               % Nozzle angle for actuators at 315º and at 225º       [rad]
latd        = zeros(n,1);               % Geodetic Latitude of the Vehicle during flight       [rad]
alt         = zeros(n,1);               % Geodetic Altitude of the Vehicle during flight       [rad]
lon         = zeros(n,1);               % Geodetic Longitude of the Vehicle during flight      [rad]
q           = zeros(n,4);               % Quaternion of atitude                                [-]
angles      = zeros(n,3);               % Euler angles in DLR Standard: 1-2-3                  [rad]
angles_deg  = zeros(n,3);               % Euler angles in DLR Standard: 1-2-3                  [º]
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
AoA_deg     = zeros(n,2);               % Angle of Attack [XZ plane, YZ plane] of DLR_NRS      [º]
AoA_comp    = zeros(n,2);               % Computed AoA [XZ plane, YZ plane] of DLR_NRS         [rad]
AoA_comp_deg= zeros(n,2);               % Computed AoA [XZ plane, YZ plane] of DLR_NRS         [º]
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
M_alpha     = zeros(n,1);
M_beta      = zeros(n,1);
PID_deg     = zeros(n,3);               % PID gains computed as a function of M_beta in degrees
Temperature = zeros(n,1);               % Temperature [K]
pitch_error = zeros(n,1);
yaw_error   = zeros(n,1);
latd_error  = zeros(n,1);
lon_error   = zeros(n,1);
Oil_cons    = zeros(n,1);               %               [L]
DMARS_data  = zeros(n,16);              % DMARS data sent in the protocol (output of the DMARS_plant)
                                        % [W_b(i,:), ang_acc_b(i,:), q(i,:), V(i,:), latd(i), lon(i), alt(i)];
                               

Velocity_in_Mach = zeros(n,1);


% Initial Position --------------------------------------------------------

XYZ   = zeros(n,3);

alt(1,1) = 50.0;                   % Initial Altitude                 [m]
latd(1,1) = -2.315995 * pi/180;    % Initial Geodetic Latitud         [rad]
lon(1,1) = -44.367775 * pi/180;    % Initial Longitude                [rad]

% Initial Atitude ---------------------------------------------------------

angles_deg(1,3)  = 0;
angles(1,3)      = angles_deg(1,3) * pi/180;  
q(1,:) = [cos(angles(1,3)/2) 0 0 sin(angles(1,3)/2)];  

% Wind conditions ---------------------------------------------------------

V_wind      = zeros(n,3);           % Wind Velocity vector in DLR NRS [m/s]
% V_wind(1:n,1:2) = vento_1(1:n,2:3); % Wind from files
% V_wind    = [zeros(n,1) [zeros(1000,1);  10*ones(n-1000,1)]  zeros(n,1)]; 


%%
for i = 1:(n-1)
    %% Pressão Dinâmica e Pressão Atmosférica local
    [Pdin(i), Patm(i), Temperature(i)] = Dynamic_Pressure( alt(i), V(i,:), V_wind(i,:) );
    
    %% Direct Cossine Matrix
    D_NB = DCM_NRS_to_BRS( q(i,:) );
    
    %% Real Angles of Attack no Triedo de Navegação do DLR
    [AoA(i,:), Speed_Att(i,:)] =  Angle_Of_Attack_in_DLR_NRS( V(i,:), V_wind(i,:), q(i,:));
    AoA_deg(i,:) = AoA(i,:) * 180/pi;
    
    
    
    %% Command Conversion from TVA Comand in DLR NRS To Actuator Command
    if i == 1
        Act_cmd_b(i,:) = Command_Conversion_TVA_To_ACT(TVA_cmd(i,:),         [0, 0], angles(i,3),        0);
    else
        Act_cmd_b(i,:) = Command_Conversion_TVA_To_ACT(TVA_cmd(i,:), TVA_cmd(i-1,:), angles(i,3), W_b(i,3));
    end
    
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
    
    Cld(i) = data_100hz(ii, 7);
    Clp(i) = data_100hz(ii, 6);
    Cmq(i) = data_100hz(ii, 8);
    Cnr(i) = data_100hz(ii, 9);
    
    Coef = diag([ Cmq(i,:), Cnr(i,:), Clp(i,:)]);
    
%% FORCES

    %% Thrust Force 
    
    % In DLR Body Reference System
    FE_b(i,:) = Thrust_Force_in_DLR_BRS(Fe(i)-Patm(i)*Nzl_S , Act_b(i,:), Nozzle_misalignment);
    
    % In DLR Navigation Reference System
    FE(i,:) = (D_NB' * FE_b(i,:)')';

    %% Gravitational Force 
    
    % In DLR Navigation Reference System
    FG(i,:) = Gravitational_Force_in_DLR_NRS(latd(i), alt(i), M(i));        
   
    % In DLR Body Reference System
    FG_b(i,:) = (D_NB * FG(i,:)')';

    %% Força Aerodinâmica 
    
    % In DLR Navigation Reference System
    FA(i,:) = Aerodynamic_Force_in_DLR_NRS(q(i,:), Cnalfa(i), Cnbeta(i), Cd(i), Pdin(i), AoA(i,:), Sref);
   
    % In DLR Body Reference System
    FA_b(i,:) = (D_NB * FA(i,:)')';
    
    %% Força de Coriolis 
    
    % In DLR Navigation Reference System
    FCo(i,:) = Coriolis_Force_in_DLR_NRS(D_NB, M_p(i), W_b(i,:), CoG(i), le);
   
    % In DLR Body Reference System
    FCo_b(i,:) = (D_NB * FCo(i,:)')';
    
%% MOMENTS

    %% Momento Propulsivo 
    
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
    
    % No triedo de Navegação do DLR
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
    I = diag([Ixx(i), Iyy(i), Izz(i)]); %BRS

    % In DLR Body Reference System
    ang_acc_b(i,:) = Angular_Aceleration_in_DLR_BRS(I, W_b(i,:), D_NB, MCo(i,:), MFE(i,:), MFA(i,:), MA_f(i,:), MA_d(i,:) );
    %ang_acc_b(i,3) = 0; % OVERWRITING TO TEST >>>>> FIXED ROLL RATE 

    % In DLR Navigation Reference System
    ang_acc(i,:) = (D_NB' * ang_acc_b(i,:)')'; 
   
 
%% M_ALPHA & M_BETA
    
    %% M_alpha
    M_alpha(i) = Pdin(i,:) * Cnalfa(i) * Sref * (CoP(i) - CoG(i)) * (1 * pi / 180) / Ixx(i);
    
    %% M_beta
    M_beta(i) = norm(FE(i,:)) * (le - CoG(i)) * (-1 * pi / 180) / Ixx(i);
    
%% GAINS - @TO DO: Should be removed from this simulation!!
    M_beta_deg = M_beta(i) * 180 / pi;

    PID_deg(i,:) =  5.1/M_beta_deg  * [ 1  0.5  1 ]; 
    
    for j = 1:3
        if PID_deg(i,j) > 0.3
            PID_deg(i,j) = 0.3;
        end
    end
    

%% INTEGRATIONS

    %% Translation Integration using 4th order Runge-Kutta

    [position, V(i+1,:), XYZ(i,:)] = Translation_Integration(latd(i), lon(i), alt(i), V(i,:), acc(i,:), dt);
    
    latd(i+1) = position(1);
    lon(i+1)  = position(2);
    alt(i+1)  = position(3);
    

    %% Attitude Integration using Quaternion Cinematics  
    % OVERWRITING TO TEST >>>>  FIXED ROLL RATE 
%     W_b(i,3) = 0 * pi/180;
    
    [W(i+1,:), W_b(i+1,:), q(i+1,:), angles(i+1,:)] = Rotation_Integration(W(i,:), W_b(i,:), ang_acc(i,:), D_NB, q(i,:), dt );
    
    angles_deg(i+1,:) = angles(i+1,:) * 180/pi;
    
%% GUIDANCE CONTROL

%     liftoffcounter = i/100;     % the numbers indicate flight time in seconds
    
    % latd control comand
%     latd_error_old = latd_error(i);
    latd_error(i+1)= latd_ref(i+1) - latd(i+1)    ;
%     latd_error_int = latd_error_int + latd_error(i+1) * dt;
%     latd_error_dev = (latd_error(i+1) - latd_error_old)/dt;
    
    % lon control comand
%     lon_error_old = lon_error(i);
    lon_error(i+1)= lon_ref(i+1) - lon(i+1);    
%     lon_error_int = lon_error_int + lon_error(i+1) * dt;
%     lon_error_dev = (lon_error(i+1) - lon_error_old)/dt;
    
    % translate latd_error and lon_error to left_right_error and front_back_error
    LAT_error = 6378137                * latd_error(i+1);            % [m]
    LON_error = 6378137*cos(latd(i+1)) *  lon_error(i+1);            % [m]
    
    POS = [LON_error, LAT_error, 0];
    VEL = [-V(i+1,2), V(i+1,1), 0 ];
    
    crox = cross(VEL/norm(VEL), POS/norm(POS) );    
    guin_error = asin(crox(3));                     % [rad]
    
    
    e_2(i) = norm(POS) * cos(guin_error) * sign(dot(POS,VEL));
    
%     % e_1 control comand
%     e_1_old = e_1(i-1);
    e_1(i) = norm(POS) * sin(guin_error);
%     e_1_int = e_1_int + e_1(i) * dt;
%     pitch_error_dev = (pitch_error(i+1) - pitch_error_old)/dt;
%     
%     % e_2 control comand    
%     yaw_error_old  = yaw_error(i);
%     yaw_error(i+1) = yaw_desired_deg - angles_deg(i+1,2);
%     yaw_error_int  = yaw_error_int + yaw_error(i+1) * dt;
%     yaw_error_dev  = (yaw_error(i+1) - yaw_error_old)/dt;
%     
%     P = PID_deg(i,1);
%     I = PID_deg(i,2);
%     D = PID_deg(i,3);
%     
%     % TVA_cmd in degrees here
%     TVA_cmd(i+1,1) = P * pitch_error(i+1) + I * pitch_error_int + D * pitch_error_dev;
%     TVA_cmd(i+1,2) = P *   yaw_error(i+1) + I *   yaw_error_int + D *   yaw_error_dev;
    
    
%% ATTITUDE CONTROL - @TO DO: Should be removed from this simulation!!   

    % Computed Angles of Attack no Triedo de Navegação do DLR
    [AoA_comp(i+1,:), Speed_Att_comp(i+1,:)] =  Angle_Of_Attack_in_DLR_NRS( V(i+1,:), [0 0 0], q(i+1,:));
    AoA_comp_deg(i+1,:) = AoA_comp(i+1,:) * 180/pi;
        

    liftoffcounter = i/100;     % the numbers indicate flight time in seconds
    
    if ( liftoffcounter <= 15)               
        corr_angle_deg = 20;                                % maximo AoA permitido
    elseif (liftoffcounter > 15 && (liftoffcounter <= 20))
        corr_angle_deg = 20;
    elseif (liftoffcounter > 20 && (liftoffcounter <= 25))
        corr_angle_deg = 10;
    elseif (liftoffcounter > 25 && (liftoffcounter <= 45))
       corr_angle_deg = 0;                                           
    elseif (liftoffcounter > 45 && (liftoffcounter <= 75))
       corr_angle_deg = 10;         
    elseif (liftoffcounter > 75 && (liftoffcounter <= 85))
       corr_angle_deg = 0;
    else
       corr_angle_deg = 0;
    end    
    
    
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
    pitch_error_old = pitch_error(i);
    pitch_error(i+1) = pitch_desired_deg - angles_deg(i+1,1);    
    pitch_error_int = pitch_error_int + pitch_error(i+1) * dt;
    pitch_error_dev = (pitch_error(i+1) - pitch_error_old)/dt;
    
    % yaw control comand    
    yaw_error_old  = yaw_error(i);
    yaw_error(i+1) = yaw_desired_deg - angles_deg(i+1,2);
    yaw_error_int  = yaw_error_int + yaw_error(i+1) * dt;
    yaw_error_dev  = (yaw_error(i+1) - yaw_error_old)/dt;
    
    P = PID_deg(i,1);
    I = PID_deg(i,2);
    D = PID_deg(i,3);
    
    % TVA_cmd in degrees here
    TVA_cmd(i+1,1) = P * pitch_error(i+1) + I * pitch_error_int + D * pitch_error_dev;
    TVA_cmd(i+1,2) = P *   yaw_error(i+1) + I *   yaw_error_int + D *   yaw_error_dev;

    if ( liftoffcounter < 15 || ( liftoffcounter > 30 && liftoffcounter < 85 )  )
        if (M_beta(i) > 0)
            % for some time try to compensate the impact of malpha as an acceleraton offset
            TVA_cmd(i+1,1) = TVA_cmd(i+1,1) + AoA_comp_deg(i,1) * M_alpha(i)/M_beta(i);
            TVA_cmd(i+1,2) = TVA_cmd(i+1,2) + AoA_comp_deg(i,2) * M_alpha(i)/M_beta(i);
        else
            TVA_cmd(i+1,2) = 0;
            TVA_cmd(i+1,1) = 0;
        end
    end
  
    if norm(TVA_cmd(i+1,:)) > 3
        TVA_cmd(i+1,:) = 3 * TVA_cmd(i+1,:)/norm(TVA_cmd(i+1,:));
    end
    
    TVA_cmd(i+1,:) =  TVA_cmd(i+1,:) * pi/180;        % change back to [rd]


    % OVERWRITING TO TEST fixed nozzle
%     TVA_cmd(i+1,:) = [0.1 0] * pi/180;

    
    
%% DMARS PROTOCOL
    
      %No protocolo real, V é passado na seguinte ordem: [Veast, Vnorth, Vup]
      %Aqui, a ordem de V é [Vnorth, Veast, Vup]
    
    DMARS_u = [W_b(i,:), ang_acc_b(i,:), q(i,:), V(i,:), 180/pi*latd(i), 180/pi*lon(i), alt(i)];
    DMARS_0 = [W_b(1,:), ang_acc_b(1,:), q(1,:), V(1,:), 180/pi*latd(1), 180/pi*lon(1), alt(1)];
      
    if i == 1
        DMARS_y = DMARS_plant(   DMARS_0,   DMARS_0,   DMARS_0,   DMARS_0,   DMARS_0);
    elseif i == 2
        DMARS_y = DMARS_plant( DMARS_y_1,   DMARS_0, DMARS_u_1,   DMARS_0,   DMARS_0);
    elseif i == 3
        DMARS_y = DMARS_plant( DMARS_y_1, DMARS_y_2, DMARS_u_1, DMARS_u_2,   DMARS_0);
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

I         = [Ixx  , Iyy  , Izz  ]; 
I_p       = [Ixx_p, Iyy_p, Izz_p];
% xyz       = [x(:,1), y(:,1), z(:,1)];
Act_cmd_b_deg = Act_cmd_b * 180/pi;
Act_b_deg = Act_b * 180/pi;

lon_deg = 180/pi * lon;
latd_deg = 180/pi * latd;

M_alpha_beta_deg = [M_alpha(:), M_beta(:)] * 180/pi;

W_b_deg = W_b * 180/pi;


%% PLOTS

figure;
plot(e_1);
hold;
plot(e_2);
legend('e_1','e_2');

%% Footprint versus Altitude - SCATTER
% subplot(1,2,1);
% scatter(lon(1:n), latd(1:n), 3, alt(1:n), 'filled')
% title('Footprint X Altitude');
% xlabel('Lon [º]');
% ylabel('Latd [º]');
% colorbar;
% axis('equal');
% grid on;
% 
% % Rough trajectory plane - SCATTER
% subplot(1,2,2);
% distance = 6378.137*sqrt((lon-lon(1)).^2+(latd-latd(1)).^2) * pi/180;
% scatter(distance, alt/1e3, 3, 0:n, 'filled');
% title('Trajectory plane over time');
% xlabel('Distance from launch-pad [km]');
% ylabel('Altitude [km]');
% colorbar;
% axis('equal');
% grid on;

%% Footprint versus Altitude - PLOTS
figure();
subplot(1,2,1);
plot(lon_ref_deg(1:n), latd_ref_deg(1:n), 'b', 'LineWidth', 1)
hold on;
plot(lon_deg(1:n), latd_deg(1:n), 'r', 'LineWidth', 1)
title('Footprint X Altitude');
xlabel('Lon [º]');
ylabel('Latd [º]');
axis('equal');
grid on;
legend('reference','executed');

% Rough trajectory plane
subplot(1,2,2);
distance = 6378.137*sqrt((lon-lon(1)).^2+(latd-latd(1)).^2);
distance_ref = 6378.137*sqrt((lon_ref-lon_ref(1)).^2+(latd_ref-latd_ref(1)).^2);

% distance = 6378.137*sqrt((lon-lon(1)).^2+(latd-latd(1)).^2) * pi/180;
% distance_ref = 6378.137*sqrt((lon_ref_deg-lon_ref_deg(1)).^2+(latd_ref_deg-latd_ref_deg(1)).^2) * pi/180;

plot(distance_ref, alt_ref/1e3, 'b', 'LineWidth', 1);
hold on;
plot(distance, alt/1e3,'r', 'LineWidth', 1);
title('Trajectory plane over time');
xlabel('Distance from launch-pad [km]');
ylabel('Altitude [km]');
legend('reference','executed');
axis('equal');
grid on;

%% AoA and TVA_cmd
% figure();
% plot(TVA_cmd * 180/pi,'DisplayName','TVA-cmd')
% hold;
% plot( AoA_comp_deg(:,1) .* M_alpha_beta_deg(:,1)./M_alpha_beta_deg(:,2),'g','DisplayName','AoAp-comp-deg * Ma/Mb')
% plot( AoA_comp_deg(:,2) .* M_alpha_beta_deg(:,1)./M_alpha_beta_deg(:,2),'y','DisplayName','AoAy-comp-deg * Ma/Mb')
% legend();

%%
figure();
plot(angles_deg(1:n,1), 'b')
hold
plot(angles_deg(1:n,2), 'Color',[0.0, 0.66, 0.0])
plot(angles_deg(1:n,3), 'r')
% plot(Speed_pitch*180/pi,  'Color',[0.0, 0.85, 1.0]);
% plot(Speed_yaw*180/pi,'Color',[0.1, 0.9, 0.1]);
plot(pitch_ref_deg,'Color',[0.0, 0.0, 0.66])
plot(yaw_ref_deg,'Color',[0.0, 0.33, 0.0])
% legend('pitch','yaw','roll','Speed-pitch','Speed-yaw', 'Pitch-ref', 'Yaw-ref', 'Location','northwest')
legend('pitch','yaw','roll', 'Pitch-ref', 'Yaw-ref', 'Location','northwest')

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
% ylabel('Rate [º/s]')

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

%% Clean up 
clear('Coef'); clear('x');     clear('y');   clear('z');     clear('ans');   clear('I_times_ang_acc');
clear('Ixx');  clear('Iyy');   clear('Izz'); clear('Ixx_p'); clear('Iyy_p'); clear('Izz_p');            clear('angles');
clear('D_NB'); clear('D_l');   clear('L_C'); clear('S');     clear('data');  clear('AoA_pitch');        clear('AoA_yaw');
clear('Nz_d'); clear('Nzl_S'); clear('R_e'); clear('R_l');   clear('R_lh');  clear('data_size');        clear('Exp_Omega_k_T');
clear('dt');   clear('f');     clear('fx');  clear('fy');    clear('fz');    clear('Fe_traj');          clear('liftoffcounter');
clear('i');    clear('le');    clear('T');   clear('pitch'); clear('yaw');   clear('roll');             clear('position');
clear('q0');   clear('q1');    clear('q2');  clear('q3');    clear('w_b');   clear('alt_traj');         clear('M_alpha');       
clear('k1');   clear('k2');    clear('k3');  clear('k4');    clear('dl');    clear('M_beta');          % clear('data_100hz');
clear('Dref'); clear('Sref');  clear('Mach');clear('P');     clear('D');     clear('M_beta_deg');       clear('n');     
clear('ii');   clear('N');     clear('j');   clear('distance_ref');   clear('corr_angle_deg');
clear('');  clear('');

clear('DMARS_0');     clear('Sound_Velocity');      clear('distance');
clear('DMARS_u');     clear('Nozzle_eccentricity'); clear('Nozzle_misalignment');   
clear('DMARS_u_1');   clear('pitch_error');         clear('yaw_error');            
clear('DMARS_u_2');   clear('Speed_pitch_deg');     clear('Speed_yaw_deg');
clear('DMARS_u_3');   clear('pitch_error_dev');     clear('yaw_error_dev');
clear('DMARS_y');     clear('pitch_error_int');     clear('yaw_error_int');
clear('DMARS_y_1');   clear('pitch_error_old');     clear('yaw_error_old');
clear('DMARS_y_2');   clear('pitch_desired_deg');   clear('yaw_desired_deg'); 
%  clear('XYZ');