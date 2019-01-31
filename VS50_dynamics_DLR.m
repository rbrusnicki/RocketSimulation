% function [lon, latd, alt] = VS50_dynamics_DLR()
% clc;
% clear all;
set(0,'DefaultAxesXGrid','on')
set(0,'DefaultAxesYGrid','on')
set(0,'DefaultLineLineWidth', 2);
% set(0,'defaultFigurePosition', [180 558 1260 420])
set(0,'defaultFigurePosition', [10 100 800 800])
colordef white

%% Dados do veículo
le    = -10.9332;			 	 % Position of the 'Nozzle Throat' in Longitudinal Axis     [m]
dl = 0.0 * pi/180;               % Fins Misalignment                                        [rad]
Nz_d  = 0.820;                   % Nozzle exit diameter                                     [m]
Nzl_S = pi*(Nz_d)^2;             % Nozzle exit area                                         [m^2]

dt = 0.01;                      % Incremento de tempo utilizado                             [s]
T  = 85;                        % Tempo total de simulação                                  [s]
Dref = 1.72; %1.46;             % Diametro de referência do veículo                         [m]
Sref = pi * (Dref/2)^2;         % Area de referência do veículo                             [m]

% gera_data();
load('2018-07-03_VS50_conf3_with_DLR_att_100Hz.mat');

% n = length(data_100hz);              % número de iterações
n = 2000;

%% Leitura dos dados invariantes de voo

Fe_traj = data_100hz(:,12);              % Thrust Magnitude in nominal trajectory   [N]  
alt_traj = 1e3 * data_100hz(:,38);       % Altitude of the nominal trajectory       [m]

Fe      = data_100hz(:,55);              % Thrust Magnitude of S50 in vacuum        [N]
M       = data_100hz(:,25);              % Mass                         			[Kg]
Cnalfa  = data_100hz(:,10);              % Aerodynamic Coeficient       			[1/rad] 
Cnbeta  = data_100hz(:,11);              % Aerodynamic Coeficient       			[1/rad] 
Cd      = data_100hz(:, 5);              % Cd - coeficiente de arrasto 				[-]
Cld     = data_100hz(:, 7);              % Cld - coeficiente de 'desalinhamento'   	[-]
Clp     = data_100hz(:, 6);              % Coef. de Amortecimento Aerod. em x_b     [1/rad]
Cmq     = data_100hz(:, 8);              % Coef. de Amortecimento Aerod. em y_b     [1/rad]
Cnr     = data_100hz(:, 9);              % Coef. de Amortecimento Aerod. em z_b     [1/rad]
M_p     = data_100hz(:,34);              % d(Mass)/dt                               [Kg/s]
CoG	    = data_100hz(:,28);              % Center of gravity in longitudinal axis	[m]
CoP	    = data_100hz(:,31);              % Center of Pressure in longitudinal axis	[m]
Ixx     = data_100hz(:,15);              % Moment of inertia along x axis           [Kg.m^2]
Iyy     = data_100hz(:,14);              % Moment of inertia along y axis           [Kg.m^2]
Izz     = data_100hz(:,13);              % Moment of inertia along z axis           [Kg.m^2]
Ixx_p   = data_100hz(:,21);              % d(Ixx)/dt                                [Kg.m^2/s]
Iyy_p   = data_100hz(:,20);              % d(Iyy)/dt                                [Kg.m^2/s]
Izz_p   = data_100hz(:,19);              % d(Izz)/dt                                [Kg.m^2/s]


%% Dados variantes de voo (LOGDATA)


TVA_cmd     = zeros(n,2);  % TVA nozzle pitch and yaw angles commands in DLR NRS. [rad]    %(-)values produce (+)pitch and (+)yaw (left-hand rule)     
% TVA_cmd_b   = zeros(n,2);  % TVA nozzle pitch and yaw angles commands in DLR BRS. [rad]    %(-)values produce (-)pitch and (-)yaw (right-hand rule)
% TVA_b       = zeros(n,2);  % TVA nozzle pitch and yaw angles in DLR Body Ref. Sys.[rad]    %(-)values produce (-)pitch and (-)yaw (right-hand rule)

Act_cmd_b   = zeros(n,2);               % Nozzle angle command for actuators at 315º and  225º [rad]
Act_b       = zeros(n,2);               % Nozzle angle for actuators at 315º and at 225º       [rad]

latd        = zeros(n,1);               % Geodetic Latitude of the Vehicle during flight       [rad]
alt         = zeros(n,1);               % Geodetic Altitude of the Vehicle during flight       [rad]
lon         = zeros(n,1);               % Geodetic Longitude of the Vehicle during flight      [rad]
q           = [ones(n,1) zeros(n,3)];   % Quaternion of atitude                                [-]
angles      = zeros(n,3);               % Euler angles in DLR Standard: 1-2-3                  [rad]
angles_deg  = zeros(n,3);               % Euler angles in DLR Standard: 1-2-3                  [º]
acc         = zeros(n,3);               % Aceleration vector in DLR Navigation Ref. Sys.       [m/s^2]
ang_acc     = zeros(n,3);               % Angular Aceleration vector in DLR Nav. Ref. Sys.     [m/s^2]
V           = zeros(n,3);               % Velocity vector in DLR Navigation Ref. Sys.          [m/s]
V_wind      = zeros(n,3);               % Wind Velocity vector in DLR Navigation Ref. Sys.     [m/s]
Pdin        = zeros(n,1);               % Dynamic Pressure                                     [Pa]
Patm        = zeros(n,1);               % Atmospheric Pressure                                 [Pa]
FE_b        = zeros(n,3);               % Thrust Force Vector in DLR Body Reference System     [m]
FE          = zeros(n,3);               % Thrust Force Vector in DLR Navigation Ref. Sys.      [m]
FG          = zeros(n,3);               % Gravitational Force in DLR Navigation Ref. Sys.      [m]
FG_b        = zeros(n,3);               % Gravitational Force in DLR Body Ref. Sys.            [m]
FA          = zeros(n,3);               % Aerodynamic Force in DLR Navigation Reference System [m]
FCo         = zeros(n,3);               % Coriolis Force in DLR Navigation Reference System    [m]
W_b         = zeros(n,3);               % Angular Velocity in DLR Body Reference System        [rad/s]
W           = zeros(n,3);               % Angular Velocity in DLR Navigation Reference System  [rad/s]
AoA_pitch   = zeros(n,1);               % Angle of Attack in XZ plane of DLR Nav. Ref. Sys.    [rad]
AoA_yaw     = zeros(n,1);               % Angle of Attack in YZ plane of DLR Nav. Ref. Sys.    [rad]
MCo         = zeros(n,3);               % Coriolis Moment in DLR Navigation Reference System   [N.m]
MFE         = zeros(n,3);               % Propulsive Moment in DLR Navigation Reference Sys.   [N.m]
MFA         = zeros(n,3);               % Aerodynamic Moment in DLR Navigation Reference Sys.  [N.m]
MA_f        = zeros(n,3);               % Moment due to Fins Misalignment in DLR Nav. Ref. Sys.[N.m]
MA_d        = zeros(n,3);               % Aerodynamic Damping Moment in DLR Nav. Ref. Sys.     [N.m]
Mextra      = zeros(n,3);               %                                                      [N.m]
M_alpha     = zeros(n,1);
M_beta      = zeros(n,1);


alt(1,1) = 50;                          % Initial Altitude                 [m]
latd(1,1) = -2.31599 * pi/180;         % Initial Geodetic Latitud         [rad]
lon(1,1) = -44.3677 * pi/180;         % Initial Longitude                [rad]


%%

for i = 1:n%(n-1)
    %% Pressão Dinâmica e Pressão Atmosférica local
    [Pdin(i), Patm(i)] = Dynamic_Pressure( alt(i), V(i,:), V_wind(i,:) );
    
    %% Direct Cossine Matrix
    D_NB = DCM_Navigation_to_Body(q(i,:));
    
    %% Angles of Attack no Triedo de Navegação do DLR
    [AoA_pitch(i), AoA_yaw(i)] =  Angle_Of_Attack_in_DLR_Navigation_Reference_System(D_NB, V(i,:), V_wind(i,:));
    
    %% TVA command Conversion from Inertial To DLR Body Reference System
    if i == 1
        Act_cmd_b(i,:) = Act_Command_Conversion_Inertial_To_Body(TVA_cmd(i,:),         [0, 0], angles(i,3),        0);
    else
        Act_cmd_b(i,:) = Act_Command_Conversion_Inertial_To_Body(TVA_cmd(i,:), TVA_cmd(i-1,:), angles(i,3), W_b(i,3));
    end
    
    %% TVA's command input to TVA's plant output
    if i == 1
        Act_b(i,:) = Act_Plant(        [0,0],        [0,0],            [0,0],            [0,0]);
    elseif i == 2
        Act_b(i,:) = Act_Plant( Act_b(i-1,:),        [0,0], Act_cmd_b(i-1,:),            [0,0]);
    else
        Act_b(i,:) = Act_Plant( Act_b(i-1,:), Act_b(i-2,:), Act_cmd_b(i-1,:), Act_cmd_b(i-2,:));
    end

     Act_b(i,:) = [-0.141 * pi/180, 0];                   %%%%%%%%%%%%%%%% R#!%#$%@$¨%@$&%$@¨$#¨$#¨@¨$ OVERWRITING FOR DEBUG
    
    
%% FORCES

    %% Força de Empuxo no Triedo do Corpo do DLR 
    FE_b(i,:) = Thrust_Force_in_DLR_Body_Reference_System(Fe(i)-Patm(i)*Nzl_S , Act_b(i,:));

    %% Força de Empuxo no Triedo de Navegação do DLR 
    FE(i,:) = Thrust_Force_in_DLR_Navigation_Reference_System(FE_b(i,:), D_NB);

    %% Força Gravitacional instantânea no Triedo de Navegação do DLR
    FG(i,:) = Gravitational_Force_in_DLR_Navigation_Reference_System(latd(i), alt(i), M(i));        

    %% Força Gravitacional instantânea no Triedo do Corpo do DLR 
    FG_b(i,:) = Gravitational_Force_in_DLR_Body_Reference_System(D_NB, FG(i,:));

    %% Força Aerodinâmica no triedo de Navegação do DLR 
    FA(i,:) = Aerodynamic_Force_in_DLR_Navigation_Reference_System(q(i,:), Cnalfa(i), Cnbeta(i), Cd(i), Pdin(i), AoA_pitch(i), AoA_yaw(i), Sref);

    %% Força de Coriolis no triedo de Navegação do DLR
    FCo(i,:) = Coriolis_Force_in_DLR_Navigation_Reference_System(D_NB, M_p(i), W_b(i,:), CoG(i), le);
    
%% MOMENTS

    %% Momento de Coriolis
    MCo(i,:) = Coriolis_Moment_in_DLR_Navigation_Reference_System(Ixx_p(i), Iyy_p(i), Izz_p(i), D_NB, M_p(i), W_b(i,:), CoG(i), le);

    %% Momento Propulsivo
    MFE(i,:) = Thrust_Moment_in_DLR_Navigation_Reference_System(FE(i,:), D_NB, CoG(i), le);
    
    %% Momento Aerodinâmico
    MFA(i,:) = Aerodynamic_Moment_in_DLR_Navigation_Reference_System(FA(i,:), D_NB, CoG(i), CoP(i));
    
    %% Momento Aerodinâmico devido ao Desalinhamento de Empenas in DLR Navigation Reference System
    MA_f(i,:) = Aerodynamic_Moment_due_to_Fins_Misalignment(Pdin(i), D_NB, Cld(i), dl);
    
    %% Momento Aerodinâmico de Amortecimento
    Coef = diag([ Cmq(i,:), Cnr(i,:), Clp(i,:)]);
    MA_d(i,:) = Aerodynamic_Damping_Moment(D_NB, W_b(i,:), Coef, Pdin(i,:), V(i,:), V_wind(i,:));
       
%% ACELERATIONS

    %% Aceleration in DLR Navigation Reference System
    % If thrust force is smaller than gravitational force, aceleration must be zero at the beginning
    if norm(FE(i,:)) < norm(FG(i,:)) && i*dt < 30
        acc(i,:) = [0, 0, 0];
    else
        acc(i,:) = ( FG(i,:) + FA(i,:) + FE(i,:) + FCo(i,:) )/ M(i,:);
    end
    
    %% Angular Aceleration in DLR Navigation Reference System
    I = diag([Ixx(i), Iyy(i), Izz(i)]); %BRS
  
    ang_acc_b = Angular_Aceleration_in_DLR_Body_Reference_System(I, W_b(i,:), D_NB, MCo(i,:), MFE(i,:), MFA(i,:), MA_f(i,:), MA_d(i,:) ); %BRS
    
    ang_acc(i,:) = (D_NB' * ang_acc_b)'; %NRS
 
%% M_ALPHA & M_BETA
    
    %% M_alpha
    M_alpha(i) = Pdin(i,:) * Cnalfa(i) * Sref * (CoP(i) - CoG(i)) * (1 * pi / 180) / Ixx(i);
    
    %% M_beta
    M_beta(i) = norm(FE(i,:)) * (le - CoG(i)) * (-1 * pi / 180) / Ixx(i);
    
%% INTEGRATIONS

    %% Translation Integration using 4th order Runge-Kutta
     
%     if i==1
%         x = zeros(n,2);              % initial value of x [m] and x' [m/s]  at time 0 [s] 
%         y = zeros(n,2);              % initial value of y [m] and y' [m/s]  at time 0 [s] 
%         z = zeros(n,2);              % initial value of z [m] and z' [m/s]  at time 0 [s]     
%     end
%     
%     xyz = [x(i,:), y(i,:), z(i,:)];
%     
%     [position, V(i+1,:), XYZ] = Translation_Integration(xyz, latd(i), lon(i), alt(i), acc(i,:), dt);
    [position, V(i+1,:)] = Translation_Integration2(latd(i), lon(i), alt(i), V(i,:), acc(i,:), dt);
    
    latd(i+1) = position(1);
    lon(i+1)  = position(2);
    alt(i+1)  = position(3);
    
%     x(i+1,:) = XYZ(1:2);
%     y(i+1,:) = XYZ(3:4);
%     z(i+1,:) = XYZ(5:6);
    
    %% Attitude Integration using Quaternion Cinematics  
    
    [W(i+1,:), W_b(i+1,:), q(i+1,:), angles(i+1,:)] = Rotation_Integration(W(i,:), W_b(i,:), ang_acc(i,:), D_NB, q(i,:), dt );
    
    angles_deg(i+1,:) = angles(i+1,:) * 180/pi;
    
    
end

%% Organize
AoA_deg   = [AoA_pitch,  AoA_yaw] * 180/pi;
I         = [Ixx  , Iyy  , Izz  ]; 
I_p       = [Ixx_p, Iyy_p, Izz_p];
% xyz       = [x(:,1), y(:,1), z(:,1)];
Act_cmd_b_deg = Act_cmd_b * 180/pi;
Act_b_deg = Act_b * 180/pi;

lon = 180/pi * lon;
latd = 180/pi * latd;

M_alpha_beta_deg = [M_alpha(:), M_beta(:)] * 180/pi;


%% Footprint versus Altitude
figure();
scatter(lon(1:n), latd(1:n), 3, alt(1:n), 'filled')
title('Footprint X Altitude');
xlabel('Lon [º]');
ylabel('Latd [º]');
colorbar;
axis equal;

%% Clean up 
clear('Coef'); clear('XYZ');   clear('x');   clear('y');     clear('z');     clear('ang_acc_b'); clear('I_times_ang_acc');
clear('Ixx');  clear('Iyy');   clear('Izz'); clear('Ixx_p'); clear('Iyy_p'); clear('Izz_p');     clear('angles');
clear('D_NB'); clear('D_l');   clear('L_C'); clear('S');     clear('data');  clear('AoA_pitch'); clear('AoA_yaw');
clear('Nz_d'); clear('Nzl_S'); clear('R_e'); clear('R_l');   clear('R_lh');  clear('data_size'); clear('Exp_Omega_k_T');
clear('dt');   clear('f');     clear('fx');  clear('fy');    clear('fz');    clear('Fe_traj');   clear('Mextra_b'); 
clear('i');    clear('le');    clear('T');   clear('pitch'); clear('yaw');   clear('roll');      clear('position');
clear('q0');   clear('q1');    clear('q2');  clear('q3');    clear('w_b');   clear('alt_traj');  clear('M_alpha');       
clear('k1');   clear('k2');    clear('k3');  clear('k4');    clear('dl');    clear('M_beta');    clear('data_100hz');
clear('Dref'); clear('Sref');  

