function[AoA_pitch, AoA_yaw, Speed_pitch, Speed_yaw] =  Angle_Of_Attack_in_DLR_Navigation_Reference_System2( V, V_wind, q)
% Função que calcula os Anglos de Ataque no triedo de Navegação do DLR 
%
% INPUTS VETORIAIS: 
% q:     quaternion de atitude   
% V:     Velocidade do veículo no triedo de navegação do DLR [m/s]
% Vwind: Velocidade do vento no triedo de navegação do DLR [m/s]
%
% OUTPUT VETORIAL:
% AoA_pitch: Angulo de ataque no triedo de navegação do DLR [rad]
% AoA_yaw: Angulo de derrapagem no triedo de navegação do DLR [rad]
%
% Autor: Roberto Brusnicki
% Data: 04/04/19


% Ettl code 
% qut(pl_q0, -pl_q1, -pl_q2, -pl_q3, q0s, q1s, q2s, q3s) % Quaternion subtract
% quaternion_euler(s, x, y, z)                           % Conversion from quaternions to Euler Angle
% AoAp = Math.Asin(Math.Sin(pitch * rad1) * Math.Cos(yaw * rad1)) / rad1      % 'AoA body fixed //simple correction of AoAp if AoAy is big
% AoAy = yaw


%vetor velocidade do foguete, escrito no triedo de Navegação do DLR
VV = V - V_wind;

Vx = VV(1);
Vy = VV(2);
Vz = VV(3);
Vmod = norm(VV);

if(Vmod == 0)
    Speed_pitch = 0;
    Speed_yaw = 0;
    AoA_pitch = 0;
    AoA_yaw = 0;
else

    azi = atan2(-Vy,Vx);
    ele = atan2(Vz, sqrt(Vx^2+Vy^2));

    % b = cos(ele);

    x = cos(ele) * sin(azi);
    y = cos(ele) * cos(azi);
    z = sin(ele);

    Speed_yaw = asin(y) ;
    Speed_pitch = atan2(x,z) ;
    
    
    
%     
%     Speed_pitch = asin(-Vy/Vmod);          % rad
%     Speed_yaw = asin( Vx/(Vmod*cos(Speed_pitch)) );        % rad

%     S_q( = angle2quat(Speed_pitch, Speed_yaw, 0, 'XYZ');
    S_q  = [ cos(Speed_pitch/2) * cos(Speed_yaw/2), ...
             sin(Speed_pitch/2) * cos(Speed_yaw/2), ...
             cos(Speed_pitch/2) * sin(Speed_yaw/2), ...
             sin(Speed_pitch/2) * sin(Speed_yaw/2)];
      
         
    % NÃO IMPLEMENTADO NO LABVIEW:
    [r1 r2 r3] = quat2angle(q, 'XYZ');
    q = angle2quat(r1,r2,0, 'XYZ');
%     
         
    q_c = [ q(1), -q(2), -q(3), -q(4)]';
               
%     S = quatmultiply( q_c, S_q( );


% Calculate vector portion of quaternion product
% vec = s1*v2 + s2*v1 + cross(v1,v2)
vec = [ q_c(1) * S_q(2)                    q_c(1) * S_q(3)                q_c(1) * S_q(4)] + ...
      [ S_q(1) * q_c(2)                    S_q(1) * q_c(3)                S_q(1) * q_c(4)] + ...
      [ q_c(3) * S_q(4) - q_c(4) * S_q(3)  q_c(4) * S_q(2)-q_c(2)*S_q(4)  q_c(2)*S_q(3)-q_c(3)*S_q(2)];

% Calculate scalar portion of quaternion product
% scalar = s1*s2 - dot(v1,v2)
scalar = q_c(1)*S_q(1) - q_c(2)*S_q(2) - q_c(3)*S_q(3) - q_c(4)*S_q(4);
    
S = [scalar  vec];

%     [PPitch, YYaw, RRoll] = quat2angle(S, 'XYZ');
   PPitch = atan2( -2*(S(3)*S(4) - S(1)*S(2)),   S(1)^2 - S(2)^2 - S(3)^2 + S(4)^2 );
   YYaw   =  asin( 2*(S(2)*S(4) + S(1)*S(3)) );
%  RRoll  = atan2( -2*(S(2)*S(3) - S(1)*S(4)), S(1)^2 + S(2)^2 - S(3)^2 - S(4)^2 );
        
   

    AoA_pitch = asin(sin(PPitch)*cos(YYaw));
    AoA_yaw = YYaw;
end


