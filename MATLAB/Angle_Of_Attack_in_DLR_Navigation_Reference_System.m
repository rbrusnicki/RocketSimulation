function[AoA_pitch, AoA_yaw, Speed_pitch, Speed_yaw] =  Angle_Of_Attack_in_DLR_Navigation_Reference_System(D_NB, V, V_wind)
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
% Data: 21/11/18

%vetor unitário que aponta na direção longitudinal do foguete, escrito no triedo de Navegação do DLR
r = D_NB' * [0; 0; 1];

%vetor velocidade do foguete, escrito no triedo de Navegação do DLR
v = V - V_wind;

Vx = v(1);
Vy = v(2);
Vz = v(3);
Vmod = norm(v);

if(Vmod == 0)
    Speed_pitch = 0;
    Speed_yaw = 0;
else
    Speed_pitch = asin(-Vy/Vmod);                          % rad
    Speed_yaw = asin( Vx/(Vmod*cos(Speed_pitch)) );        % rad
end

a1 = atan2( v(3), v(2) );
a2 = atan2( r(3), r(2) );
a3 = atan2( r(3), r(1) );
a4 = atan2( v(3), v(1) );

if ( v(3)^2+v(2)^2 == 0 || r(3)^2+r(2)^2 == 0 )
    AoA_pitch = 0;
else
    AoA_pitch = ( a1 - a2 );
end

if ( r(3)^2+r(1)^2 == 0 || v(3)^2+v(1)^2 == 0 )
    AoA_yaw = 0;
else
    AoA_yaw = ( a3 - a4 )/cos(AoA_pitch);
end

% Ettl code 
% qut(pl_q0, -pl_q1, -pl_q2, -pl_q3, q0s, q1s, q2s, q3s) % Quaternion subtract
% 
% quaternion_euler(s, x, y, z)                           % Conversion from quaternions to Euler Angle
% 
% AoAp = Math.Asin(Math.Sin(pitch * rad1) * Math.Cos(yaw * rad1)) / rad1      % 'AoA body fixed //simple correction of AoAp if AoAy is big
% 
% AoAy = yaw
