function [position, Velocity, XYZ] = Translation_Integration2( latd, lon, alt, V, acc, dt)

w_t = 7.2921151467e-5;                                     %Velocidade angular da Terra       rad/s]
R_e = 6378137;                                             %Raio equatorial                      [m]
f = 1/298.257223563;                                       %Achatamento polar terrestre          [-]
D_l = f * sin(2*latd) * (1 - f/2 + 2*f*sin(latd)^2);       %latd - latc                        [rad]
L_C = latd - D_l;                                          %geocentric latitude                [rad]
R_l = R_e / sqrt( 1 + ((1-f)^-2 - 1) * sin(L_C)^2 );       %Raio local do elipsóide terrestre    [m]
R_l = R_l + alt;                       %Total radius from center of earth until the vehicle      [m]
R_lh = R_l * cos(L_C);                 %Radius of the circle of earth with constant latitude     [m]


%Convert geodetic coordinates to Earth-centered Earth-fixed (ECEF) coordinates
p = LLA_2_ECEF(latd, lon, alt);

%Rotação necessária para reescrever a V e acc no triedo ecef a partir do triedo NRS do DLR
Rot1 =  T3( -lon ) * T2( -pi/2 + latd ) * T3( pi );

V(2) = V(2) - w_t * R_lh;
V_ecef = ( Rot1 * V' )';

acc = ( Rot1 * acc' )';

x = [   p(1),  V_ecef(1) ];
y = [   p(2),  V_ecef(2) ];
z = [   p(3),  V_ecef(3) ];

XYZ = [p(1), p(2), p(3)];
  
% Runge-Kuttas
fx = @(t, x) [ x(2) + acc(1) * t  , acc(1) ];
fy = @(t, y) [ y(2) + acc(2) * t  , acc(2) ];
fz = @(t, z) [ z(2) + acc(3) * t  , acc(3) ];

k1 = fx( 0       ,    x                );
k2 = fx( 0.5 * dt,    x + 0.5 * dt * k1);
k3 = fx( 0.5 * dt,    x + 0.5 * dt * k2);
k4 = fx( 0       ,    x + 1.0 * dt * k3);
X = x + (1/6)*(k1 + 2*k2 + 2*k3 + k4)*dt; 

k1 = fy( 0       ,    y                );
k2 = fy( 0.5 * dt,    y + 0.5 * dt * k1);
k3 = fy( 0.5 * dt,    y + 0.5 * dt * k2);
k4 = fy( 0       ,    y + 1.0 * dt * k3);
Y = y + (1/6)*(k1 + 2*k2 + 2*k3 + k4)*dt; 

k1 = fz( 0       ,    z                );
k2 = fz( 0.5 * dt,    z + 0.5 * dt * k1);
k3 = fz( 0.5 * dt,    z + 0.5 * dt * k2);
k4 = fz( 0       ,    z + 1.0 * dt * k3);
Z = z + (1/6)*(k1 + 2*k2 + 2*k3 + k4)*dt; 


% Nova posição ecef, compensando a rotação da terra durante o passo de integração
p_new = ( T3(w_t * dt) * [X(1); Y(1); Z(1)] )';

%Convert Earth-centered Earth-fixed (ECEF) coordinates to geodetic coordinates
position = ECEF_2_LLA( p_new );


D_l = f * sin(2*position(1)) * (1 - f/2 + 2*f*sin(position(1))^2);     
L_C = position(1) - D_l;                                          
R_l = R_e / sqrt( 1 + ((1-f)^-2 - 1) * sin(L_C)^2 );       
R_l = R_l + position(3);                       
R_lh = R_l * cos(L_C);                

% Nova velocidade ecef, compensando a rotação da terra durante o passo de integração
V_new =  T3(w_t * dt) * [X(2); Y(2); Z(2)];


%Rotação necessária para reescrever a V e acc no triedo NRS do DLR a partir do triedo ecef
Rot2 =  T3( -pi ) * T2( pi/2 - position(1) ) * T3( position(2) );


Velocity = ( Rot2 * V_new )';

Velocity(2) = Velocity(2) +  w_t * R_lh;


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