function [position, V, XYZ] = Translation_Integration(xyz, latd, lon, alt, acc, dt)
    
x = xyz(1:2);
y = xyz(3:4);
z = xyz(5:6);

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


R_e = 6378137;                                             %Raio equatorial                     [m]
f = 1/298.257223563;                                       %Achatamento polar terrestre         [-]
D_l = f * sin(2*latd) * (1 - f/2 + 2*f*sin(latd)^2);       %latd - latc                         [rad]
L_C = latd - D_l;                                          %geocentric latitude                 [rad]
R_l = R_e / sqrt( 1 + ((1-f)^-2 - 1) * sin(L_C)^2 );       %Raio local do elipsóide terrestre   [m]
R_l = R_l + alt;                       %Total radius from center of earth until the vehicle     [m]
R_lh = R_l * cos(L_C);                 %Radius of the circle of earth with constant latitude    [m]

Latd = latd + (1/R_l ) * (X(1) - x(1)) ;
Lon  = lon  - (1/R_lh) * (Y(1) - y(1)) ;
Alt  = alt  +            (Z(1) - z(1)) ;

position = [Latd, Lon, Alt];

V  = [X(2), Y(2), Z(2)]; 

XYZ = [X, Y, Z];
    
end