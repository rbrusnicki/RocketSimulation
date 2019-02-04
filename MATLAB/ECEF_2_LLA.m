function [LLA] = ECEF_2_LLA(position)

x = position(1);
y = position(2);
z = position(3);

a = 6378137;          
b = a * (1 - 1/298.257223563);      

if z < 0
    b = -b;
end

R = sqrt(x^2 + y^2);
E = (b*z - a^2 + b^2)/( a*R );
F = (b*z + a^2 - b^2)/( a*R );
P = 4*(E*F + 1)/3;
Q = 2*(E^2 - F^2);
D = sqrt(P^3 + Q^2);
v = (D - Q)^(1/3) - (D + Q)^(1/3);
G = (sqrt(E^2 + v) + E)/2;
t = sqrt(G^2 + (F - v*G) / (2*G - E)) - G;

latd = atan2( a*(1 - t^2) , 2*b*t);
lon = atan2(y, x);
altd = (R - a*t) * cos(latd) + (z - b) * sin(latd);
 
if z < 0
    altd = -altd;
end

if latd > pi/2
    latd = latd - pi;
end

LLA = [latd, lon, altd];
     
% latd = latd *180/pi
% lon = lon *180/pi
% altd
