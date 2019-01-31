function [position] = LLA_2_ECEF(latd, lon, alt)

f = 1/298.257223563;                                       %Achatamento polar terrestre          [-]
R_e = 6378137;                                             %Raio equatorial                      [m]
ex = sqrt( 1 - ( 1 - f )^2 );                              %Excentricidade                       [-]

e2 = ex ^ 2;
sinlatd = sin(latd);
coslatd = cos(latd);

N  = R_e ./ sqrt(1 - e2 * sinlatd.^2);
x = (N + alt) .* coslatd .* cos(lon);
y = (N + alt) .* coslatd .* sin(lon);
z = (N*(1 - e2) + alt) .* sinlatd;

position = [x,y,z];
end



