function LLA = ECEF_2_LLA2( p )

f = 1/298.257223563;                                       %Achatamento polar terrestre          [-]
R_e = 6378137;                                             %Raio equatorial                      [m]
ex = sqrt( 1 - ( 1 - f )^2 );                              %Excentricidade                       [-]

x = p(:,1);
y = p(:,2);
z = p(:,3);

% Ellipsoid constants
a  = R_e;                % Semimajor axis
b = a * (1 - f);         % Semiminor axis

e2 = ex ^ 2;             % Square of first eccentricity
ep2 = e2 / (1 - e2);     % Square of second eccentricity


% Longitude
lon = atan2(y,x);

% Distance from Z-axis
rho = hypot(x,y);

% Bowring's formula for initial parametric (beta) and geodetic (phi) latitudes
beta = atan2(z, (1 - f) * rho);
latd = atan2(z  + b * ep2 * sin(beta).^3, rho - a * e2  * cos(beta).^3);

% Fixed-point iteration with Bowring's formula
% (typically converges within two or three iterations)
betaNew = atan2((1 - f)*sin(latd), cos(latd));
count = 0;
while any(beta(:) ~= betaNew(:)) && count < 5
    beta = betaNew;
    latd = atan2(z   + b * ep2 * sin(beta).^3, rho - a * e2  * cos(beta).^3);
    betaNew = atan2((1 - f)*sin(latd), cos(latd));
    count = count + 1;
end

% Calculate ellipsoidal height from the final value for latitude
sinlatd = sin(latd);
N = a ./ sqrt(1 - e2 * sinlatd.^2);
alt = rho .* cos(latd) + (z + e2 * N .* sinlatd) .* sinlatd - N;



LLA = [latd, lon, alt];

end

