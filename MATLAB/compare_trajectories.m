%% Footprint versus Altitude
figure();
scatter(lon(1:n), latd(1:n), 3, alt(1:n), 'filled')
title('Footprint X Altitude');
xlabel('Lon [º]');
ylabel('Latd [º]');
colorbar;
axis equal;

%% Rough trajectory plane
figure();
distance = 6378*sqrt((lon-lon(1)).^2+(latd-latd(1)).^2);
plot(distance,alt);
title('Trajectory plane');
xlabel('Distance from launch-pad [Km]');
ylabel('Altitude [Km]');
grid;


%%
alt_ref = data_100hz(:,38)';
lon_ref = data_100hz(:,41)';
latd_ref = data_100hz(:,43');

figure();
plot(latd_ref, 'b');
hold;
plot(latd, 'r');
title('Latitude Comparision');
legend('latd-ref','latd');

figure();
plot(lon_ref, 'b');
hold;
plot(lon, 'r');
title('Longitude Comparision');
legend('lon-ref','lon');

figure();
plot(alt_ref, 'b');
hold;
plot(alt/1000, 'r');
title('Altitude Comparision');
legend('alt-ref','alt');

R_eq = 6378.137;

kk = 8200;
 
Errors_in_km = [R_eq * (latd(kk) - latd_ref(kk)) * pi/180;
                R_eq * (lon(kk) - lon_ref(kk))* pi/180;
                alt_ref(kk) - alt(kk)/1000]
            
Total_distance_in_Km = norm(Errors_in_km)


