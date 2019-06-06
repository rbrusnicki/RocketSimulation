function [] = Compute_the_Coeficients(Airflow_speed, Temperature)

K = 1.0/340;

Mach = K * sqrt(Temperature);


load('Roll Drive.txt');
vento_1(:,1) = interp1(VENTO_1(:,1),VENTO_1(:,2),tempo,'spline');
