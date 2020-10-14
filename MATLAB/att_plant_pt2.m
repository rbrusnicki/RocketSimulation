s = tf('s');

Mp = 0.248;
tp = 3.45;
ksi = -log(Mp)/sqrt(log(Mp)*log(Mp)+pi^2);
beta = asin(sqrt(1-ksi^2));
wd = pi/tp;
wn = wd/sin(beta);

PT2 = wn^2/(s^2 + 2*ksi*wn*s + wn^2)
% step(PT2)
% grid
% hold
 
% Mp2 = 0.2163;
% tp2 = 3.0;
% ksi2 = -log(Mp2)/sqrt(log(Mp2)*log(Mp2)+pi^2);
% beta2 = asin(sqrt(1-ksi2^2));
% wd2 = pi/tp2;
% wn2 = wd2/sin(beta2);
% sys2 = wn2^2/(s^2 + 2*ksi2*wn2*s + wn2^2)
% step(sys2)

i=1;
a(i,:) = 15.6;


K1 = 15.6;
K2 = 63.5;
sys1 = PT2 * K1 * (1/s) * (1/s);
sys2 = PT2 * K2 * (1/s) * (1/s);

PID(i,1:3) =  15.6/a(i,:)  * [ 0.8e-3,  8.9e-6,  18e-3 ]; 

P = PID(i,1);
I = PID(i,2);
D = PID(i,3);

PID = ( P + I * 1/s + D * s);

cl_sys1 = (PID * sys1) / ( 1 + PID * sys1);
cl_sys2 = (PID * sys2) / ( 1 + PID * sys2);

step(cl_sys1)
% hold
% step(cl_sys2)
grid

%% Josef Ettl's PT2
JE_PT2 = 41*s / (s^2 +  4.65*s + 41);

P = 1.866;
I = 0.6909;
D = 0.2955;
PID = ( P + I * 1/s + D * s);

sys = JE_PT2 * 1/s^2
cl_sys = (PID * sys ) / ( 1 + PID * sys);

step(100*cl_sys)
grid




