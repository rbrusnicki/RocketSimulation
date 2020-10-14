function [p_new, V_new] = Flat_Earth_Translation_Integration(p, V, acc, dt)

x = [   p(1),  V(1) ];
y = [   p(2),  V(2) ];
z = [   p(3),  V(3) ];
  
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


p_new =  [X(1), Y(1), Z(1)];
           
V_new =   [X(2), Y(2), Z(2)];


end
