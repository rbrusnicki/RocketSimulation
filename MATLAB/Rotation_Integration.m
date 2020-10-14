function [W, W_b, Q, angles] = Rotation_Integration(w, w_b, ang_acc, D_NB, q, dt )

W_b = w_b + ( D_NB * ang_acc' )' * dt;
% % wx = [   0,  w_b(1) ];
% % wy = [   0,  w_b(2) ];
% % wz = [   0,  w_b(3) ];
% % 
% % % Runge-Kuttas
% % fx = @(t, wx) [ wx(2) + ang_acc(1) * t  , ang_acc(1) ];
% % fy = @(t, wy) [ wy(2) + ang_acc(2) * t  , ang_acc(2) ];
% % fz = @(t, wz) [ wz(2) + ang_acc(3) * t  , ang_acc(3) ];
% % 
% % k1 = fx( 0       ,    wx                );
% % k2 = fx( 0.5 * dt,    wx + 0.5 * dt * k1);
% % k3 = fx( 0.5 * dt,    wx + 0.5 * dt * k2);
% % k4 = fx( 0       ,    wx + 1.0 * dt * k3);
% % WX = wx + (1/6)*(k1 + 2*k2 + 2*k3 + k4)*dt; 
% % 
% % k1 = fy( 0       ,    wy                );
% % k2 = fy( 0.5 * dt,    wy + 0.5 * dt * k1);
% % k3 = fy( 0.5 * dt,    wy + 0.5 * dt * k2);
% % k4 = fy( 0       ,    wy + 1.0 * dt * k3);
% % WY = wy + (1/6)*(k1 + 2*k2 + 2*k3 + k4)*dt; 
% % 
% % k1 = fz( 0       ,    wz                );
% % k2 = fz( 0.5 * dt,    wz + 0.5 * dt * k1);
% % k3 = fz( 0.5 * dt,    wz + 0.5 * dt * k2);
% % k4 = fz( 0       ,    wz + 1.0 * dt * k3);
% % WZ = wz + (1/6)*(k1 + 2*k2 + 2*k3 + k4)*dt; 
% % 
% % W_b = [WX(2), WY(2), WZ(2)];


W = (D_NB' * W_b')';

S = [   0    -W_b(1) -W_b(2) -W_b(3);
      W_b(1)    0     W_b(3) -W_b(2);
      W_b(2) -W_b(3)    0     W_b(1);
      W_b(3)  W_b(2) -W_b(1)    0   ];
  
S_old = [   0   -w_b(1) -w_b(2) -w_b(3);
         w_b(1)    0     w_b(3) -w_b(2);
         w_b(2) -w_b(3)    0     w_b(1);
         w_b(3)  w_b(2) -w_b(1)    0   ];

wb = norm( [W_b(1), W_b(2), W_b(3)] );


if wb ~= 0
    Exp_Omega_k_T = cos(wb*dt/2) * eye(4) + (1/wb) * sin(wb*dt/2) * S + (S*S_old - S_old*S)/48;
else
    Exp_Omega_k_T = eye(4) + (dt/2) * S;
end

Q = ( Exp_Omega_k_T * q' )';
Q = Q / norm(Q);         %normalização do quaternion

[pitch yaw roll] = quat2angle(Q, 'XYZ');       
angles = [pitch yaw roll];

end

 
% if varfi > 1e-6
%     B = cos(varfi/2) * eye(4) + sin(varfi/2)/varfi * Q + (Q*Q_old - Q_old*Q)/48;
% else
%     B = eye(4) + 1/2 * Q;
% end