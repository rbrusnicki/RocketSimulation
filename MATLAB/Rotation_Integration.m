function [W, W_b, Q, angles] = Rotation_Integration(w, w_b, ang_acc, D_NB, q, dt )

W_b = w_b + ( D_NB * ang_acc' )' * dt;


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