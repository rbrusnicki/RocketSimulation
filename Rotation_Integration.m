function [W, W_b, Q, angles] = Rotation_Integration(w, w_b, ang_acc, D_NB, q, dt )

W = w + ang_acc * dt;

W_b = w_b + ( D_NB * ang_acc' )' * dt;

S = [   0    -W_b(1) -W_b(2) -W_b(3);
      W_b(1)    0     W_b(3) -W_b(2);
      W_b(2) -W_b(3)    0     W_b(1);
      W_b(3)  W_b(2) -W_b(1)    0   ];

wb = norm( [W_b(1), W_b(2), W_b(3)] );

if wb ~= 0
    Exp_Omega_k_T = cos(wb*dt/2) * eye(4) + (1/wb) * sin(wb*dt/2) * S;
else
    Exp_Omega_k_T = eye(4) + (dt/2) * S;
end

Q = ( Exp_Omega_k_T * q' )';
Q = Q / norm(Q);         %normalização do quaternion

[pitch yaw roll] = quat2angle(Q, 'XYZ');       
angles = [pitch yaw roll];

end
