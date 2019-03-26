function[SKEW] = skew(W, q)

q0 = q(1);  % escalar component
q1 = q(2);  % ex
q2 = q(3);  % ey
q3 = q(4);  % ez

% 'Rotates' from Navigation Reference System to Body Reference System
%If r1 is a vector writen in Nav. Ref. Sys., then DCM*r1 is the same vector writen in Body Ref. Sys.
%If r2 is a vector writen in Body Ref. Sys., then DCM'*r2 is the same vector writen in Nav. Ref. Sys.
DCM = [ (q0^2+q1^2-q2^2-q3^2)     2*(q1*q2+q0*q3)     2*(q1*q3-q0*q2);
           2*(q1*q2-q0*q3)      q0^2-q1^2+q2^2-q3^2   2*(q2*q3+q0*q1);
           2*(q1*q3+q0*q2)        2*(q2*q3-q0*q1)   q0^2-q1^2-q2^2+q3^2];

W_b = DCM * W';

SKEW =   0.5* [   0  , -W_b(1), -W_b(2), -W_b(3);
               W_b(1),     0  ,  W_b(3), -W_b(2);
               W_b(2), -W_b(3),     0  ,  W_b(1);
               W_b(3),  W_b(2), -W_b(1),     0  ];