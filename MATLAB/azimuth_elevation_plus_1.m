clc
clear

pitch =  10 * (pi/180);
yaw   =  20 * (pi/180);
roll  =  30 * (pi/180);

D_NB = angle2dcm(pitch, yaw, roll, 'XYZ');

[p, y, r] = dcm2angle(D_NB, 'ZYZ');

azi = -p
ele = pi/2 - y
til = r

azi_new = azi + 1 * pi/180;
ele_new = ele + 1 * pi/180;

D_NB_new = angle2dcm( -azi_new, pi/2-ele_new, til, 'ZYZ');

[p, y, r] = dcm2angle(D_NB_new, 'XYZ');
(180/pi) * [p, y, r]
