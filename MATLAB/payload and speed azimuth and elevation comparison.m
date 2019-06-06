%% CALCULATIONS FOR THE PAYLOAD FROM THE QUATERNION
q = [0.99257470463289454 0.092849915777521844 0.078577022970267182 0.000018004425107697591];

[pl_pitch pl_yaw pl_roll] = quat2angle(q, 'xyz');     

payload_attitude = [pl_pitch pl_yaw pl_roll] * 180/pi 

x = sin(pl_yaw);
y = -sin(pl_pitch) * cos(pl_yaw);
l = sqrt(x^2 + y^2);
z = sqrt(1 - l^2);

payload_xyz_point = [x y z]

if (abs(pl_pitch) > pi/2)
    ele = -atan2(z,l);
else
    ele = atan2(z,l);
end

azi = atan2(-y,x);

payload_elevation = ele * 180/pi
payload_azimuth = azi * 180/pi

%% CALCULATIONS FOR THE SPEED FROM THE VELOCITY
Speed_Velocity = [73.093 -86.126 454.993]

x = Speed_Velocity(1) / norm(Speed_Velocity);
y = Speed_Velocity(2) / norm(Speed_Velocity);
z = Speed_Velocity(3) / norm(Speed_Velocity);

l = sqrt(x^2 + y^2);

speed_xyz_point = [x y z]

if (abs(pl_pitch) > pi/2)
    ele = -atan2(z,l);
else
    ele = atan2(z,l);
end

azi = atan2(-y,x);

speed_elevation = ele * 180/pi
speed_azimuth = azi * 180/pi


speed_pitch = atan2(-y, z)    * 180/pi    % Must be between -180º and  +180º
speed_yaw   =  asin(x)    * 180/pi         % Must be between  -90º and   +90º

%%

% create a certain quaternion from euler
pitch_0 = 0;
yaw_0   = 0;
roll_0  = 0;

[q0, q1, q2, q3] = angle2quat(pitch_0, yaw_0, roll_0, 'xyz')
 q = angle2quat( pitch, roll, yaw, 'YXZ' )

% JOSEF ETTL CONVERSION CODE FROM QUAT TO EULER

q0 = 1;
q1 = 0;
q2 = 0;
q3 = 0;

m11 = 2 * q0 ^ 2 - 1 + 2 * q1 ^ 2;
m21 = 2 * q1 * q2 - 2 * q0 * q3;
% m22 = 2 * q0 ^ 2 - 1 + 2 * q2 ^ 2;
% m23 = 2 * q2 * q3 + 2 * q0 * q1;
m32 = 2 * q2 * q3 - 2 * q0 * q1;
m31 = 2 * q1 * q3 + 2 * q0 * q2;
m33 = 2 * q0 ^ 2 - 1 + 2 * q3 ^ 2;

pitch = atan2(-m32,m33) * 180/pi;

% if m31 < -1
%     m31 = -1;
% end
% if m31 > 1
%     m31 = 1;
% end
yaw = asin(m31) * 180/pi;
roll = atan2(-m21, m11) * 180/pi;

% if abs(m31) == 1
%     pitch = atan2(m23, m22) * 180/pi;
%     roll = 0;
% end

attitude = [pitch, yaw, roll]


