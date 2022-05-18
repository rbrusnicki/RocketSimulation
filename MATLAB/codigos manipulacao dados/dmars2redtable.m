%% Conversion function from dmars euler angles to the red table angles

% mesa : Z-Y-X   outter-middle-inner
% dmars: X-Y-Z

dmars_pitch = 0 * pi/180;
dmars_yaw   = 0 * pi/180;
dmars_roll  = 0 * pi/180;

% quat = angle2quat(dmars_pitch, dmars_yaw, dmars_roll,'XYZ');

angles = [dmars_pitch dmars_yaw dmars_roll];

cang = cos( angles/2 );
sang = sin( angles/2 );

quat = [ cang(:,1).*cang(:,2).*cang(:,3) - sang(:,1).*sang(:,2).*sang(:,3), ...
         cang(:,1).*sang(:,2).*sang(:,3) + sang(:,1).*cang(:,2).*cang(:,3), ...
         cang(:,1).*sang(:,2).*cang(:,3) - sang(:,1).*cang(:,2).*sang(:,3), ...
         cang(:,1).*cang(:,2).*sang(:,3) + sang(:,1).*sang(:,2).*cang(:,3)];
        

% [outter, middle, inner] = quat2angle(quat, 'ZYX');

r11 =  2.*(quat(:,2).*quat(:,3) + quat(:,1).*quat(:,4)); 
r12 = quat(:,1).^2 + quat(:,2).^2 - quat(:,3).^2 - quat(:,4).^2;
r21 = -2.*(quat(:,2).*quat(:,4) - quat(:,1).*quat(:,3));
r31 =  2.*(quat(:,3).*quat(:,4) + quat(:,1).*quat(:,2));
r32 =  quat(:,1).^2 - quat(:,2).^2 - quat(:,3).^2 + quat(:,4).^2;

outter = atan2( r11, r12 );
middle = asin ( r21 );   
inner  = atan2( r31, r32 );
                         
                              
middle = -middle - pi/2;

[outter, middle, inner] * 180/pi