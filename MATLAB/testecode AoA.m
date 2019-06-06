clc

payload_pitch =  15  *  pi/180;             % between -180º and  +180º
payload_yaw   =  17  *  pi/180;              % between -90º  and  +90º

sy = sin(payload_yaw);      cy = cos(payload_yaw);
sp = sin(payload_pitch);    cp = cos(payload_pitch);

%                                   %      DCM 'xyz'
DCM_pl = [  cy,  sy*sp, -sy*cp;     %     [          cy*cz, sz*cx+sy*sx*cz, sz*sx-sy*cx*cz]
             0,     cp,     sp;     %     [         -cy*sz, cz*cx-sy*sx*sz, cz*sx+sy*cx*sz]
            sy, -cy*sp,  cy*cp];    %     [             sy,         -cy*sx,          cy*cx]

Speed_Vector = DCM_pl' * [0; 0; 100] ;

% Speed_Vector = [73.093; -86.126; 454.993]
Vmod = norm(Speed_Vector);

Vx = Speed_Vector(1) /Vmod ;
Vy = Speed_Vector(2) /Vmod;
Vz = Speed_Vector(3) /Vmod;


Speed_pitch = atan2(-Vy, Vz)    * 180/pi   % Must be between -180º and  +180º
Speed_yaw   =  asin(Vx )    * 180/pi    % Must be between -90º  and  +90º


%%
clc;

AoA_pitch = 17  *  pi/180;
AoA_yaw   = 13  *  pi/180;


sy = sin(AoA_yaw);
cy = cos(AoA_yaw);
sp = sin(AoA_pitch);
cp = cos(AoA_pitch);

DCM_AoA = [  cy,  sy*sp, -sy*cp;        % DCM XYZ
              0,     cp,     sp;
             sy, -cy*sp,  cy*cp];    

payload_pitch = 10  *  pi/180;
payload_yaw   = 30  *  pi/180;

sy = sin(payload_yaw);
cy = cos(payload_yaw);
sp = sin(payload_pitch);
cp = cos(payload_pitch);


DCM_pl = [  cy,  sy*sp, -sy*cp;        % DCM XYZ
             0,     cp,     sp;
            sy, -cy*sp,  cy*cp];


Speed_Vector = DCM_pl' * DCM_AoA' * [0; 0; 1];


V = DCM_pl * Speed_Vector;

Vx = V(1)/Vmod;
Vy = V(2)/Vmod;
Vz = V(3)/Vmod;

AoA_pitch = atan2(-Vy, Vz)    * 180/pi    % Must be between -180º and  +180º
AoA_yaw   =  asin(Vx)    * 180/pi         % Must be between  -90º and   +90º



%% Josef Ettl's Code
clc;
VV = [73.093; 86.126; 454.993];
% VV = [0 10 100]

VN = VV(1);
VE = VV(2);
VU = VV(3);

azi = atan2(-VE,VN);
ele = atan2(VU, sqrt(VE^2+VN^2));

azi2  = azi * 180/pi
ele2 = ele * 180/pi
% b = cos(ele);

x = cos(ele) * sin(azi)
y = cos(ele) * cos(azi)
z = sin(ele)

Speed_yaw = asin(y) * 180/pi
Speed_pitch = atan2(x,z) * 180/pi
