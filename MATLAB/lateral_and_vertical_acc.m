function [k_acc] = lateral_and_vertical_acc(q, acc, i, dt, FE_b, FG, Cnalfa, Cnbeta, Cd, Pdin, AoA, Sref, M, M_p, W_b, CoG, le )

 %% New Quaternion 
    % azimute e elevação iniciais
    D_BN = DCM_NRS_to_BRS( q )';
    pv = D_BN(:,3); % == D_BN * [0; 0; 1]
    azi_0 =  atan2(-pv(2),pv(1));
    ele_0 = atan2(pv(3), sqrt(pv(1)^2+pv(2)^2));

    %alteração em azimute e elevação
    azi = 1 *pi/180;  
    ele = 1 *pi/180;
    new_azi = azi_0 + azi;
    new_ele = ele_0 + ele;
    
    if new_ele > pi/2
        new_ele = pi/2;
    end
    
    new_quat = angle2quat( -new_azi , pi/2 - new_ele, 0, 'ZYX');
    [new_pitch, new_yaw, ~] = quat2angle(new_quat, 'XYZ');
    [~, ~, roll] = quat2angle(q, 'XYZ');
    new_quat = angle2quat(new_pitch, new_yaw, roll, 'XYZ');

    
 %% DCM
    % Direct Cossine Matrix
    D_NB = DCM_NRS_to_BRS( new_quat );

 %% forces
    % Thrust Force 
    FE = (D_NB' * FE_b')';
    % Força Aerodinâmica 
    FA = Aerodynamic_Force_in_DLR_NRS(new_quat, Cnalfa, Cnbeta, Cd, Pdin, AoA, Sref);
    % Força de Coriolis     
    FCo = Coriolis_Force_in_DLR_NRS(D_NB, M_p, W_b, CoG, le);

 %% acc
    % In DLR Navigation Reference System
    if norm(FE) < norm(FG) && i*dt < 10 
        acc_new = [0, 0, 0];
    else
        acc_new = ( FG + FA + FE + FCo ) / M;
    end
    
%% transform
    D_BN = D_NB';
    %payload_vector in NRS
    pv = D_BN(:,3); % == D_BN * [0; 0; 1];
    
    % acceleration to up direction
    up_acc = acc_new(:,3) - acc(:,3);
    
    % horizontal acceleration in NRS
    hor_acc = [acc_new(:,1); acc_new(:,2); 0];
    
    % horizontal direction of payload vector
    hor_pv = [pv(1); pv(2); 0] / sqrt(pv(1)^2 + pv(2)^2); %normalized
    
    % acceleration in the payload vector direction
    pv_acc = dot(hor_acc,hor_pv) * hor_pv;
    
    % lateral acceleration
    lat_acc = hor_acc - pv_acc;
    
    u = cross(hor_acc,pv_acc);
    lat_acc_mod = norm(lat_acc) * sign(u(3));
    
    k_acc = [up_acc, lat_acc_mod];

end