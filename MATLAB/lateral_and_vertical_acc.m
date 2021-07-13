function [k_acc] = lateral_and_vertical_acc(q, acc, i, dt, FE_b, FG, Cnalfa, Cnbeta, Cd, Pdin, AoA, Sref, M, M_p, W_b, CoG, le )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% INCREASING ELEVATION
 %% New Quaternion 
    [p, y, r] = quat2angle(q, 'ZYZ');

    azi = -p;
    ele = pi/2 - y;
    til = r;

    azi_new = azi + 0 * pi/180;
    ele_new = ele + 1 * pi/180;
    
    if ele_new > pi/2
        ele_new = pi/2;
    end

    new_quat = angle2quat( -azi_new, pi/2-ele_new, til, 'ZYZ');
    
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
    D_NB = DCM_NRS_to_BRS( q );
    D_BN = D_NB';
    %payload_vector in NRS
    pv = D_BN(:,3); % == D_BN * [0; 0; 1];
    
    % acceleration to up direction
    up_acc = acc_new(:,3) - acc(:,3);

    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
% INCREASING AZIMUTH
%% New Quaternion 
    [p, y, r] = quat2angle(q, 'ZYZ');

    azi = -p;
    ele = pi/2 - y;
    til = r;

    azi_new = azi + 1 * pi/180;
    ele_new = ele + 0 * pi/180;

    new_quat = angle2quat( -azi_new, pi/2-ele_new, til, 'ZYZ');
    
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
    D_NB = DCM_NRS_to_BRS( q );
    D_BN = D_NB';
    %payload_vector in NRS
    pv = D_BN(:,3); % == D_BN * [0; 0; 1];
    
    % horizontal acceleration in NRS
    hor_acc = [acc_new(:,1); acc_new(:,2); 0] - [acc(:,1); acc(:,2); 0];
    
    % horizontal direction of payload vector
    hor_pv = [pv(1); pv(2); 0] / sqrt(pv(1)^2 + pv(2)^2); %normalized
    
    % acceleration in the payload vector direction
    pv_acc = dot(hor_acc,hor_pv) * hor_pv;
    
    % lateral acceleration
    lat_acc = hor_acc - pv_acc;
    
    u = cross(lat_acc, hor_pv); 
    lat_acc_mod = norm(lat_acc) * sign(u(3)); %positive sign means the rockets accelerates to the right
    
    k_acc = [up_acc, lat_acc_mod];
end