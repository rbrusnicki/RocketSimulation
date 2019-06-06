function [] = DLR_PID()

    GRAD = 2;
    LAENGE = 20;
    PREDNUM = 25;
    D = 1.0;
    w0 = 30.0;
    D1 = 1.0;
    w1 = 2.0;
    max_x = 0;
    max_y = 0;

    % powfeld_A[3][3][20];        %preparation of arrays for LMS
    % powfeld_B[3][20];
    % feld[5][5];                 %matrix
    % pitch_buffer[20];           %for pitch prediction
    % yaw_buffer[20];             %for yaw prediction
    % pred_2_pitch;               %pitch result of prediction
    % pred_2_yaw;                 %yaw result of prediction

    % preset of filtering
    aus1 = 0;
    rate1 = 0;
    rate2 = 0;
    pitch_filtered = 0;

    % preset of filtering
    aus11 = 0;
    rate11 = 0;
    rate22 = 0;
    yaw_filtered = 0;

    % pitch_rate_old;
    % yaw_rate_old;
    % pitch_acc;
    % yaw_acc;
    % pitch_acc_filt;
    % yaw_acc_filt;
    % pitch_acc_rate;
    % yaw_acc_rate;
    % yaw_stream;
    % pitch_stream;
    % v_elevation;
    % v_azimuth;
    % max_x_filt,max_y_filt;
    % pitch_ar,yaw_ar;
    delta_yaw = 0;
    delta_pitch = 0;
    delta_pitchi = 0;
    delta_yawi = 0;
    pitch_rate = 0;
    yaw_rate = 0;
    delta_pitch_old = 0;
    delta_yaw_old = 0;

end

function [powfeld_A] = makepowfeld_A(GRAD, LAENGE)
    for i = 0:GRAD
        for j = 0:GRAD
          for l = 0:LAENGE-1
              powfeld_A(i+1, j+1, l+1) = l^( 2*GRAD - j - i );
          end
        end
    end
end

function [powfeld_B] = makepowfeld_B()
    for i = 0:GRAD
        for l = 0:LAENGE-1
            powfeld_B(i+1,l+1) = l^(GRAD - i);
        end
    end
end

%%
% Least means square function
% eingangswerte = pointer on input field
% grad, which kind of polenom (2nd,...3 rd order)
% laenge, length of field as input for LMS

function [] = LMS(eingangswerte, grad, laenge)

    for i = 0:grad
        for j = 0:(grad+1)
            feld(j+1, i+1) = 0;
        end
    end
    % k = 2 * grad;
    
    for i = 0:grad
        for j = 0:grad
            for l = 0:(laenge - 1) 
                feld(j+1, i+1) = feld(j+1,i+1) + powfeld_A(i+1, j+1, l+1);
            end
        end
    end

    for i = 0:grad
        for l = 0:(laenge - 1)
            feld(grad+2, i+1) = feld(grad+2, i+1) + eingangswerte(l+1) * powfeld_B(i+1, l+1);
        end
    end
    
  %solutin of the Matrix
    for spalte = 0:grad
        an = feld(spalte+1, spalte+1);

        for i = 0:(grad+1)
            feld(i+1, spalte+1) = feld(i+1, spalte+1)/an;
        end

        for zeile = (spalte + 1):grad
            an = feld(spalte + 1, zeile + 1);
            for i = 0:(grad+1)
                feld(i+1, zeile+1) = feld(i+1, zeile+1) - an * feld(i+1, spalte+1);
            end
        end

        for zeile = 0:(spalte-1)
            an = feld(spalte+1, zeile+1);
            for i = 0:(grad+1)
                feld(i+1, zeile+1) = feld(i+1, zeile+1) - an * feld(i+1, spalte+1);
            end
        end
    end 
end

% low pass 4 th order
% sample rate 100 Hz = 10 msec
% D is damping coefficient
% w0 corner frequency in rad/s
% aus is output
% rate is previous rate
% value=new input

function [aus] = Low_pass2_10msec(D, w0, aus, rate, value)
    rate = w0*w0 * ( value - aus) * 0.01 + rate;
    aus = aus + (rate - w0 * 2.0 * D * aus) * 0.01;
    %rückgabewert ist poijnter auf aus = 3. wert
end

%prediction by the LMS
function [] = prediction()

    struct DMARS_Data * dmarsDataPtr = &dmarsData;

    % wo wird rate1 + rate2 verwendet??? -> Filter ist unklar
    Low_pass2_10msec(D, w0, aus1, rate1, dmarsDataPtr.PitchAngle);      %filtern der Werte aus DMARS
    Low_pass2_10msec(D, w0, pitch_filtered, rate2, aus1);

    % wo wird rate11 und rate22 verwendet???
    Low_pass2_10msec(D, w0, aus11, rate11, dmarsDataPtr.YawAngle);
    Low_pass2_10msec(D, w0, yaw_filtered, rate22, aus11);

    for i = 0:18				%%buffer Arrays einmal weiterschieben
        pitch_buffer(i+1) = pitch_buffer(i + 2);
        yaw_buffer(i+1) = yaw_buffer(i + 2);
    end

    pitch_buffer(20) = pitch_filtered;     %%Werte aus Filter übernehmen
    yaw_buffer(20) = yaw_filtered;

    LMS (pitch_buffer, 2, 20);              		%Prediktor Array Pitch berechnen
    pred_2_pitch = feld(3,0) * PREDNUM * PREDNUM + feld(3,1) * PREDNUM + feld(3,2);    				% zukünftigen Wert berechnen
    LMS (yaw_buffer, 2, 20);				%Prediktor Array Yaw berechnen
    pred_2_yaw = feld(3,0) * PREDNUM * PREDNUM + feld(3,1) * PREDNUM + feld(3,2);

    %nach prediktor pred_2_pitc, pred_2_yaw

end

function [yaw_stream, pitch_stream] = azi_ele_euler( azi, ele )
    rad1  =  pi/ 180.0;
    z = sin(ele * rad1);
    b = cos(ele * rad1);
    y = b * cos(azi * rad1);
    x = b * sin(azi * rad1);
    yaw_stream = asin(y) / rad1;
    pitch_stream = atan2(x, z) / rad1;
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [] = attitudecontrol()

    corrfactor = 0;
    corr_angle = 0;
    %  N,Nh,x_coord,y_coord,radius_lat,v_erd_rot,ex;
    %  v_azimuth;
    %  v_elevation;

    rad1 = pi / 180.0;
%  struct DMARS_Data * dmarsDataPtr1 = &dmarsData;

    % Control parameters depending on mbeta
    kp = 5.1 / Mbeta;
    kd = kp;
    ki = kp / 2;

    if (kp > 1)
        kp = 1.0;
        kd = 1.0;
        ki = 0.2;
    end
    
    % calculation of the speed vector
    v_azimuth = atan2((dmarsDataPtr1.VelocityE), (dmarsDataPtr1.VelocityN)) / rad1 ;
    v_elevation = atan2(dmarsDataPtr1.VelocityUP, sqrt(-dmarsDataPtr1.VelocityE^2 + dmarsDataPtr1.VelocityN^2) ) / rad1 ;

    % Convert azimuth and elevation into Euler angles Pitch and Yaw
    [yaw_stream, pitch_stream] = azi_ele_euler(v_azimuth,v_elevation);               
    % the maximum correction angle to the current pitch and yaw angles are calculated depending on the allowable AoA

        if (liftoffcounter <= 15)               % the numbers indicate flight time in seconds
            corr_angle = 20;                    % maximo AoA permitido
        elseif (liftoffcounter > 15 && (liftoffcounter <= 20))
            corr_angle = 20;
        elseif (liftoffcounter > 20 && (liftoffcounter <= 25))
            corr_angle = 10;
        elseif (liftoffcounter > 25 && (liftoffcounter <= 45))
           corr_angle = 0;
        elseif (liftoffcounter > 45 && (liftoffcounter <= 75))
           corr_angle = 10;
        elseif (liftoffcounter > 75 && (liftoffcounter <= 85))
           corr_angle = 0;
        else
           corr_angle = 0;
        end

    % pitch_desiredare values coming from the nominal trajectory
    % yaw_desiredare values coming from the nominal trajectory

    % if pitch_desired and yaw_desired are out of the max. AoA range than pitch_desired and yaw_desired have to be corrected.
    
    if (abs(pitch_stream - pitch_desired) > corr_angle)
       if (pitch_stream > pitch_desired)
             pitch_desired = pitch_stream - corr_angle;
       elseif (pitch_stream < pitch_desired)
             pitch_desired = pitch_stream + corr_angle;
       end
    end
    
    if (abs(yaw_stream - yaw_desired) > corr_angle)
        if (yaw_stream > yaw_desired)
            yaw_desired = yaw_stream - corr_angle;
        elseif (yaw_stream < yaw_desired)
            yaw_desired = yaw_stream + corr_angle;
        end
    end
    
    if (liftoffcounter <= 1)
        pitch_desired = 0;
        yaw_desired = 0;
    end


    delta_pitch = pitch_desired - pred_2_pitch;                  % build the difference
    delta_pitchi = delta_pitchi + delta_pitch * 0.01;       % build the integral of the difference
    pitch_rate = (delta_pitch - delta_pitch_old) / 0.01;     % build the velocity of difference
    
    delta_yaw = yaw_desired - pred_2_yaw;
    delta_yawi = delta_yawi + delta_yaw * 0.01;
    yaw_rate = (delta_yaw - delta_yaw_old) / 0.01;

    delta_pitch_old = delta_pitch;
    delta_yaw_old = delta_yaw;

    max_x = delta_pitch * kp + kd * pitch_rate + delta_pitchi * ki; % calculate the nozzle deflection angle in DMARS reference system
    max_y = delta_yaw * kp + kd * yaw_rate + delta_yawi * ki;

    if ( liftoffcounter < 15 || ( liftoffcounter > 30 && liftoffcounter < 80 )  )
        if (Mbeta > 0)
            % for some time try to compensate the impact of malpha as an acceleraton offset
            max_x = max_x - (pred_2_pitch - pitch_stream) * Malpha / Mbeta;
            max_y = max_y - (pred_2_yaw - yaw_stream) * Malpha / Mbeta;
        else
            max_x=0;
            max_y=0;
        end
    end

    % the maximum nozzle deflection angle is 3 degrees
    corrfactor = sqrt( max_x*max_x + max_y*max_y ) / 3.0;
    if (corrfactor > 1.0)
        max_x = max_x / corrfactor;
        max_y = max_y / corrfactor;
    end

% filter the nozzle deflection angle in order to get rid of the Eigenfrequency oscillation
 Low_pass2_10msec(D, w0, max_x_filt, pitch_ar, max_x);
 Low_pass2_10msec(D, w0, max_y_filt, yaw_ar, max_y);

end

  %TVA_pitchr = max_x * cos(dmarsData.RollAngle * rad1) + max_y * sin(dmarsData.RollAngle * rad1);
  %TVA_yawr = max_y * cos(dmarsData.RollAngle * rad1) + max_x * sin(dmarsData.RollAngle * rad1);