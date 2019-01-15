function [ang_acc_b] = Angular_Aceleration_in_DLR_Body_Reference_System(I, W_b, D_NB, MCo, MFE, MFA, MA_f, MA_d )
% Computes angular aceleration in DLR Body Reference System
%
% INPUTS:
% I:    Inertial Matrix (only diagonal components different of zero).           [-]
% W_b:  Angular Velocity in DLR Body Reference System                           [rad/s]
% D_NB: DCM matrix that converts from Navigation Ref. Sys. to Body Ref. Sys.    [-]
% MCo:  Coriolis Moment in DLR Navigation Reference System                      [N.m]
% MFE:  Thrust Moment in DLR Navigation Reference System                        [N.m]
% MFA:  Aerodynamic Moment in DLR Navigation Reference System                   [N.m]
% MA_f: Aerodynamic Moment due to fins misaligment in DLR Nav. Ref. Sys.        [N.m]
% MA_d: Aerodynamic Damping Momentin DLR Navigation Reference System            [N.m]
%
% OUTPUT:
% ang_acc_b: Angular aceleration in DLR Body Reference System                   [rad/s^2]
%
%
% Author: Roberto Brusnicki - 05/01/2019


Mextra_b = cross(W_b', I*W_b')'; %BRS

Mextra = ( D_NB' * Mextra_b' )'; %NRS

I_times_ang_acc = ( 0*MCo' + MFE' + MFA' - Mextra' + MA_f' + MA_d' );  %NRS   

ang_acc_b = I\( D_NB*I_times_ang_acc );  % BRS
        
end