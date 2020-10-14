function [D_NB] = DCM_NRS_to_BRS(q)
% Computes the DCM matrix as a function of the attitude quaternion.
%
% INPUT:
%   q: quaternion               [-]
%
% OUTPUT:
%   D_NB: DCM matrix that converts from Navigation reference system to Body reference system    [-]
%
% Author: Roberto Brusnicki - 15/01/2019


q0 = q(1);  % escalar component
q1 = q(2);  % ex
q2 = q(3);  % ey
q3 = q(4);  % ez

 % 'Rotates' from Navigation Reference System to Body Reference System
%If r1 is a vector writen in Nav. Ref. Sys., then D_NB*r1 is the same vector writen in Body Ref. Sys.
%If r2 is a vector writen in Body Ref. Sys., then D_BN*r2 is the same vector writen in Nav. Ref. Sys.
D_NB = [    1-2*q2^2-2*q3^2      2*(q1*q2+q0*q3)     2*(q1*q3-q0*q2);
            2*(q1*q2-q0*q3)      1-2*q1^2-2*q3^2     2*(q2*q3+q0*q1);
            2*(q1*q3+q0*q2)      2*(q2*q3-q0*q1)     1-2*q1^2-2*q2^2];