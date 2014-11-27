function support_foot = detect_support_foot (F_r, F_l)
%   DETECT_SUPPORT_FOOT estimates the support foot of a humanoid.
%   foot = detect_support_foot (F_r, F_l) estimates the support foot for a
%   humanoid robot. The estimation is based on the Z component of the
%   information coming from the force sensor. The output is a struct
%   providing the following fields:
%       SSF:    = -1 if the support is on right foot
%               =  1 if the support is on left foot
%       SF:     = -1 if the support is on right foot
%               =  1 if the support is on left foot
%               =  0 if there is double support (both force sensors give at least a 40% of the max value)
% 
%       PR:     the percentage of support on right foot
%       PL:     the percentage of support on left foot
%       var:    detects a change in the support foot
%
%   See also VNORM.

%   Author: Paolo Pierro
%   $Revision: 0.8 $  $Date: 2011/06/16 $

V = 0.3;
N_R = vnorm(F_r,1);
N_L = vnorm(F_l,1);
N = N_R + N_L;
P_R = N_R./N;
P_L = N_L./N;
support_foot.SSF = 2*(N_L > N_R) - 1;
support_foot.SF = (P_L > V) - (P_R > V);
support_foot.PR = P_R;
support_foot.PL = P_L;
support_foot.var = [0,abs(diff(support_foot.SSF))/2];