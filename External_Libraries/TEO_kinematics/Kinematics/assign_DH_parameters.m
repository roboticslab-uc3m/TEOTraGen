function joint = assign_DH_parameters (a, d, alpha, theta)
% ASSIGN_DH_PARAMETERS assigns to a joint the corresponding
% Denavit-Hartenberg parameters a, d, alpha, theta.

%   Author: Paolo Pierro
%   $Revision: 1.1 $  $Date: 2011/08/04 $

joint.a     = a;
joint.d     = d;
joint.alpha = alpha;
joint.theta = theta;
end