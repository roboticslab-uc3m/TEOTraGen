function [joint, n_joints] = humanoid_flex_ankle_DH_parameters(theta)
%   HUMANOID_FLEX_ANKLE_DH_PARAMETERS create the kinematic model for the
%   passive joints in a humanoid ankle
%   [joint, n_joints] = create_humanoid_flex_ankle (L, theta) creates a new
%   kinematic chain for the passive joints (roll and pitch) in the ankle of
%   a humanoid robot. The joints variables named by the vector theta.
%
%   See also ASSIGN_DH_PARAMETERS, HUMANOID_LEG_FLOATING_DH_PARAMETERS,
%   HUMANOID_LEG_STANDING_DH_PARAMETERS, GENERATE_HUMANOID_LEGS_KINEMATICS,
%   HUMANOID_TORSO_DH_PARAMETERS, HUMANOID_ARM_DH_PARAMETERS.

%   Author: Paolo Pierro
%   $Revision: 1.1 $  $Date: 2011/08/04 $

% Number of d.o.f.s
n_joints = 2;

% Define symbolic constants to avoid precision troubles
PI_2 = sym(pi/2);
ZERO = sym(0);

% Assign DH-Parameters
% Joint 1
jj = 1; 
joint(jj) = assign_DH_parameters(ZERO, ZERO, PI_2, theta(jj));

% Joint 2
jj = 2; 
joint(jj) = assign_DH_parameters(ZERO, ZERO, -PI_2, theta(jj)-PI_2);

% Define joints type and generalized variable name
for jj = 1:n_joints
    joint(jj).type =  'r';
    joint(jj).q    = theta(jj);
end
end