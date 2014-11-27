function [joint, n_joints] = humanoid_leg_standing_DH_parameters(L, theta)
%   HUMANOID_LEG_STANDING_DH_PARAMETERS create the kinematic model of
%   humanoid standing leg
%   [joint, n_joints] = humanoid_leg_standing_DH_parameters (L, theta) creates a new
%   humanoid robot leg model where the length of the links is defined by
%   the vector L and the joints variables named by the vector theta. The
%   model has to be used in standing phase. It has to be used in order to
%   generate the kinematics of the robot base (in the center of the waist)
%   with respect to the foot.
%
%   See also ASSIGN_DH_PARAMETERS, HUMANOID_TORSO_DH_PARAMETERS,
%   HUMANOID_LEG_FLOATING_DH_PARAMETERS, GENERATE_HUMANOID_LEGS_KINEMATICS,
%   HUMANOID_TORSO_DH_PARAMETERS, HUMANOID_ARM_DH_PARAMETERS.

%   Author: Paolo Pierro
%   $Revision: 1.1 $  $Date: 2011/08/04 $

% Number of d.o.f.s
n_joints = 6;

% Define symbolic constants to avoid precision troubles
PI_2 = sym(pi/2);
ZERO = sym(0);

% Assign DH-Parameters
% Joint 1
jj = 1; joint(jj) = assign_DH_parameters(ZERO, ZERO, PI_2, theta(jj));

% Joint 2
jj = 2; joint(jj) = assign_DH_parameters(L(2), L(1), ZERO, theta(jj));

% Joint 3
jj = 3; joint(jj) = assign_DH_parameters(L(3), ZERO, ZERO, theta(jj));

% Joint 4
jj = 4; joint(jj) = assign_DH_parameters(ZERO, ZERO, -PI_2, theta(jj));

% Joint 5
jj = 5; joint(jj) = assign_DH_parameters(ZERO, ZERO, -PI_2, theta(jj)+PI_2);

% Joint 6
jj = 6; joint(jj) = assign_DH_parameters(L(4), ZERO, ZERO, theta(jj));

% Define joints type and generalized variable name
for jj = 1:n_joints
    joint(jj).type =  'r';
    joint(jj).q    = theta(jj);
end
end