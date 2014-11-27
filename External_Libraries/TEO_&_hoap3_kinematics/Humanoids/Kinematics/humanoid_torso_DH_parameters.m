function [joint, n_joints] = humanoid_torso_DH_parameters(L, theta)
%   HUMANOID_TORSO_DH_PARAMETERS create the kinematic model of a humanoid
%   torso
%   [joint, n_joints] = create_humanoid_torso (L, theta) creates a new
%   kinematic chain for the torso of a humanoid robot. The length of the
%   links is defined by the vector L and the joints variables named by the
%   vector theta. It has to be used in order to provide the pose of the IMU
%   sensors (placed in the Center of Mass) with respect to the robot waist
%   (in the center).
%
%   See also ASSIGN_DH_PARAMETERS, GENERATE_HUMANOID_TORSO_KINEMATICS,
%   HUMANOID_LEG_FLOATING_DH_PARAMETERS,
%   HUMANOID_LEG_STANDING_DH_PARAMETERS, HUMANOID_ARM_DH_PARAMETERS.
%   

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
joint(jj) = assign_DH_parameters(ZERO, ZERO, -PI_2, theta(jj));

% Joint 2
jj = 2; 
joint(jj) = assign_DH_parameters(-L, ZERO, ZERO, theta(jj));

% Define joints type and generalized variable name
for jj = 1:n_joints
    joint(jj).type =  'r';
    joint(jj).q    = theta(jj);
end
end