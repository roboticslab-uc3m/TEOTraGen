function dynamics_parameters = extract_dynamic_parameters(w_R_F0, P, q, dq, robot_structure)
% EXTRACT_DYNAMIC_PARAMETERS extract the dynamic parameters for a robot
% defined by its structure robot_structure.
% 
%   dynamics_parameters = extract_dynamic_parameters (w_R_F0, P, q, dq,
%   robot_structure) extracts the dynamic parameters needed for computing
%   the dynamic model. The input w_R_F0 is the rotation matrix of the first
%   reference frame w.r.t. the world frame. P is the 4 by 4 by n tensor
%   expressing the transformation matrices of every joint with respect to
%   the base frame. The input q and dq are the generalized coordinates
%   array and its time derivative.
%
%   See also EVALUATE_GEOMETRIC_JACOBIAN.

%   Author: Paolo Pierro
%   $Revision: 1.1 $  $Date: 2011/08/06 $

% Number of joints
n_joints = size(robot_structure.joint, 2);
dynamics_parameters.n_joints = n_joints;

% Generalized variables
dynamics_parameters.q = reshape(q, n_joints, 1);
dynamics_parameters.dq = reshape(dq, n_joints, 1);

% Determine the rotation matrix from base to link jj
T_link = link_transformation_matrix(P, q, n_joints);

% Determine the number of information
link_info  = isfield(robot_structure.link,  'mass');

if link_info
    % Include links mass and inertia information
    
    % Preallocate array of structures
    dynamics_parameters.link = struct('mass', num2cell(zeros(n_joints, 1)), 'CoM', zeros(3,1), 'inertia', eye(3));
    for jj = 1:n_joints
        dynamics_parameters.link(jj).mass = robot_structure.link(jj).mass;
        % Position of the CoM of every link w.r.t. the base frame.
        dynamics_parameters.link(jj).CoM = vector_coordinate_transform(T_link(:, :, jj), robot_structure.link(jj).CoM);
        % Inertia Matrix of every link w.r.t. the base frame.
        dynamics_parameters.link(jj).inertia = rotate_inertia_matrix(robot_structure.link(jj).inertia, T_link(1:3, 1:3, jj));
    end
end

dynamics_parameters.joint = struct('type', num2cell(zeros(n_joints, 1)));
for jj = 1:n_joints
    dynamics_parameters.joint(jj).type = robot_structure.joint(jj).type;
end
if isfield(robot_structure.joint, 'kr')
    % Preallocate array of structures
    dynamics_parameters.joint(n_joints).kr = 1;
    for jj = 1:n_joints
        dynamics_parameters.joint(jj).kr = robot_structure.joint(jj).kr;
    end
end
if isfield(robot_structure.joint, 'mass')
    % Preallocate array of structures
    dynamics_parameters.joint(n_joints).mass = 0;
    for jj = 1:n_joints
        dynamics_parameters.joint(jj).mass = robot_structure.joint(jj).mass;
    end
end
if isfield(robot_structure.joint, 'inertia')
    % Preallocate array of structures
    dynamics_parameters.joint(n_joints).inertia = eye(3);
    for jj = 1:n_joints
        dynamics_parameters.joint(jj).inertia = rotate_inertia_matrix(robot_structure.joint(jj).inertia, T_link(1:3, 1:3, jj));
    end
end

% Extract gravity information
dynamics_parameters.g0 = transpose(w_R_F0) * robot_structure.g0;
end

function T_link = link_transformation_matrix(P, q, n)
% Initialize matrix for increasing speed
T_link  = sym(zeros(4, 4, n));

% Rotation Matrix of every joint
T_z = rotation_around_z (q);

for jj = 1:n
    % Transformation matrix from link jj to base
    T_link(:,:, jj) = multiply_homogeneous_matrix({P(:,:, jj), T_z(:, :, jj)});
end
end

function T = rotation_around_z (theta)
% Evaluate the transformation matrix corresponding to rotation around z
% axis of the frame jj. The rotation angle is defined by the theta

% Number of joints
n = length(theta);

% Convert input to a row array
theta = reshape(theta, 1, n);

% Get common values for increasing speed
ZERO    = zeros(1, n);
ONE     = ones(1, n);
c_theta = cos(theta);
s_theta = sin(theta);

T(1, :, :) = reshape([c_theta; -s_theta; ZERO; ZERO], 1, 4, n);
T(2, :, :) = reshape([s_theta;  c_theta; ZERO; ZERO], 1, 4, n);
T(3, :, :) = reshape([   ZERO;  ZERO;     ONE; ZERO], 1, 4, n);
T(4, :, :) = reshape([   ZERO;  ZERO;    ZERO;  ONE], 1, 4, n);
end