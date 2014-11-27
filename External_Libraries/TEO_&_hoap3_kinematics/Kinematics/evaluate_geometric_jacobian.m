function J = evaluate_geometric_jacobian (P, robot)
%EVALUATE_GEOMETRIC_JACOBIAN computes the differential kinematics for the
%manipulator defined by the array of transformation matrixes P whose
%definition is given in the function FORWARD_KINEMATICS
%
%   J = evaluate_geometric_jacobian (P, robot) generates one matrix 6 * n,
%   where n is the number of degrees of freedom of the manipulator defined
%   by robot.n_joints. The J matrix represents the geometric Jacobian of
%   the robotic manipulator defined in input. All the joints are supposed
%   to be revolute or prismatic (defined by robot.joint.type)
%
%   See also FORWARD_KINEMATICS.
 
%   Author: Paolo Pierro
%   $Revision: 3.2 $  $Date: 2011/08/05 $

% Initialize matrices for increasing speed
J_p = sym(zeros(3, robot.n_joints));
J_o = sym(zeros(3, robot.n_joints));

% Components to take into account from the transformation matrices (x, y, z)
components = 1:3;

% Evaluate the position of the end-effector
p_ee = P(components, 4, robot.n_joints + 1);

% Versor z of every reference system
z = P(components, 3, 1:robot.n_joints);

% Position of the origin of every reference system
p = P(components, 4, 1:robot.n_joints);

for jj = 1:robot.n_joints
    % contribution to the end-effector velocity of the jj-th joint
    [J_p(components, jj), J_o(components, jj)] = jacobian_component (z(:, jj), (p_ee - p(:,jj)), robot.joint(jj).type);
end

% Geometric Jacobian
J = [J_p; J_o];
end

%%%%%%%%%%%%%%%%%%%%%%%
% Auxiliary functions %
%%%%%%%%%%%%%%%%%%%%%%%

function [jp, jo] = jacobian_component (z, p, type)
% Calculate the contribution to the end-effector linear and angular speed of
% the joint specified as input (it works only with revolute and prismatic joints).
% Number of attempts of the simplify function
sim_iterations = 20;

switch type
    case 'p'
          % prismatic joint
          jp = simplify_symbolic_matrix(z, 6, sim_iterations);
          jo = sym(zeros(3, 1));
          
    case 'r'
          % revolute joint
          jp = simplify_symbolic_matrix(cross(z, p), 6, sim_iterations);
          jo = simplify_symbolic_matrix(z, 6, sim_iterations);
end
end