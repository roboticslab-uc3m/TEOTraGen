function H = evaluate_geometric_hessian (P, robot)
%EVALUATE_GEOMETRIC_HESSIAN computes the differential kinematics of 2nd
%order for the manipulator defined by the array of transformation matrixes
%P whose definition is given in the function FORWARD_KINEMATICS
%
%   H =  evaluate_geometric_hessian(P, robot) generates one tensor 6 * n *
%   n, where n is the number of degrees of freedom of the manipulator
%   defined by robot.n_joints. The H matrix represents the geometric
%   Hessian of the robotic manipulator defined in input. All the joints are
%   supposed to be revolute or prismatic (defined by robot.joint.type)
%
%   See also FORWARD_KINEMATICS, EVALUATE_GEOMETRIC_JACOBIAN.
%
%   WORKS ONLY WITH ROTOIDAL: CHECK PAPER The Kinematic Hessian and Higher Derivatives. Arang Hourtash
 
%   Author: Paolo Pierro
%   $Revision: 0.3 $  $Date: 2011/08/05 $

% Initialize the matrix for increasing speed
H_p = sym(zeros(3, robot.n_joints, robot.n_joints));
H_o = H_p;

% Components to take into account from the transformation matrices (x, y, z)
components = 1:3;

% Evaluate the position of the end-effector
p_ee = P(components, 4, robot.n_joints + 1);

% Versor z of every reference system
z = P(components, 3, 1:robot.n_joints);

% Position of the origin of every reference system
p = P(components, 4, 1:robot.n_joints);

for ii = 1:robot.n_joints
    for jj = 1:ii
        % contribution to the end-effector acceleration of the ii-th and jj-th joint
        [H_p(components, ii, jj), H_o(components, ii, jj)] = hessian_component (z(:,ii), z(:,jj), (p_ee - p(:, ii)), robot.joint(ii).type, robot.joint(jj).type);
        if ii > jj
            H_p(components, jj, ii) = H_p(components, ii, jj);
            H_o(components, jj, ii) = H_o(components, ii, jj);
        end
    end
end
% Geometric Jacobian
H(1:3,:,:) = H_p;
H(4:6,:,:) = H_o;

%%%%%%%%%%%%%%%%%%%%%%%
% Auxiliary functions %
%%%%%%%%%%%%%%%%%%%%%%%

% Calculate the contribution to the end-effector linear and angular speed of
% the joint specified as input (it works only with revolute  and prismatic joints).
function [hp, ho] = hessian_component (z_i, z_j, p, type_i, type_j)
type = strcmpi(type_i, 'r') + strcmpi(type_j, 'r');
switch type
    case 0
            %    to be checked!!!
          % both prismatic joints
          hp = sym(zeros(3,1));
          ho = sym(zeros(3,1));
          
    case 1
            %    to be checked!!!
          % revolute and prismatic joint
          hp = cross(z_i, z_j);
          ho = sym(zeros(3,1));
          
    case 2
          % both revolute joints
          hp = cross(z_i, cross(z_j, p));
          ho = cross(z_i, z_j);
end