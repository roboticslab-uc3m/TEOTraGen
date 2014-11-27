function J_dynamics = evaluate_dynamics_jacobians (P, robot)
%EVALUATE_DYNAMICS_JACOBIANS determines the Jacobians necessary for the
%dynamics model computation
%
%   See also EVALUATE_GEOMETRIC_JACOBIAN.

%   Author: Paolo Pierro
%   $Revision: 1.1 $  $Date: 2011/08/04 $

% Determine the number of information
motor_info = isfield(robot.joint, 'mass');
link_info  = isfield(robot.link,  'mass');
J_dynamics = [];

% Components to take into account from the transformation matrices (x, y, z)
components = 1:3;

% Versor z of every reference system
z = P(components, 3, 1:robot.n_joints);

% Position of the origin of every reference system
p = P(components, 4, 1:robot.n_joints);

if link_info
    % Include links mass and inertia contribution
    
    % Initialize matrices for increasing speed
    Jp_l = sym(zeros(3, robot.n_joints, robot.n_joints));
    Jo_l = sym(zeros(3, robot.n_joints, robot.n_joints));
    
    for ii = 1:robot.n_joints
        for jj = 1:ii
            % contribution to the ii-th joint velocity of the jj-th link
            [Jp_l(components, jj, ii), Jo_l(components, jj, ii)] = jacobian_component (z(:,jj), (robot.link(ii).CoM - p(:, jj)), robot.joint(jj).type);
        end
    end
    J_dynamics.Jp_l = Jp_l;
    J_dynamics.Jo_l = Jo_l;
end

if motor_info
    % Include motors mass and inertia contribution
    
    % Initialize matrices for increasing speed
    Jp_m = sym(zeros(3, robot.n_joints, robot.n_joints));
    Jo_m = sym(zeros(3, robot.n_joints, robot.n_joints));
    for ii = 1:robot.n_joints
        for jj = 1:ii
            % contribution to the ii-th joint velocity of the jj-th motor
            Jp_m(components, jj, ii) = jacobian_component (z(:,jj), (p(:, ii) - p(:, jj)), robot.joint(jj).type);
            if jj < ii
                Jo_m(components, jj, ii) = Jo_l(components, jj, ii);
            else
                Jo_m(components, jj, ii) = robot.joint(ii).kr * z(:, ii);
            end
        end
    end
    J_dynamics.Jp_m = Jp_m;
    J_dynamics.Jo_m = Jo_m;
end
end

%%%%%%%%%%%%%%%%%%%%%%%
% Auxiliary functions %
%%%%%%%%%%%%%%%%%%%%%%%

% Calculate the contribution to the end-effector linear and angular speed of
% the joint specified as input (it works only with revolute  and prismatic joints).
function [jp, jo] = jacobian_component (z, p, type)
% Number of attempts of the simplify function
sim_iterations = 10;

switch type
    case 'p'
          % prismatic joint
          jp = simplify_symbolic_matrix(z, 4, sim_iterations);
          jo = sym(zeros(3, 1));
          
    case 'r'
          % revolute joint
          jp = simplify_symbolic_matrix(cross(z, p), 6, sim_iterations);
          jo = simplify_symbolic_matrix(z, 4, sim_iterations);
end
end