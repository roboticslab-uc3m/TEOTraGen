function B = evaluate_acceleration_effect_matrix(J_dynamics, robot)
%EVALUATE_ACCELERATION_EFFECT_MATRIX determines the Inertia Matrix for a given
%robot and dynamics Jacobians J_dynamics
%  The coefficient B(i,i) represents the moment of inertia at Joint i axis,
%  in the current manipulator configuration, when the other joints are
%  blocked.
%  The coefficient B(i,j) accounts for the effect of acceleration of Joint
%  j on Joint j.
%
%   See also EVALUATE_DYNAMICS_JACOBIANS, FORWARD_KINEMATICS.
 
%   Author: Paolo Pierro
%   $Revision: 1.1 $  $Date: 2011/08/04 $

% Number of attempts of the simplify function
sim_iterations = 20;

% Determine the number of information
motor_info = isfield(robot.joint, 'mass');
link_info  = isfield(robot.link,  'mass');

% Initialize matrices for increasing speed
B_ii = sym(zeros(robot.n_joints, robot.n_joints, robot.n_joints));
B    = sym(zeros(robot.n_joints));

for ii = 1:robot.n_joints
	if link_info
        % Include links mass and inertia contribution
        B_ii(:, :, ii) =    robot.link(ii).mass * transpose(J_dynamics.Jp_l(:, :, ii)) * J_dynamics.Jp_l(:, :, ii) +...
                            transpose(J_dynamics.Jo_l(:, :, ii)) * robot.link(ii).inertia * J_dynamics.Jo_l(:, :, ii);
    end
    if motor_info
        % Include motors mass and inertia contribution
        B_ii(:, :, ii) = B_ii(:, :, ii) +   robot.joint(ii).mass * transpose(J_dynamics.Jp_m(:, :, ii)) * J_dynamics.Jp_m(:, :, ii) + ...
                                            transpose(J_dynamics.Jo_m(:, :, ii)) * robot.joint(ii).inertia * J_dynamics.Jo_m(:, :, ii);
    end
end

for jj = 1:robot.n_joints
    for kk = 1:jj
        B(jj, kk) = simplify_symbolic_matrix(sum(reshape(B_ii(jj, kk, :), 1, robot.n_joints)), 4, sim_iterations);
        if jj > kk
            % Inertia Matrix is symmetrical
            B(kk, jj) = B(jj, kk);
        end
    end
end
end