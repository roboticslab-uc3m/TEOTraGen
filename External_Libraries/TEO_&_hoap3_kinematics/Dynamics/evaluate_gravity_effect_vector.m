function g = evaluate_gravity_effect_vector(J_dynamics, robot)
%EVALUATE_GRAVITY_EFFECT_VECTOR determines the vector taking into account
%gravitational effects.
%  The term g(i) represents the moment generated at Joint i axis of the
%  manipulator, in the current configuration, by the presence of gravity.
%
%   See also EVALUATE_DYNAMICS_JACOBIANS.
 
%   Author: Paolo Pierro
%   $Revision: 1.0 $  $Date: 2011/07/31 $

% Determine the n. of degrees of freedom
n = size(J_dynamics.Jp_l, 2);

% Initialize matrices for increasing speed
g_jj = sym(zeros(3, n));
g    = sym(zeros(n, 1));

if isfield(J_dynamics, 'Jp_m')
    % complete model
    for ii = 1:n
        for jj = 1:n
            g_jj(ii, jj) =  robot.link(jj).mass * transpose(robot.g0) * J_dynamics.Jp_l(:, ii, jj) +...
                            robot.joint(jj).mass * transpose(robot.g0) * J_dynamics.Jp_m(:, ii, jj);
        end
        g(ii) = -simplify_symbolic_matrix(sum(g_jj(ii, :)));
    end
else
    for ii = 1:n
        for jj = 1:n
            g_jj(ii, jj) =  robot.link(jj).mass * transpose(robot.g0) * J_dynamics.Jp_l(:, ii, jj);
        end
        g(ii) = -simplify_symbolic_matrix(sum(g_jj(ii, :)));
    end
end
end