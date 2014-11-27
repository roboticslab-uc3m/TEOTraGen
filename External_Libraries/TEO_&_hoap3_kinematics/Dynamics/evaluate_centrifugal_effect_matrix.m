function C = evaluate_centrifugal_effect_matrix (B, robot)
%EVALUATE_CENTRIFUGAL_EFFECT_MATRIX determines the Matrix taking into
%account the effects of quadratic velocity terms of a robot (both
%centrifugal and Coriolis effects)
%
%   See also EVALUATE_ACCELERATION_EFFECT_MATRIX, EVALUATE_DYNAMICS_JACOBIANS.
 
%   Author: Paolo Pierro
%   $Revision: 1.0 $  $Date: 2011/07/31 $

% Number of attempts of the simplify function
sim_iterations = 20;

% Initialize matrices for increasing speed
C_ijk = sym(zeros(robot.n_joints, robot.n_joints, robot.n_joints));
C     = sym(zeros(robot.n_joints));

% Evaluate Christoffel symbols of the first type
for ii = 1:robot.n_joints
    for jj = 1:robot.n_joints
        for kk = 1:jj
            A = 0.5 * (diff(B(ii, jj), robot.q(kk)) + diff(B(ii, kk), robot.q(jj)) - diff(B(jj, kk), robot.q(ii)));
            C_ijk(ii, jj, kk) = A * robot.dq(kk); 
            if jj > kk
                C_ijk(ii, kk, jj) = A * robot.dq(jj);
            end
        end
    end
end

% Determine matrix C
for ii = 1:robot.n_joints
    for jj = 1:robot.n_joints
        C(ii, jj) = simplify_symbolic_matrix(sum(reshape(C_ijk(ii, jj, :), 1, robot.n_joints)), 6, sim_iterations);
    end
end
end