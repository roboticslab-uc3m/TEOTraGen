function [B, C, g] = evaluate_dynamic_model(J_dynamics, robot)
%EVALUATE_DYNAMIC_MODEL computes the dynamic model for simple manipulator
%structures defined by a structure robot, and by the Jacobians for the
%dynamics J_dynamics.
%
%   See also EVALUATE_DYNAMICS_JACOBIANS,
%   EVALUATE_ACCELERATION_EFFECT_MATRIX,
%   EVALUATE_CENTRIFUGAL_EFFECT_MATRIX, EVALUATE_GRAVITY_EFFECT_VECTOR.
 
%   Author: Paolo Pierro
%   $Revision: 1.1 $  $Date: 2011/08/04 $

% Acceleration effects
B = evaluate_acceleration_effect_matrix(J_dynamics, robot);
disp('B calculated');

% Velocities effects
C = evaluate_centrifugal_effect_matrix (B, robot);
disp('C calculated');

% Gravity effects
g = evaluate_gravity_effect_vector(J_dynamics, robot);
disp('g calculated');
end