function [A, T, P] = forward_kinematics (robot)
%FORWARD_KINEMATICS computes the forward kinematics using the
%Denavit-Hartenberg method
%
%   [A, T] = FORWARD_KINEMATICS (robot) generates one vector of matrixes A,
%   in which the A(:, :, jj) represents the transformation matrix from the
%   reference system jj-1 to the system jj and one matrix T which is the
%   transformation matrix from the reference origin to the end-effector
%   reference. The parameter robot should be defined specifying the
%   Denavit-Hartenberg parameters (d, alpha and theta) and the number of
%   joints n.
%
%   See also CREATE_MANIPULATOR.

%   Author: Paolo Pierro
%   $Revision: 3.0 $  $Date: 2011/08/03 $

% Determine the transformation matrices from the systems coordinates jj to jj-1
A = get_transformation_matrices ([robot.joint.a], [robot.joint.d], [robot.joint.alpha], [robot.joint.theta]);

% Evaluate trasformation matrices of every reference system w.r.t. the
% origin (starting from the second)
P1 = cumprod_symbolic (A, robot.n_joints);

% Extract the transformation matrix from the origin up to the end-effector
T = P1(:, :, robot.n_joints);

% Evaluate trasformation matrices of every reference system w.r.t. the
% origin (starting from the first)
P(:, :, 1) = sym(eye(4));
P(:, :, 2:robot.n_joints + 1) = P1;
end

%%%%%%%%%%%%%%%%%%%%%%%
% Auxiliary functions %
%%%%%%%%%%%%%%%%%%%%%%%

function A = get_transformation_matrices (a, d, alpha, theta)
% Evaluate the transformation matrices from Frame jj to Frame jj-1 defined
% by the input Denavit–Hartenberg parameters for the n joints

% Number of attempts of the simplify function
sim_iterations = 2;

% Number of joints
n = size(a, 2);

% Get common values for increasing speed
ZERO    = zeros(1, n);
ONE     = ones(1, n);
c_alpha = cos(alpha);
s_alpha = sin(alpha);
c_theta = simplify(cos(theta), sim_iterations);
s_theta = simplify(sin(theta), sim_iterations);

A(1, :, :) = reshape([c_theta; -c_alpha.*s_theta;  s_alpha.*s_theta; a.*c_theta], 1, 4, n);
A(2, :, :) = reshape([s_theta;  c_alpha.*c_theta; -s_alpha.*c_theta; a.*s_theta], 1, 4, n);
A(3, :, :) = reshape([ZERO;              s_alpha;           c_alpha;          d], 1, 4, n);
A(4, :, :) = reshape([ZERO;                 ZERO;              ZERO;        ONE], 1, 4, n);
end

function P = cumprod_symbolic (A, n)
% Returns the cumulative product along the third dimension of an array of
% matrixes A. The matrix P(:,:,jj) represents the cumulative product
% of the A matrix until jj

% Number of attempts of the simplify function
sim_iterations = 20;

% Initialize matrix for increasing speed
P = sym(zeros(4, 4, n));

P(:,:,1) = A(:,:,1);
for ii = 2:n
    P(:,:,ii) = simplify_symbolic_matrix(P(:,:,ii-1) * A(:,:,ii), 6, sim_iterations);
end
end