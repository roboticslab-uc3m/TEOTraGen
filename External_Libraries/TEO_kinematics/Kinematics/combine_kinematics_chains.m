function [T, J, H] = combine_kinematics_chains(kinematics)
%COMBINE_KINEMATICS_CHAINS combines different kinematics chains.
%
%   See also CREATE_HOMOGENEOUS_MATRIX.

%   Author: Paolo Pierro
%   $Revision: 1.0 $  $Date: 2011/06/09 $

n = size(kinematics,2);
m = size(kinematics{1},2);

if m < 2
    error('COMBINE_KINEMATICS_CHAINS:argChk', 'Wrong type of input arguments: each input should be at least a couple: a homogeneous transformation and a jacobian matrix')
end

switch n
    case 1
        if ~is_homogeneous_matrix(kinematics{1}{1})
            error('COMBINE_KINEMATICS_CHAINS:argChk', 'Wrong type of input arguments: each input should be a couple: a homogeneous transformation and a jacobian matrix')
        end
        T = kinematics{1}{1};
        J = kinematics{1}{2};
        if m == 3
            H = kinematics{1}{3};
        else
            H = [];
        end
    case 2
        if ~is_homogeneous_matrix(kinematics{1}{1}) || ~is_homogeneous_matrix(kinematics{2}{1})
            error('COMBINE_KINEMATICS_CHAINS:argChk', 'Wrong type of input arguments: each input should be a couple: a homogeneous transformation and a jacobian matrix')
        end
        R = kinematics{1}{1}(1:3,1:3) * kinematics{2}{1}(1:3,1:3);
        p = kinematics{1}{1}(1:3,4)   + kinematics{1}{1}(1:3,1:3) * kinematics{2}{1}(1:3,4);
        T = create_homogeneous_matrix(R, p);
        J = [kinematics{1}{2}, rotate_jacobian_matrix(kinematics{2}{2}, R)];
        if m == 3
            H = concatenate_hessians(kinematics{1}{3}, rotate_hessian_matrix(kinematics{2}{3}, R));
        else
            H = [];
        end
    otherwise
        [T_rem, J_rem, H_rem] = combine_kinematics_chains({kinematics{2:n}});
        R = kinematics{1}{1}(1:3,1:3) * T_rem(1:3,1:3);
        p = kinematics{1}{1}(1:3,4)   + T_rem(1:3,1:3) * T_rem(1:3,4);
        T = create_homogeneous_matrix(R, p);
        J = [kinematics{1}{2}, rotate_jacobian_matrix(J_rem, R)];
        if m == 3
            H = concatenate_hessians(kinematics{1}{3}, rotate_hessian_matrix(H_rem, R));
        else
            H = [];
        end
end
end

function C = concatenate_hessians(A, B)
[m_a, n1_a, n2_a] = size(A,1);
[m_b, n1_b, n2_b] = size(A,1);
C = sym(zeros(m_a, n1_a+n1_b, n2_a+n2_b));
for jj=1:m_a
    C(jj, :, :) = [A(jj, :, :), B(jj, :, :)];
end
end