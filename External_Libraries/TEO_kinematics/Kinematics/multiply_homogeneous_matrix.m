function T_multiplied = multiply_homogeneous_matrix(T)
%MULTIPLY_HOMOGENEOUS_MATRIX multiplies two homogeneous transformation matrix.
%
%   See also CREATE_HOMOGENEOUS_MATRIX.

%   Author: Paolo Pierro
%   $Revision: 1.0 $  $Date: 2011/04/04 $

n = size(T,2);

switch n
    case 1
        if ~is_homogeneous_matrix(T{1})
            error('MULTIPLY_HOMOGENEOUS_MATRIX:argChk', 'Wrong type of input arguments: the inputs must be homogeneous transformation matrices')
        end
        T_multiplied = T{1};
    case 2
        if ~is_homogeneous_matrix(T{1}) || ~is_homogeneous_matrix(T{2})
            error('MULTIPLY_HOMOGENEOUS_MATRIX:argChk', 'Wrong type of input arguments: the inputs must be homogeneous transformation matrices')
        end
        R = T{1}(1:3,1:3) * T{2}(1:3,1:3);
        p = T{1}(1:3,4)   + T{1}(1:3,1:3) * T{2}(1:3,4);
        T_multiplied = create_homogeneous_matrix(R, p);
    otherwise
        T_rem = multiply_homogeneous_matrix({T{2:n}});
        R = T{1}(1:3,1:3) * T_rem(1:3,1:3);
        p = T{1}(1:3,4)   + T{1}(1:3,1:3) * T_rem(1:3,4);
        T_multiplied = create_homogeneous_matrix(R, p);
end
end