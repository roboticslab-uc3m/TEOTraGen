function create_matrix_function (M, function_name, joint_var)
%   CREATE_MATRIX_FUNCTION generate a MATLAB file function from a
%   symbolic matrix
%
%   create_matrix_function(J, 'calculate_jacobian', q) generates the MATLAB
%   function 'calculate_jacobian' which calculates the jacobian of a
%   manipulator specified by the matrix J. The joint variables are defined
%   by the vector q.
  
%   Author: Paolo Pierro
%   $Revision: 1.0 $  $Date: 2011/06/09 $
n = numel(joint_var);
if n >1
    matlabFunction(M, 'file', function_name, 'vars', {joint_var});
else
    matlabFunction(M, 'file', function_name, 'vars', joint_var);
end