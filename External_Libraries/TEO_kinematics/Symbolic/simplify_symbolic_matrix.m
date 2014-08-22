function S = simplify_symbolic_matrix(h, max_var, max_iterations)
%SIMPLIFY_SYMBOLIC_MATRIX simplifies a symbolic matrix, if it does not have too
%many symbolic variables (max_var). It is possible to define also the max
%number of iterations (max_iterations)

%   Author: Paolo Pierro
%   $Revision: 1.1 $  $Date: 2011/08/05 $

if nargin < 2
    max_var = 4;
end
if has_few_vars(h, max_var)
    if nargin < 3
        % Number of attempts of the simplify function
        max_iterations = 100;
    end
    S = simplify(h, max_iterations);
else
    S = h;
end
end

function res = has_few_vars(h, n)
% HAS_FEW_VARS outputs 1 if the number of variables of h are less than n
if isnumeric(h)
    res = 0;
else
    res = (length(symvar(h)) <= n);
end
end