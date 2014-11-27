function dq = time_derivate_symbolic_variable (q)
% TIME_DERIVATE_SYMBOLIC_VARIABLE(Q)
%
%   This function is purely for symbolic use and permits automatically
%   generate a vector which contains variables whose name is the original
%   name with a '_dot' added.

var = symvar(q)';
n = length(var);
var_dot = sym(zeros(n, 1));

for jj = 1:n,
  var_dot(jj) = sym([char(var(jj)), '_dot'], 'real');
end

dq = subs(q, var, var_dot);

end