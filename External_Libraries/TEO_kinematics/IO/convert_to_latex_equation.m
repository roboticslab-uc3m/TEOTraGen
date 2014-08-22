function A_latex = convert_to_latex_equation (A_sym, output_var, variables_in)
%CONVERT_TO_LATEX_EQUATION  LaTeX representation of a symbolic expression
%in the form of equation
%   CONVERT_TO_LATEX_EQUATION(S, A) returns the LaTeX representation of the
%   symbolic expression S with LaTeX name A.
%   The result is copied to the system clipboard.
%
%   Examples:
%      syms x
%      f = taylor(log(1+x));
%      convert_to_latex_equation(f,'f') = 
%         \begin{equation}
%             f = \frac{x^5}5 - \frac{x^4}4 + \frac{x^3}3 - \frac{x^2}2 + x
%         \end{equation}
%
%      syms alpha t
%      A = [alpha t alpha*t];
%        convert_to_latex_equation(H,'H_4') = 
%         \begin{equation}
%             \mathbf{H}_4 = \left[ \begin{array}{ccc}
%                  1 & \frac12 & \frac13 \\
%                  \frac12 & \frac13 & \frac14 \\
%                  \frac13 & \frac14 & \frac15
%             \end{array} \right]
%         \end{equation}
%
%   See also SYM/PRETTY, SYM/CCODE, SYM/FORTRAN.

%   Author: Paolo Pierro
%   $Revision: 0.5 $  $Date: 2011/01/19 $

% Definition of string constants
newline     = sprintf('\n');
tab         = sprintf('\t');
newcolomn   = ' &';
newrow      = ' \\';

% First line fro the equation beginning
A_latex = ['\begin{equation}', newline, tab];
type = is_sym_vector(A_sym);
A_latex = [A_latex, convert_output_name(output_var, type), ' = '];
if type == 1
    [m, n] = size(A_sym);
    A_latex = [A_latex, '\left[ \begin{array}{', 'c' * ones(1,m), '}', newline];
    for ii = 1:m
        A_latex = [A_latex, tab, tab];
        for jj = 1:n
            A_latex = [ A_latex,' ', latex2(A_sym(ii,jj))];
            if jj < n
                A_latex = [A_latex, newcolomn];
            elseif ii < m
                A_latex = [A_latex, newrow, newline];
            else
                A_latex = [A_latex, newline];
            end
        end
    end
    A_latex = [A_latex, tab, '\end{array} \right]',newline];
else
    A_latex = [A_latex, latex2(A_sym),newline];
end
A_latex = [A_latex, '\end{equation}'];
if nargin > 2
    variables = [variables_in, define_standard_variables];
else
    variables = define_standard_variables ();
end
A_latex = change_variables_name (A_latex, variables);
clipboard('copy', A_latex)
end

function type = is_sym_vector(A_sym)
[m,n] = size(A_sym);
if n*m==1
    % it is A_sym scalar
    type = 0;
else
    % it is A_sym vector
    type = 1;
end
end

function output_latex = convert_output_name(output_var, type)
% This function converts a variable name as a string to a LaTeX standard,
% considering if it is a vector or it has Superscript and subscript
output_struct = define_name_struct(output_var);
output_latex = write_scalar_vector(output_struct.name, type);
if isempty(output_struct.Superscript)== 0
    % it has superscript
    output_latex = [convert2Superscript(output_struct.Superscript),output_latex];
end
if isempty(output_struct.subscript) == 0
    % it has subscript
    output_latex = [output_latex,convert2subscript(output_struct.subscript)];
end
end

function output_struct = define_name_struct (output_var)
% This function creates a struct of a varioable name, considering
% Superscript and subscript
output_struct.name = output_var;
output_struct.Superscript = [];
output_struct.subscript = [];

L = length(output_struct.name);
if L > 3
    if strcmp(output_struct.name(2:3),'__') == 1
        % it has superscript
        output_struct.Superscript = output_struct.name(1);
        output_struct.name = output_struct.name(4:L);
    end
end

L = length(output_struct.name);
if L > 2
    if strcmp(output_struct.name(L-1), '_') == 1
        % it has superscript
        output_struct.subscript = output_struct.name(L);
        output_struct.name = output_struct.name(1:L-2);
    end
end
end

function OUTname = write_scalar_vector(INname, type)
if type == 1
    % it is A_sym vector
    OUTname = ['\mathbf{',INname,'}'];
else
    % it is A_sym scalar
    OUTname = INname;
end
end

function OUTstr = convert2Superscript(INstr)
L = length(INstr);
if L > 1
    OUTstr = ['{', INstr, '}'];
else
    OUTstr = INstr;
end
OUTstr = ['{}^', OUTstr];
end

function OUTstr = convert2subscript(INstr)
L = length(INstr);
if L > 1
    OUTstr = ['{', INstr, '}'];
else
    OUTstr = INstr;
end
OUTstr = ['_',OUTstr];
end

function OUTname = latex2(INname)
%This functions is based on built-in Matlab function latex, modified in
%order to correct its bugs
OUTname = latex(INname);
OUTname = strrep(OUTname,'\!','');
OUTname = strrep(OUTname,'\,','');
OUTname = clean_useless_function (OUTname, '\mathrm');
OUTname = clean_uselessness (OUTname);
end

function OUTname = clean_useless_function (INname, useless)
OUTname = INname;
L = length(INname);
L_useless = length(useless);
bracket_begin = strfind(INname, useless);
num_uselessness = length(bracket_begin);
while num_uselessness > 1
    OUTname = [OUTname(1:bracket_begin(1)+L_useless-1),clean_useless_function(OUTname(bracket_begin(1)+L_useless:L), useless)];
    bracket_begin = strfind(OUTname, useless);
    num_uselessness = length(bracket_begin);
end
if num_uselessness == 1
    L = length(OUTname);
    bracket_arg = find_bracket_argument (OUTname(bracket_begin+L_useless:L));
    OUTname = strrep(OUTname, [useless,'{',bracket_arg,'}'], bracket_arg);
end
end

function OUTname = clean_uselessness (INname)
OUTname = INname;
useless1 = '_{';
bracket_begin = strfind(OUTname, useless1);
useless2 = '}';
bracket_close = strfind(OUTname, useless2);
bracket_dif = mat_diff(bracket_close, bracket_begin);
[m, n] = find(bracket_dif == 3);
%bracket_dif = bracket_close - bracket_begin;
OUTname([bracket_begin(n) + 1, bracket_close(m)])=[];
end

function OUTname = change_variables_name (INname, variables)
OUTname=INname;
L = length(variables);
for jj=1:L
    OUTname = strrep(OUTname,variables(jj).oldname,variables(jj).newname);
end
end

function bracket_arg = find_bracket_argument (string)
bracket_close  = find_closing_bracket (string);
bracket_arg = string(2:bracket_close-1);
end

function out = find_closing_bracket (strin)
bracket_end  = strfind(strin, '}');
out = bracket_end(1);
end

function variables = define_standard_variables ()
variables = struct([]);
symbols = load_standard_symbols();
L = length(symbols);
for jj=1:L
    variables = add_variable (variables, symbols(jj).name, convert2latex_symbol (symbols(jj).name));
end
end

function variables_add = add_variable (variables, old_name, new_name)
variables_add = variables;
L = length(variables_add);
variables_add(L+1).oldname = old_name;
variables_add(L+1).newname = new_name;
end

function latex_symbol = convert2latex_symbol (symbol)
latex_symbol = ['\',symbol];
end

function symbols = load_standard_symbols()
jj = 1;
symbols(jj).name = 'alpha';
jj=jj+1;
symbols(jj).name = 'beta';
jj=jj+1;
symbols(jj).name = 'gamma';
jj=jj+1;
symbols(jj).name = 'delta';
jj=jj+1;
symbols(jj).name = 'theta';
jj=jj+1;
symbols(jj).name = 'lambda';
jj=jj+1;
symbols(jj).name = 'psi';
jj=jj+1;
symbols(jj).name = 'omega';
jj=jj+1;
symbols(jj).name = 'phi';
end

function D = mat_diff(a, b)
m = length(a);
n = length(b);
D = zeros(m, n);
for jj = 1:n
    D(:, jj) = a - b(jj);
end
end