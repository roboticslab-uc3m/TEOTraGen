function write_matrix_to_file (matrix, function_name, input, output, joint_var)
%  write_matrix_to_file writes the symbolic matrix specified as input in a
%  desired matlab function.
%  As inputs, the following parameters have to be specified:
%       matrix = the symbolic matrix that represents the function
%       function_name = the desired name of the function
%       input = the input variable of the function to be created
%       output = the output variable of the function
%       joint_var = vector indicating the joint variables (i.e. teta1, d2, ... tetan)
%     
%  example of usage:
%       write_matrix_to_file(J, 'evaluate_jacobian','q','J',joint_var);
%  
%   Author: Paolo Pierro
%   $Revision: 3.0 $  $Date: 2009/03/02 $

[m,n]=size(matrix);
n_var = length(joint_var);
file_name = [function_name,'.m'];
fid = fopen(file_name, 'w');
fprintf(fid, ['function ', output, ' = ', function_name, ' (', input, ')\n']);
fprintf(fid, [output, '=[']);
for ii=1:m
    for jj=1:n
        string = char(simplify(matrix(ii,jj)));
        for kk=1:n_var
            string = strrep(string, char(joint_var(kk)), [input,'(',int2str(kk),')']);
        end
        if jj==n
            if ii==m
                fprintf(fid, '\t%s];', string);
            else            
                fprintf(fid, '\t%s;\n', string);
            end
        else
            fprintf(fid, '\t%s,', string);
        end
    end
end
fclose (fid);