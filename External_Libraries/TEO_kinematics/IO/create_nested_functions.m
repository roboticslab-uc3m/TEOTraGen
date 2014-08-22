function create_nested_functions(kinematics, lib_function_name)
create_lib_function (lib_function_name, kinematics);

n = numel(kinematics);
for jj=1:n
    if exists_field(kinematics(jj), 'T')
        create_pose_function (kinematics(jj).T.value, kinematics(jj).T.name, kinematics(jj).var);
        move2libfunction (kinematics(jj).T.name, lib_function_name);
    end
    if exists_field(kinematics(jj), 'J')
        create_matrix_function (kinematics(jj).J.value, kinematics(jj).J.name, kinematics(jj).var);
        move2libfunction (kinematics(jj).J.name, lib_function_name);
    end
    if exists_field(kinematics(jj), 'R')
        create_matrix_function (kinematics(jj).R.value, kinematics(jj).R.name, kinematics(jj).var);
        move2libfunction (kinematics(jj).R.name, lib_function_name);
    end
end

end

function create_lib_function (function_name, kinematics)
file_name = [function_name, '.m'];
fid = fopen(file_name, 'w');
output='h';
text_fun = ['function ', output, ' = ', function_name, ' ()'];
fprintf(fid, text_fun);
n = numel(kinematics);
for jj=1:n
    if exists_field(kinematics(jj), 'T')
        text_handle = ['\n', output,'.', kinematics(jj).T.name, ' = @', kinematics(jj).T.name, ';'];
        fprintf(fid, text_handle);
    end
    if exists_field(kinematics(jj), 'J')
        text_handle = ['\n', output,'.', kinematics(jj).J.name, ' = @', kinematics(jj).J.name, ';'];
        fprintf(fid, text_handle);
    end
    if exists_field(kinematics(jj), 'R')
        text_handle = ['\n', output,'.', kinematics(jj).R.name, ' = @', kinematics(jj).R.name, ';'];
        fprintf(fid, text_handle);
    end    
end
fprintf(fid, '\nend');
fclose(fid);
end

function move2libfunction (original, destination)
file_name = [original, '.m'];
fid = fopen(file_name, 'r');
text = fscanf(fid,'%c');
fclose(fid);
delete(file_name);

file_name = [destination, '.m'];
fid = fopen(file_name, 'a');
fprintf(fid, '\n');
fprintf(fid, '%c', text);
fprintf(fid, '\nend');
fclose(fid);
end

function output = exists_field (S, fieldname)
if isfield(S, fieldname)
    if ~isempty(S.(fieldname))
        output = 1;
        return;
    end
end
output = 0;
end