function create_nested_functions_new(kinematics, lib_function_name)
create_lib_function (lib_function_name, kinematics);

n = numel(kinematics);
for jj=1:n
    if exists_field(kinematics(jj), 'type')
        if strcmpi(kinematics(jj).type, 'pose')
            create_pose_function (kinematics(jj).value, kinematics(jj).name, kinematics(jj).var);
            move2libfunction (kinematics(jj).name, lib_function_name);
        elseif strcmpi(kinematics(jj).type, 'pose_rpy')
            create_pose_function (kinematics(jj).value, kinematics(jj).name, kinematics(jj).var, 'rpy');
            move2libfunction (kinematics(jj).name, lib_function_name);
        elseif strcmpi(kinematics(jj).type, 'generic')
            create_matrix_function (kinematics(jj).value, kinematics(jj).name, kinematics(jj).var);
            move2libfunction (kinematics(jj).name, lib_function_name);
        end
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
    if exists_field(kinematics(jj), 'type')
        text_handle = ['\n', output,'.', kinematics(jj).name, ' = @', kinematics(jj).name, ';'];
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