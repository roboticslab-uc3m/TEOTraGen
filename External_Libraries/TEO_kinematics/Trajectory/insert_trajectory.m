function trajectory = insert_trajectory(trajectory_template, field_names, trajectory_struct, trajectory_field)
%   INSERT_TRAJECTORY inserts a trajectory structure into a complex
%   trajectory template.
%   trajectory = insert_trajectory(trajectory_template, field_names,
%   trajectory_struct, trajectory_field) will insert the trajectory defined
%   by 'trajectory_struct' into the field 'trajectory_field' of the
%   'trajectory_template', consisting of the fields 'field_names'.
%
%   See also CREATE_TRAJECTORY_TEMPLATE, CREATE_TRAJECTORY_STRUCTURE.

%   Author: Paolo Pierro
%   $Revision: 1.0 $  $Date: 2011/01/26 $

if nargin < 4
    error('INSERT_TRAJECTORY:argChk', 'Wrong number of input arguments')
end

if  ~isfield(trajectory_template, trajectory_field)
    error('INSERT_TRAJECTORY:argChk', 'The entered trajectory field is not a field of the trajectory template.')
end

if check_Ts (trajectory_template, trajectory_struct)
    error('INSERT_TRAJECTORY:argChk', 'The present sampling time Ts is different from your input')
end

[old_indices, old_start, old_end] = extract_time_indices (trajectory_template);
[new_indices, new_start, new_end] = extract_time_indices (trajectory_struct);

trajectory = trajectory_template;
L = length(field_names);

for jj=1:L
    if strcmp(field_names(jj).name, trajectory_field)
        trajectory.(field_names(jj).name) = insert_vector (trajectory.(field_names(jj).name), old_start, old_end, trajectory_struct.data, new_start, new_end);
    else
        trajectory.(field_names(jj).name) = fill_vector (trajectory.(field_names(jj).name), old_end, new_end);
    end
end

if new_end > old_end
    [trajectory.time, trajectory.T] = generate_time_vector (old_start, new_end, trajectory.Ts);
end
end

function error_Ts = check_Ts(trajectory_old, trajectory_new)
% CHECK_TS checks the input trajectories have the same sampling time.

error_Ts = trajectory_old.Ts ~= trajectory_new.Ts;
end

function [indices, start_index, end_index] = extract_time_indices (trajectory)
% EXTRACT_TIME_INDICES extracts the initial, final and all the time indices
% corresponding to the input trajectory.

indices = round(trajectory.time./trajectory.Ts) + 1;
start_index = indices(1);
end_index = indices(length(indices));
end


function data = insert_vector (old_data, old_start, old_end, new_data, new_start, new_end)
% INSERT_VECTOR inserts the 'new_data' vector going from 'new_start' to
% 'new_end' into 'old_data', whose indices go from 'old_start' to
% 'old_end'.

[m, n] = size(old_data);
if new_start > old_end
    data = [old_data(:,old_start:old_end), replicate_vector(old_data(:,old_end), new_start - old_end - 1), new_data];
else
    if new_end >= old_end
        data = [old_data(:,old_start:new_start-1), new_data];
    else
        data = [old_data(:,old_start:new_start-1), new_data, old_data(:,new_end+1:old_end)];
    end
end
end

function data = fill_vector (old_data, old_end, new_end)
% FILL_VECTOR (old_data, old_end, new_end) adds the last value of the
% 'old_data' to a new vector ending in 'new_end'.

[m, n] = size(old_data);
if new_end > old_end
    data = [old_data, replicate_vector(old_data(:,old_end), new_end - old_end)];
else
	data = old_data;
end
end

function [t, T] = generate_time_vector (start_index, end_index, Ts)
% GENERATE_TIME_VECTOR generates a time vector whose indices go from
% 'start_index' to 'end_index' with sampling time 'Ts'. It outputs also the
% whole duration.

t = ((start_index:end_index) - 1) * Ts;
T = (end_index - start_index) * Ts;
end

function data = replicate_vector (vector, n)
% REPLICATE_VECTOR replicates the input 'vector' 'n' times.

data = vector * ones(1, n);
end