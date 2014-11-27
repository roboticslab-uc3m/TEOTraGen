function trajectory = extract_trajectory(trajectory_structure, field_names, range, type)
%   EXTRACT_TRAJECTORY extracts just some elements a trajectory structure

%   See also CREATE_TRAJECTORY_TEMPLATE, CREATE_TRAJECTORY_STRUCTURE.

%   Author: Domingo Esteban
%   $Revision: 1.0 $  $Date: 2014/03/26 $

if nargin < 4
    error('EXTRACT_TRAJECTORY:argChk', 'Wrong number of input arguments')
end

if nargin < 3
  type = 'time';
end

if check_range (trajectory_structure, range, type)
  error('EXTRACT_TRAJECTORY:argChk', 'The introduced range is not between the trajectory')
end

if strcmp(type,'time') || strcmp(type,'t')
  new_start = find(trajectory_structure.time==range(1),1);
  new_end = find(trajectory_structure.time==range(end),1);
  
elseif strcmp(type,'number') || strcmp(type,'n')
  new_start = range(1);
  new_end = range(end);
  
end

trajectory = trajectory_structure;

L = length(field_names);
for jj=1:L
  if isfield(trajectory_structure, field_names(jj).name)
    trajectory.(field_names(jj).name) = insert_vector (trajectory.(field_names(jj).name), new_start, new_end);
  else
    error('EXTRACT_TRAJECTORY:argChk', 'the field is not inside the trajectory structure')
  end
end

trajectory.time = insert_vector (trajectory.time, new_start, new_end);
trajectory.T = trajectory.time(end);

end

function error_range = check_range(trajectory_structure, range, type)
% CHECK_RANGE checks if the range is between the trajectory structure.

if isempty(range)
  error_range = 1;
elseif (range(1) < 0) || (range(end) < 0) || (range(1) >= range(end))
  error_range = 1;
elseif strcmp(type,'time') || strcmp(type,'t')
  if (isempty(find(trajectory_structure.time==range(1),1))) || (isempty(find(trajectory_structure.time==range(end),1)))
    error_range = 1;
  else
    error_range = 0;
  end
elseif strcmp(type,'number') || strcmp(type,'n')
  if mod(range(1),1) || mod(range(end),1)
    error_range = 1;
  elseif (range(1) > size(trajectory_structure.time,2)) || (range(end) > size(trajectory_structure.time,2))
    error_range = 1;
  else
    error_range = 0;
  end
else
  error('EXTRACT_TRAJECTORY:wrongInput', 'The type of range is not a validate option')
end

end


function data = insert_vector (old_data, new_start, new_end)
% INSERT_VECTOR inserts the 'new_data' vector going from 'new_start' to
% 'new_end' into 'old_data'.

if (new_start < new_end)
  data = old_data(:,new_start:new_end);
else
  error('EXTRACT_TRAJECTORY:wrongInput', 'The type of range is not a validate option')
end

end
