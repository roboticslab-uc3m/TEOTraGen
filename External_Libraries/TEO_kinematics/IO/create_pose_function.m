function create_pose_function (T, function_name, joint_var, orientation)
%   CREATE_POSE_FUNCTION generate a MATLAB file function from a sym
%   homogeneous position matrix
%
%   create_pose_function(T, 'calculate_position', q) generates the
%   MATLAB function 'calculate_position' which calculates the pose of a
%   manipulator specified by the matrix T. The joint variables are defined
%   by the vector q. The orientation is expressed in quaternion.
%
%   create_pose_function(T, 'calculate_position', q, ‘RPY’)
%   generates the MATLAB function 'calculate_position' which calculates the
%   pose of a manipulator specified by the matrix T. The joint variables
%   are defined by the vector q. The orientation is expressed in roll,
%   pitch and yaw angles.
  
%   Author: Paolo Pierro
%   $Revision: 1.0 $  $Date: 2011/03/03 $

if nargin < 4
    orientation = 'qt';
end

% Definition of constants
input = 'q';
output = 'pose';
NEWLINE = '\n';
TAB = '\t';
NEWROW = [';', NEWLINE, TAB, TAB];

% Create position part
pst = [TAB, char(T(1,4)), NEWROW ,char(T(2,4)), NEWROW ,char(T(3,4)), NEWROW];

% Create position part
if strcmpi(orientation, 'rpy')
    orn = ['atan2(', char(T(3,2)), ',', char(T(3,3)), ')', NEWROW, 'atan2(', char(-T(3,1)), ',', char(sqrt(T(3,2).^2+T(3,3).^2)), ')', NEWROW, 'atan2(',char(T(2,1)), ',', char(T(1,1)), ')'];
else
    orn = [char(1/2*sqrt(T(1,1)+T(2,2)+T(3,3)+1)), NEWROW, '1/2*sign(', char(T(3,2)-T(2,3)), ')*', char(sqrt(T(1,1)-T(2,2)-T(3,3)+1)), NEWROW, '1/2*sign(', char(T(1,3)-T(3,1)), ')*', char(sqrt(T(2,2)-T(1,1)-T(3,3)+1)), NEWROW, '1/2*sign(', char(T(2,1)-T(1,2)), ')*', char(sqrt(T(3,3)-T(2,2)-T(1,1)+1))];
end

pose = [pst, orn];
n = length(joint_var);
for kk=1:n
    pose = strrep(pose, char(joint_var(kk)),  [input,'(',int2str(kk),',:)']);
end
pose = strrep(pose, '*', '.*');
pose = strrep(pose, '/', './');
pose = strrep(pose, '^', '.^');

% write to file
file_name = [function_name, '.m'];
fid = fopen(file_name, 'w');
fprintf(fid, ['function ', output, ' = ', function_name, ' (', input, ')', NEWLINE]);
fprintf(fid, [output, '=[', pose, '];']);
fclose(fid);
end