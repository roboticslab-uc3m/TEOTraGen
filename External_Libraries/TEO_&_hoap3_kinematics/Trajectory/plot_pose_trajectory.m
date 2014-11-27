function plot_pose_trajectory (trajectory, type)
%PLOT_POSE_TRAJECTORY plots a pose trajectory
%   plot_pose_trajectory (trajectory, type) plots a 6D trajectory, whose first 3
%   elements represent a translation in the space and the last 3 the
%   rotation in roll, pitch and yaw angles.
%
%   If type is 'velocity' the trajectory is intended to be a first
%   derivative of translation/rotation.
%   If 'type' is 'acceleration' it is intended to be second-derivative.
%
%   Example:
%   	vector = rand (6,1000);
%       trajectory = create_trajectory_structure(vector, 1e-3)
%       plot_pose_trajectory (trajectory);
%
%   See also CREATE_PLOT_STRUCTURE, PLOT_TRAJECTORY.

%   Author: Paolo Pierro
%   $Revision: 0.9 $  $Date: 2011/02/03 $
if nargin < 2  
    type = '';
end
trajectory_n = split_trajectories(trajectory);
L = length(trajectory_n);
if L~=6
    error('PLOT_POSE_TRAJECTORY:argChk', 'Wrong type of input arguments: trajectory should be a 6 dimensional trajectory')
end

switch type
    case 'velocity'
        dim = '/s]';
    case 'acceleration'
        dim = '/s^2]';
    otherwise
        dim = ']';
end

unit_time = '[s]';
unit_translation = ['[m',dim];
unit_rotation = ['[rad',dim];

% trajectory for translation and rotation
figure
subplot(3,2,1);
z = create_plot_structure(trajectory_n(3),unit_time, ['z ', unit_translation], 'Translation');
plot_trajectory(z);

subplot(3,2,2);
ya = create_plot_structure(trajectory_n(6),unit_time, ['\psi ', unit_rotation], 'Rotation');
plot_trajectory(ya);

subplot(3,2,3);
y = create_plot_structure(trajectory_n(2),unit_time, ['y ', unit_translation], '');
plot_trajectory(y);

subplot(3,2,4);
p = create_plot_structure(trajectory_n(5),unit_time, ['\theta ', unit_rotation], '');
plot_trajectory(p);

subplot(3,2,5);
x = create_plot_structure(trajectory_n(1),['time ', unit_time], ['x ', unit_translation], '');
plot_trajectory(x);

subplot(3,2,6);
r = create_plot_structure(trajectory_n(6),['time ', unit_time], ['\phi ', unit_rotation], '');
plot_trajectory(r);
end