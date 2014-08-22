function trajectory = create_trajectory_template (manipulator_fields, Ts)
%CREATE_TRAJECTORY_TEMPLATE creates a new trajectory template.
%   trajectory = create_trajectory_template (manipulator_fields, Ts)
%   generates a struct where the fields are 'manipulator_fields', which can
%   be also vectors. It also creates the following fields:
%       Ts   : it is the sampling time, given as input.
%       time : it is the fields xcorrespoinding to the time vector (here
%              set to 0)
%       T    : field for the total duration of the trajectory (here set to
%              0)
%
%   See also HUMANOID_OPERATIONAL_FIELDS, CREATE_TRAJECTORY_STRUCTURE.

%   Author: Paolo Pierro
%   $Revision: 0.8 $  $Date: 2011/01/26 $

L = length(manipulator_fields);
for jj=1:L
    trajectory.(manipulator_fields(jj).name) = zeros(manipulator_fields(jj).size, 1);
end
trajectory.Ts   = Ts;
trajectory.time = 0;
trajectory.T    = 0;