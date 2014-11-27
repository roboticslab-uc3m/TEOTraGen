function humanoid_fields = humanoid_operational_fields ()
%HUMANOID_OPERATIONAL_FIELDS creates the fields corresponding to the
%operational space of a humanoid robot.
%It consists of the following fields:
%       1)  name = 'CoM'
%           dimensions = 6
%           description = Center of Mass Trajectory
%
%       2)  name = 'RF'
%           dimensions = 6
%           description = Right Foot Trajectory
%
%       3)  name = 'LF'
%           dimensions = 6
%           description = Left Foot Trajectory
%
%       4)  name = 'RH'
%           dimensions = 6
%           description = Right Hand Trajectory
%
%       5)  name = 'LH'
%           dimensions = 6
%           description = Left Hand Trajectory
%
%       6)  name = 'SF'
%           dimensions = 1
%           description = Support Foot Trajectory.
%               This should be set to +1 if the left is the support foot,
%               -1 if it is the right and 0 in double support

%   Author: Paolo Pierro
%   $Revision: 0.8 $  $Date: 2011/01/26 $

humanoid_fields(1).name = 'CoM';
humanoid_fields(1).size = 6;

humanoid_fields(2).name = 'RF';
humanoid_fields(2).size = 6;

humanoid_fields(3).name = 'LF';
humanoid_fields(3).size = 6;

humanoid_fields(4).name = 'RH';
humanoid_fields(4).size = 6;

humanoid_fields(5).name = 'LH';
humanoid_fields(5).size = 6;

humanoid_fields(6).name = 'SF';
humanoid_fields(6).size = 1;