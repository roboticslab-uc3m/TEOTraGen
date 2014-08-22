function [Trajectory, InertialTrajectory] = assign_operational_trajectory(Trajectory, q0, Ts, humanoid_structure, humanoid_fields, humanoid_kinematics, units, InertialTrajectory, InitialPose)

% Check number of arguments
if nargin > 9
  error('ErrorTEOTraGen:wrongInputGUI', 'Wrong number of input arguments');
end

% Use an initial pose
if nargin < 9
  InitialPose = 0;
end

% Compute or not an inertial trajectory (trajectory w.r.t. inertial frame) 
if nargin < 8
  CalculateInertialTrajectory = 0;
else
  CalculateInertialTrajectory = 1;
end

% Parameter for length units convertion
if strcmp(units.pos,'m')
  ConvPos = 1;
elseif strcmp(units.pos,'mm')
  ConvPos = 1000;
else
  error('ErrorTEOTraGen:wrongOption', 'Wrong position units');
end

% Parameter for orientation units convertion
if strcmp(units.orient,'rad')
  ConvOrient = 1;
elseif strcmp(units.orient,'degrees')
  ConvOrient = 180/pi;
else
  error('ErrorTEOTraGen:wrongOption', 'Wrong orientation units');
end

% Assign current (for q0) local poses
pose_CoM = pose_quat2rpy(humanoid_kinematics.RF_T_CoM(q0));  % RF is the standing leg
pose_RF = pose_quat2rpy(humanoid_kinematics.CoM_T_RF(q0));
pose_LF = pose_quat2rpy(humanoid_kinematics.CoM_T_LF(q0));
pose_RH = pose_quat2rpy(humanoid_kinematics.CoM_T_RH(q0));
pose_LH = pose_quat2rpy(humanoid_kinematics.CoM_T_LH(q0));

% Convert poses to corresponding units
if  (~strcmp(units.orient,'rad') || ~strcmp(units.pos,'m'))
  pose_RF = [pose_RF(1:3)*ConvPos; pose_RF(4:6)*ConvOrient];
  pose_LF = [pose_LF(1:3)*ConvPos; pose_LF(4:6)*ConvOrient];

  pose_CoM = [pose_CoM(1:3)*ConvPos; pose_CoM(4:6)*ConvOrient];
  pose_RH = [pose_RH(1:3)*ConvPos; pose_RH(4:6)*ConvOrient];
  pose_LH = [pose_LH(1:3)*ConvPos; pose_LH(4:6)*ConvOrient];
end
  
% Insert trajectory generated into Trajectory structure
Trajectory = insert_trajectory(Trajectory, humanoid_fields, create_trajectory_structure(pose_RF, Ts, 0), 'RF');
Trajectory = insert_trajectory(Trajectory, humanoid_fields, create_trajectory_structure(pose_LF, Ts, 0), 'LF');
Trajectory = insert_trajectory(Trajectory, humanoid_fields, create_trajectory_structure(pose_CoM, Ts, 0), 'CoM');
Trajectory = insert_trajectory(Trajectory, humanoid_fields, create_trajectory_structure(pose_RH, Ts, 0), 'RH');
Trajectory = insert_trajectory(Trajectory, humanoid_fields, create_trajectory_structure(pose_LH, Ts, 0), 'LH');
  



% Calculate Intertial (Global) Poses

%TODO: These poses are assuming q0(:)=0, generate the poses using the
%current q0 values

if CalculateInertialTrajectory == 1,
  
  % TEO is standing with both foot on the floor
  Ipose_RF = [0; (-humanoid_structure.legs.link_lengths(1) + humanoid_structure.legs.link_lengths(4))/ConvPos; 0; 0; 0; 0];
  Ipose_LF = [0; (humanoid_structure.legs.link_lengths(1) - humanoid_structure.legs.link_lengths(4))/ConvPos; 0; 0; 0; 0];

  Ipose_CoM = pose_tr2rpy(pose_quat2tr(pose_rpy2quat(Ipose_RF))*pose_quat2tr(humanoid_kinematics.RF_T_CoM(q0)));  % RF is the standing leg
  Ipose_RH = pose_tr2rpy(pose_quat2tr(pose_rpy2quat(Ipose_CoM))*pose_quat2tr(humanoid_kinematics.CoM_T_RH(q0)));
  Ipose_LH = pose_tr2rpy(pose_quat2tr(pose_rpy2quat(Ipose_CoM))*pose_quat2tr(humanoid_kinematics.CoM_T_LH(q0)));

  if  (~strcmp(units.orient,'rad') || ~strcmp(units.pos,'m'))
    % Convert poses to corresponding units
    Ipose_RF = [Ipose_RF(1:3)*ConvPos; Ipose_RF(4:6)*ConvOrient];
    Ipose_LF = [Ipose_LF(1:3)*ConvPos; Ipose_LF(4:6)*ConvOrient];

    Ipose_CoM = [Ipose_CoM(1:3)*ConvPos; Ipose_CoM(4:6)*ConvOrient];
    Ipose_RH = [Ipose_RH(1:3)*ConvPos; Ipose_RH(4:6)*ConvOrient];
    Ipose_LH = [Ipose_LH(1:3)*ConvPos; Ipose_LH(4:6)*ConvOrient];
  end
  
InertialTrajectory = insert_trajectory(InertialTrajectory, humanoid_fields, create_trajectory_structure(Ipose_RF, Ts, 0), 'RF');
InertialTrajectory = insert_trajectory(InertialTrajectory, humanoid_fields, create_trajectory_structure(Ipose_LF, Ts, 0), 'LF');
InertialTrajectory = insert_trajectory(InertialTrajectory, humanoid_fields, create_trajectory_structure(Ipose_CoM, Ts, 0), 'CoM');
InertialTrajectory = insert_trajectory(InertialTrajectory, humanoid_fields, create_trajectory_structure(Ipose_RH, Ts, 0), 'RH');
InertialTrajectory = insert_trajectory(InertialTrajectory, humanoid_fields, create_trajectory_structure(Ipose_LH, Ts, 0), 'LH');

end
     
end
