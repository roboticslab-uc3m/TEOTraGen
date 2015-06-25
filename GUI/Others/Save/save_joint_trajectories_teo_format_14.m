function [ ] = save_joint_trajectories_teo_format_14(traj_name_string, q, dq, ddq)
%SAVE_TRAJECTORY_TEO_FORMAT_14 Summary of this function goes here
%   Detailed explanation goes here

if nargin == 0
  error('save_joints_trajectories_teo_format_14:argChk', 'Wrong number of input arguments')
end


%%%%%%%%%%%%%%%%%%%%%%%%
% Save joint positions %
%%%%%%%%%%%%%%%%%%%%%%%%

try
  % Convert to degrees
  q = radtodeg(q);

  % Change the signs of joints with different orientation
  q(6,:) = -q(6,:);
  q(7,:) = -q(7,:);
  q(8,:) = -q(8,:);
  q(15,:) = -q(15,:);
  q(17,:) = -q(17,:);
  q(18,:) = -q(18,:);
  q(19,:) = -q(19,:);
  q(20,:) = -q(20,:);
  q(23,:) = -q(23,:);
  q(25,:) = -q(25,:);
  
  %Current save format:
  %The q vector sorting standard at june/2015 is as follows:
  %head group 0 / joints yaw=0.0, pitch=0.1
  %right arm (from robot pov) group 1 / joints from shoulder to hand 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6
  %left arm (from robot pov) group 2 / joints from shoulder to hand 2.0, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6
  %torso group 3 / joints yaw=3.0, pitch=3.1
  %left leg (from robot pov) group 4 / joints from shoulder to hand 4.0, 4.1, 4.2, 4.3, 4.4, 4.5
  %left leg (from robot pov) group 5 / joints from shoulder to hand 5.0, 5.1, 5.2, 5.3, 5.4, 5.5
  %vector sorting is then:
  %[  0.0, 0.1, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 2.0, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6 ] for manipulation control
  %[  3.0, 3.1, 4.0, 4.1, 4.2, 4.3, 4.4, 4.5, 5.0, 5.1, 5.2, 5.3, 5.4, 5.5 ] for locomotion control
  %theta (q) correpondence:
  %[  27, 28, 15, 16, 17, 18, 19, 20, *no q for grasper, 21, 22, 23, 24, 25, 26, *no q for grasper ] for manipulation control
  %[  13, 14, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 ] for locomotion control

  % LEGS FILE
  %[file1, path1] = uiputfile([traj_name_string '.csv'], 'Save Joint Angles for locomotion as');
  file1='teo_step.csv';
  path1='/home/rh2/locomotion/';
  csvid = fopen(strcat(path1,file1), 'w');
  fprintf(csvid, '%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n',...
  [q(13,:); q(14,:); q(1,:); q(2,:); q(3,:); q(4,:); q(5,:); q(6,:); q(7,:); q(8,:); q(9,:); q(10,:); q(11,:); q(12,:)]);
  fclose(csvid);

  
  % ARMS FILE
  %[file2, path2] = uiputfile([traj_name_string '.csv'], 'Save Joint Angles for manipulation as');
  file2='teo_step.csv';
  path2='/home/rh2/manipulation/';
  q_size = size(q);
  q_zerovalues =  zeros(1,q_size(2));
  csvid = fopen(strcat(path2,file2), 'w');
  fprintf(csvid, '%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n',...
  [q_zerovalues; q_zerovalues; q(15,:); q(16,:); q(17,:); q(18,:); q(19,:); q(20,:); q_zerovalues ; q(21,:); q(22,:); q(23,:); q(24,:); q(25,:); q(26,:); q_zerovalues ]);
  fclose(csvid);
  
  
catch
  disp('Save joint trajectories canceled');
end

    
%%%%%%%%%%%%%%%%%%%%%%%%%
% Save joint velocities %
%%%%%%%%%%%%%%%%%%%%%%%%%

if nargin > 2
  try
    % Convert to degrees
    dq = radtodeg(dq);

    % Change the signs of joints with different orientation
    dq(6,:) = -dq(6,:);
    dq(7,:) = -dq(7,:);
    dq(8,:) = -dq(8,:);
    dq(15,:) = -dq(15,:);
    dq(17,:) = -dq(17,:);
    dq(18,:) = -dq(18,:);
    dq(19,:) = -dq(19,:);
    dq(20,:) = -dq(20,:);
    dq(23,:) = -dq(23,:);
    dq(25,:) = -dq(25,:);


    % LEGS FILE
    [file3, path3] = uiputfile([traj_name_string '_legs_dq.csv'], 'Save Joint Velocities as');

    csvid = fopen(strcat(path3,file3), 'w');
    fprintf(csvid, '%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n',...
                                                                                       [dq(13,:); dq(7,:); dq(8,:); dq(9,:); dq(10,:); dq(11,:); dq(12,:); dq(14,:); dq(1,:); dq(2,:); dq(3,:); dq(4,:); dq(5,:); dq(6,:)]);
    fclose(csvid);


    % ARMS FILE
    [file4, path4] = uiputfile([traj_name_string '_arms_dq.csv'], 'Save Joint Velocities as');

    csvid = fopen(strcat(path4,file4), 'w');
    fprintf(csvid, '%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n',...
                                                                                       [dq(15,:); dq(16,:); dq(17,:); dq(18,:); dq(19,:); dq(20,:); dq(21,:); dq(22,:); dq(23,:); dq(24,:); dq(25,:); dq(26,:)]);
    fclose(csvid);

  catch
    disp('Save joint velocities canceled');
  end
  
end
    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Save joint accelerations %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
if nargin > 3
  try    
    % Convert to degrees
    ddq = radtodeg(ddq);

    % Change the signs of joints with different orientation
    ddq(6,:) = -ddq(6,:);
    ddq(7,:) = -ddq(7,:);
    ddq(8,:) = -ddq(8,:);
    ddq(15,:) = -ddq(15,:);
    ddq(17,:) = -ddq(17,:);
    ddq(18,:) = -ddq(18,:);
    ddq(19,:) = -ddq(19,:);
    ddq(20,:) = -ddq(20,:);
    ddq(23,:) = -ddq(23,:);
    ddq(25,:) = -ddq(25,:);

    
    % LEGS FILE
    [file5, path5] = uiputfile([traj_name_string '_legs_ddq.csv'], 'Save Joint Accelerations as');
    
    csvid = fopen(strcat(path5,file5), 'w');
    fprintf(csvid, '%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n',...
                                                                                       [ddq(13,:); ddq(7,:); ddq(8,:); ddq(9,:); ddq(10,:); ddq(11,:); ddq(12,:); ddq(14,:); ddq(1,:); ddq(2,:); ddq(3,:); ddq(4,:); ddq(5,:); ddq(6,:)]);
    fclose(csvid);
    
    % ARMS FILE
    [file6, path6] = uiputfile([traj_name_string '_arms_dq.csv'], 'Save Joint Accelerations as');

    csvid = fopen(strcat(path6,file6), 'w');
    fprintf(csvid, '%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n',...
                                                                                       [ddq(15,:); ddq(16,:); ddq(17,:); ddq(18,:); ddq(19,:); ddq(20,:); ddq(21,:); ddq(22,:); ddq(23,:); ddq(24,:); ddq(25,:); ddq(26,:)]);
    fclose(csvid);    
    
  catch
    disp('Save joint accelerations canceled');
  end
end

end

