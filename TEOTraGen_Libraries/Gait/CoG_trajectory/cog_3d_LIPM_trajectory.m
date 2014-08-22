function [ cog_traj, dcog_traj, ddcog_traj] = cog_3d_LIPM_trajectory(footprints_poses, median_zmp, Ts, TimeSS, TimeDS, t0, zc, beta, lambda, g)
%COG_3D_LIPM Generates a Center of Gravity (CoG) using the 3D Linear
%Inverted Pendulum Model (LIPM)
%   Input:
%   Output:


cog_traj = [];
dcog_traj = [];
ddcog_traj = [];

pie = [];

for ii = 2:size(footprints_poses.inertial,2)-1,

    TDS1 = round_to_Ts(t0 + TimeDS/2,Ts);
    TSS = round_to_Ts(TDS1 + TimeSS,Ts);
    TDS2 = round_to_Ts(TSS + TimeDS/2,Ts);

    DSglobal_pose1 = [median_zmp.inertial(:,ii-1);zc;0;0;mean([footprints_poses.inertial(6,ii),footprints_poses.inertial(6,ii-1)])];
    DSglobal_pose2 = [median_zmp.inertial(:,ii);zc;0;0;mean([footprints_poses.inertial(6,ii),footprints_poses.inertial(6,ii+1)])];
    
    % Change global reference to foot reference
    DSpose1 = pose_tr2rpy(pose_rpy2tr(footprints_poses.inertial(:,ii))\pose_rpy2tr(DSglobal_pose1));
    DSpose2 = pose_tr2rpy(pose_rpy2tr(footprints_poses.inertial(:,ii))\pose_rpy2tr(DSglobal_pose2));
    
    lambdaI = DSpose1(1)*lambda;
    lambdaF = DSpose2(1)*lambda;

    betaI = DSpose1(2)*beta;
    %betaF = DSpose1(2)*beta;   
    betaF = DSpose2(2)*beta;
    
    
    [cog_traj_step, dcog_traj_step, ddcog_traj_step] = com_traj_one_step(lambdaI, lambdaF, betaI, betaF, DSpose1, DSpose2, footprints_poses.inertial(:,ii), zc, g, Ts, t0, TDS1, TSS, TDS2);  
    
    % Convert foot reference to global reference
    cog_traj_step.data = transform_frame(pose_rpy2tr(footprints_poses.inertial(:,ii)), cog_traj_step.data);
%     dcog_traj_step.data = transform_frame(pose_rpy2tr(footprints_poses.inertial(:,ii)), dcog_traj_step.data);
%     ddcog_traj_step.data = transform_frame(pose_rpy2tr(footprints_poses.inertial(:,ii)), ddcog_traj_step.data);
    
    if ii == 2
        cog_traj = cog_traj_step;
        dcog_traj = dcog_traj_step;
        ddcog_traj = ddcog_traj_step;
    else
        cog_traj = combine_2_traj(cog_traj, cog_traj_step);
        dcog_traj = combine_2_traj(dcog_traj, dcog_traj_step);
        ddcog_traj = combine_2_traj(ddcog_traj, ddcog_traj_step);
    end
    
    t0 = TDS2;
    
end

end

function [cog_traj, dcog_traj, ddcog_traj] = com_traj_one_step(lambdaI, lambdaF, betaI, betaF, DSpose1, DSpose2, footprint_pose, zc, g, Ts, t0, tDS1, tSS, tDS2)
% Simple Support Trajectory
% lambdaI
% lambdaF
% (tSS-tDS1)
[ppSS, dppSS] = cog_3d_LIPM(lambdaI, lambdaF, betaI, betaF, zc, g, round_to_Ts(tSS-tDS1,Ts), Ts);

% figure(1),plot(ppSS(1,:))
% figure(2),plot(dppSS(1,:))


% pos_ppSS = [ppSS(1,:)+footprint_pose(1);ppSS(2,:)+footprint_pose(2);ppSS(3,:)+footprint_pose(3)];
CoGorientation = zeros(3,size(ppSS,2));
ppSS = [ppSS; CoGorientation];

% Convert foot reference to global reference
% ppSS = transform_frame(pose_rpy2tr(footprint_pose), pose_SS);

dppSS = [dppSS; zeros(3,size(dppSS,2))];
ddppSS = zeros(size(ppSS));

tt = tDS1:Ts:tSS;       
trajSS = create_trajectory_structure(ppSS, Ts, tt);
dtrajSS = create_trajectory_structure(dppSS, Ts, tt);
ddtrajSS = create_trajectory_structure(ddppSS, Ts, tt);

% First Double Support (Before Simple Support Trajectory)
Pds0 = set_trajectory_condition(t0, DSpose1, zeros(6,1), zeros(6,1));
Pds1 = set_trajectory_condition(tDS1, trajSS.data(:,1), dtrajSS.data(:,1), zeros(6,1));
PDS  = [Pds0 Pds1];
[trajDS1, dtrajDS1, ddtrajDS1] = poly5_segments_trajectory (PDS, Ts);
cog_traj = combine_2_traj(trajDS1, trajSS);
dcog_traj = combine_2_traj(dtrajDS1, dtrajSS);
ddcog_traj = combine_2_traj(ddtrajDS1, ddtrajSS);


% Second Double Support (After Simple Support Trajectory)
% tDS2 = 0:Ts:TDS;
% [trajDS2, dtrajDS2, ddtrajDS2] = poly5_traj(dtrajSS(:,end), [0;0;0], [0;0;0], [0;0;0]);
Pds0 = set_trajectory_condition(tSS, trajSS.data(:,end), dtrajSS.data(:,end), zeros(6,1));
Pds1 = set_trajectory_condition(tDS2, DSpose2, zeros(6,1), zeros(6,1));
PDS  = [Pds0 Pds1];
[trajDS2, dtrajDS2, ddtrajDS2] = poly5_segments_trajectory (PDS, Ts);
cog_traj = combine_2_traj(cog_traj, trajDS2);
dcog_traj = combine_2_traj(dcog_traj, dtrajDS2);
ddcog_traj = combine_2_traj(ddcog_traj, ddtrajDS2);

% figure(2),plot(cog_traj.time,cog_traj.data(1,:))
% figure(3),plot(cog_traj.time,cog_traj.data(2,:))
% figure(4),plot(cog_traj.data(1,:),cog_traj.data(2,:))

end

function combined_traj = combine_2_traj(traj1,traj2)
    combined_traj.data = [traj1.data(:,1:end-1) traj2.data(:,:)];
    combined_traj.Ts =  traj1.Ts;
    combined_traj.time = [traj1.time(:,1:end-1) traj2.time(:,:)];
    combined_traj.T = traj1.T+traj2.T;
end

function posesT = transform_frame(T, poses)
posesT = zeros(size(poses));
for ii=1:size(poses,2),
    posesT(:,ii) = pose_tr2rpy(T*pose_rpy2tr(poses(:,ii)));
end
end
