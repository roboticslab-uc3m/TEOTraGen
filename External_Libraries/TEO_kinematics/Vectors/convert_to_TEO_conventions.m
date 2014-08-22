% Adapt to TEO conventions
function kinematics = convert_to_TEO_conventions (TEO, legs_kinematics, torso_kinematics, arms_kinematics)

% Convert standard DH variable names to specific TEO variables names
% Right Leg
w_T_RF = subs(legs_kinematics.right.floating.T, legs_kinematics.right.q, [TEO.legs.right.joint.q]);
w_J_RF = subs(legs_kinematics.right.floating.J, legs_kinematics.right.q, [TEO.legs.right.joint.q]);
RF_T_w = subs(legs_kinematics.right.standing.T, legs_kinematics.right.q, [TEO.legs.right.joint.q]);
RF_J_w = subs(legs_kinematics.right.standing.J, legs_kinematics.right.q, [TEO.legs.right.joint.q]);

% Left Leg
w_T_LF = subs(legs_kinematics.left.floating.T,  legs_kinematics.left.q,  [TEO.legs.left.joint.q]);
w_J_LF = subs(legs_kinematics.left.floating.J,  legs_kinematics.left.q,  [TEO.legs.left.joint.q]);
LF_T_w = subs(legs_kinematics.left.standing.T,  legs_kinematics.left.q,  [TEO.legs.left.joint.q]);
LF_J_w = subs(legs_kinematics.left.standing.J,  legs_kinematics.left.q,  [TEO.legs.left.joint.q]);

% Chest
% Hoap-3 does not have a Yaw joint in the Torso
% torso_kinematics_hoap = delete_kinematic_joint (torso_kinematics, 1);
w_T_CoM = subs(torso_kinematics.T, torso_kinematics.q, [TEO.waist.joint.q]);
w_J_CoM = subs(torso_kinematics.J, torso_kinematics.q, [TEO.waist.joint.q]);

% Arms
% Hoap-3 does not have a Pitch joint in the Wrist
% right_arm_kinematics_hoap = delete_kinematic_joint (arms_kinematics.right, 6);
% left_arm_kinematics_hoap  = delete_kinematic_joint (arms_kinematics.left, 6);

CoM_T_RH = subs(arms_kinematics.right.T, arms_kinematics.right.q, [TEO.arms.right.joint.q]);
CoM_J_RH = subs(arms_kinematics.right.J, arms_kinematics.right.q, [TEO.arms.right.joint.q]);
CoM_T_LH = subs(arms_kinematics.left.T,  arms_kinematics.left.q,  [TEO.arms.left.joint.q]);
CoM_J_LH = subs(arms_kinematics.left.J,  arms_kinematics.left.q,  [TEO.arms.left.joint.q]);

% Hoap 3 Pitch Shoulder (left and right) joints are initialized to 90 in the robot
% CoM_T_RH = subs(CoM_T_RH, TEO.arms.right.joint(1).q, TEO.arms.right.joint(1).q + TEO.arms.right.joint(1).offset);
% CoM_J_RH = subs(CoM_J_RH, TEO.arms.right.joint(1).q, TEO.arms.right.joint(1).q + TEO.arms.right.joint(1).offset);
% CoM_T_LH = subs(CoM_T_LH, TEO.arms.left.joint(1).q,  TEO.arms.left.joint(1).q  + TEO.arms.left.joint(1).offset);
% CoM_J_LH = subs(CoM_J_LH, TEO.arms.left.joint(1).q,  TEO.arms.left.joint(1).q  + TEO.arms.left.joint(1).offset);

% Center of Mass Position wrt to Feet
RF_T_CoM = multiply_homogeneous_matrix({RF_T_w,w_T_CoM});
RF_J_CoM = [RF_J_w, w_J_CoM];

LF_T_CoM = multiply_homogeneous_matrix({LF_T_w,w_T_CoM});
LF_J_CoM = [LF_J_w, w_J_CoM];

CoM_T_RF = invert_homogeneous_matrix(RF_T_CoM);
CoM_T_LF = invert_homogeneous_matrix(LF_T_CoM);

% Joints vector
% % n = 23 dof: 6 (right leg) + 6 (left leg) + 1 (body) + 5 (right arm) + 5
% % (left arm) ---> HOAP3

%n = 26 dof: 6 (right leg) + 6 (left leg) + 2 (body) + 6 (right arm) + 6
% (left arm)

joints = transpose([TEO.legs.right.joint.q, TEO.legs.left.joint.q, TEO.waist.joint.q, TEO.arms.right.joint.q, TEO.arms.left.joint.q]);

% Generate Full Kinematics Structs
jj=1;
kinematics(jj).T.value = w_T_RF;
kinematics(jj).T.name  = 'w_T_RF';
kinematics(jj).J.value = w_J_RF;
kinematics(jj).J.name  = 'w_J_RF';
kinematics(jj).var     = joints;

jj = jj+1;
kinematics(jj).T.value = RF_T_w;
kinematics(jj).T.name  = 'RF_T_w';
kinematics(jj).J.value = RF_J_w;
kinematics(jj).J.name  = 'RF_J_w';
kinematics(jj).var     = joints;

jj = jj+1;
kinematics(jj).T.value = w_T_LF;
kinematics(jj).T.name  = 'w_T_LF';
kinematics(jj).J.value = w_J_LF;
kinematics(jj).J.name  = 'w_J_LF';
kinematics(jj).var     = joints;

jj = jj+1;
kinematics(jj).T.value = LF_T_w;
kinematics(jj).T.name  = 'LF_T_w';
kinematics(jj).J.value = LF_J_w;
kinematics(jj).J.name  = 'LF_J_w';
kinematics(jj).var     = joints;

jj = jj+1;
kinematics(jj).T.value = w_T_CoM;
kinematics(jj).T.name  = 'w_T_CoM';
kinematics(jj).J.value = w_J_CoM;
kinematics(jj).J.name  = 'w_J_CoM';
kinematics(jj).var     = joints;

jj = jj+1;
kinematics(jj).T.value = CoM_T_RH;
kinematics(jj).T.name  = 'CoM_T_RH';
kinematics(jj).J.value = CoM_J_RH;
kinematics(jj).J.name  = 'CoM_J_RH';
kinematics(jj).var     = joints;

jj = jj+1;
kinematics(jj).T.value = CoM_T_LH;
kinematics(jj).T.name  = 'CoM_T_LH';
kinematics(jj).J.value = CoM_J_LH;
kinematics(jj).J.name  = 'CoM_J_LH';
kinematics(jj).var     = joints;

jj = jj+1;
kinematics(jj).T.value = RF_T_CoM;
kinematics(jj).T.name  = 'RF_T_CoM';
kinematics(jj).J.value = RF_J_CoM;
kinematics(jj).J.name  = 'RF_J_CoM';
kinematics(jj).var     = joints;

jj = jj+1;
kinematics(jj).T.value = LF_T_CoM;
kinematics(jj).T.name  = 'LF_T_CoM';
kinematics(jj).J.value = LF_J_CoM;
kinematics(jj).J.name  = 'LF_J_CoM';
kinematics(jj).var     = joints;

jj = jj+1;
kinematics(jj).T.value = CoM_T_RF;
kinematics(jj).T.name = 'CoM_T_RF';
kinematics(jj).var     = joints;

jj = jj+1;
kinematics(jj).T.value = CoM_T_LF;
kinematics(jj).T.name = 'CoM_T_LF';
kinematics(jj).var     = joints;
end