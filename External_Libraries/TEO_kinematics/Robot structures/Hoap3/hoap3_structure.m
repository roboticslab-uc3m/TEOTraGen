function robot = hoap3_structure(mode, angle_unit, length_unit)
%   theta01 theta02 theta03 theta04 theta05 theta06     RIGHT LEG
%   theta07 theta08 theta09 theta10 theta11 theta12     LEFT LEG
%   theta13                                             WAIST
%   theta14 theta15 theta16 theta17 theta18             RIGHT ARM
%   theta19 theta20 theta21 theta22 theta23             LEFT ARM

% Generalized variables with their first and second derivatives
q   = generate_symbolic_vector('theta', 23);
dq  = time_derivate_symbolic_variable(q);
ddq = time_derivate_symbolic_variable(dq);

%%%%%%%%%%%%%%%%
% Robot Joints %
%%%%%%%%%%%%%%%%

% Right Leg
legs.right.name = 'Right Leg';
legs.right.joint(1).name = 'Hip Yaw';
legs.right.joint(1).id   = 1;
legs.right.joint(1).type = 'r';
legs.right.joint(1).q    = q(1);
legs.right.joint(1).dq   = dq(1);
legs.right.joint(1).ddq  = ddq(1);

legs.right.joint(2).name = 'Hip Roll';
legs.right.joint(2).id   = 2;
legs.right.joint(2).type = 'r';
legs.right.joint(2).q    = q(2);
legs.right.joint(2).dq   = dq(2);
legs.right.joint(2).ddq  = ddq(2);

legs.right.joint(3).name = 'Hip Pitch';
legs.right.joint(3).id   = 3;
legs.right.joint(3).type = 'r';
legs.right.joint(3).q    = q(3);
legs.right.joint(3).dq   = dq(3);
legs.right.joint(3).ddq  = ddq(3);

legs.right.joint(4).name = 'Knee Pitch';
legs.right.joint(4).id   = 4;
legs.right.joint(4).type = 'r';
legs.right.joint(4).q    = q(4);
legs.right.joint(4).dq   = dq(4);
legs.right.joint(4).ddq  = ddq(4);

legs.right.joint(5).name = 'Ankle Pitch';
legs.right.joint(5).id   = 5;
legs.right.joint(5).type = 'r';
legs.right.joint(5).q    = q(5);
legs.right.joint(5).dq   = dq(5);
legs.right.joint(5).ddq  = ddq(5);

legs.right.joint(6).name = 'Ankle Roll';
legs.right.joint(6).id   = 6;
legs.right.joint(6).type = 'r';
legs.right.joint(6).q    = q(6);
legs.right.joint(6).dq   = dq(6);
legs.right.joint(6).ddq  = ddq(6);

% Left Leg
legs.left.name = 'Left Leg';
legs.left.joint(1).name = 'Hip Yaw';
legs.left.joint(1).id   = 11;
legs.left.joint(1).type = 'r';
legs.left.joint(1).q    = q(7);
legs.left.joint(1).dq   = dq(7);
legs.left.joint(1).ddq  = ddq(7);

legs.left.joint(2).name = 'Hip Roll';
legs.left.joint(2).id   = 12;
legs.left.joint(2).type = 'r';
legs.left.joint(2).q    = q(8);
legs.left.joint(2).dq   = dq(8);
legs.left.joint(2).ddq  = ddq(8);

legs.left.joint(3).name = 'Hip Pitch';
legs.left.joint(3).id   = 13;
legs.left.joint(3).type = 'r';
legs.left.joint(3).q    = q(9);
legs.left.joint(3).dq   = dq(9);
legs.left.joint(3).ddq  = ddq(9);

legs.left.joint(4).name = 'Knee Pitch';
legs.left.joint(4).id   = 14;
legs.left.joint(4).type = 'r';
legs.left.joint(4).q    = q(10);
legs.left.joint(4).dq   = dq(10);
legs.left.joint(4).ddq  = ddq(10);

legs.left.joint(5).name = 'Ankle Pitch';
legs.left.joint(5).id   = 15;
legs.left.joint(5).type = 'r';
legs.left.joint(5).q    = q(11);
legs.left.joint(5).dq   = dq(11);
legs.left.joint(5).ddq  = ddq(11);

legs.left.joint(6).name = 'Ankle Roll';
legs.left.joint(6).id   = 16;
legs.left.joint(6).type = 'r';
legs.left.joint(6).q    = q(12);
legs.left.joint(6).dq   = dq(12);
legs.left.joint(6).ddq  = ddq(12);

% Right Arm
arms.right.name = 'Right Arm';
arms.right.joint(1).name = 'Shoulder Pitch';
arms.right.joint(1).id   = 7;
arms.right.joint(1).type = 'r';
arms.right.joint(1).q    = q(14);
arms.right.joint(1).dq   = dq(14);
arms.right.joint(1).ddq  = ddq(14);

arms.right.joint(2).name = 'Shoulder Roll';
arms.right.joint(2).id   = 8;
arms.right.joint(2).type = 'r';
arms.right.joint(2).q    = q(15);
arms.right.joint(2).dq   = dq(15);
arms.right.joint(2).ddq = ddq(15);

arms.right.joint(3).name = 'Shoulder Yaw';
arms.right.joint(3).id   = 9;
arms.right.joint(3).type = 'r';
arms.right.joint(3).q    = q(16);
arms.right.joint(3).dq   = dq(16);
arms.right.joint(3).ddq  = ddq(16);

arms.right.joint(4).name = 'Elbow Pitch';
arms.right.joint(4).id   = 10;
arms.right.joint(4).type = 'r';
arms.right.joint(4).q    = q(17);
arms.right.joint(4).dq   = dq(17);
arms.right.joint(4).ddq  = ddq(17);

arms.right.joint(5).name = 'Wrist Pitch';
arms.right.joint(5).id   = 23;
arms.right.joint(5).type = 'r';
arms.right.joint(5).q    = q(18);
arms.right.joint(5).dq   = dq(18);
arms.right.joint(5).ddq  = ddq(18);

% Left Arm
arms.left.name = 'Left Arm';
arms.left.joint(1).name = 'Shoulder Pitch';
arms.left.joint(1).id   = 17;
arms.left.joint(1).type = 'r';
arms.left.joint(1).q    = q(19);
arms.left.joint(1).dq   = dq(19);
arms.left.joint(1).ddq  = ddq(19);

arms.left.joint(2).name = 'Shoulder Roll';
arms.left.joint(2).id   = 18;
arms.left.joint(2).type = 'r';
arms.left.joint(2).q    = q(20);
arms.left.joint(2).dq   = dq(20);
arms.left.joint(2).ddq  = ddq(20);

arms.left.joint(3).name = 'Shoulder Yaw';
arms.left.joint(3).id   = 19;
arms.left.joint(3).type = 'r';
arms.left.joint(3).q    = q(21);
arms.left.joint(3).dq   = dq(21);
arms.left.joint(3).ddq  = ddq(21);

arms.left.joint(4).name = 'Elbow Pitch';
arms.left.joint(4).id   = 20;
arms.left.joint(4).type = 'r';
arms.left.joint(4).q    = q(22);
arms.left.joint(4).dq   = dq(22);
arms.left.joint(4).ddq  = ddq(22);

arms.left.joint(5).name = 'Wrist Pitch';
arms.left.joint(5).id   = 23;
arms.left.joint(5).type = 'r';
arms.left.joint(5).q    = q(23);
arms.left.joint(5).dq   = dq(23);
arms.left.joint(5).ddq  = ddq(23);

% Waist
waist.joint(1).name = 'Waist Pitch';
waist.joint(1).id   = 21;
waist.joint(1).type = 'r';
waist.joint(1).q    = q(13);
waist.joint(1).dq   = dq(13);
waist.joint(1).ddq  = ddq(13);

if strcmpi(mode, 'symbolic')
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Robot Links Dimensions (Symbolic) %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    disp('Symbolic data loaded')
    legs.link_lengths = generate_symbolic_vector('LEG_LINK', 5);
    legs.link_lengths(4) = 0;
    arms.link_lengths = generate_symbolic_vector('ARM_LINK', 3);
    waist.link_lengths = generate_symbolic_vector('WAIST_LINK', 2);
    torso.link_lengths = generate_symbolic_vector('TORSO_LINK', 2);
    chest.link_lengths = sym(zeros(2, 1));
    chest.link_lengths(2) = sym('CHEST_LINK2','real');
    head.link_lengths = generate_symbolic_vector('HEAD_LINK', 2);
else
    DEGREE2RAD = pi/180;
    M2MM = 1000;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Robot Links Length (numeric) %
    % Default in mm                %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Legs
    legs.link_lengths = zeros(5, 1);
    legs.link_lengths(1) =  39;
    legs.link_lengths(2) = 105;
    legs.link_lengths(3) = 105;
    legs.link_lengths(4) = 0;
    legs.link_lengths(5) = 40;
    
    % Arms
    arms.link_lengths = zeros(3, 1);
    arms.link_lengths(1) = 111;
    arms.link_lengths(2) = 111;
    arms.link_lengths(3) = 171;
   
    % Waist
    waist.link_lengths = zeros(2, 1);
    waist.link_lengths(1) = 35;
    waist.link_lengths(2) = 55;
    
    % Torso
    torso.link_lengths = zeros(2, 1);
    torso.link_lengths(1) = 35;
    torso.link_lengths(2) = 70;

    % Chest
    chest.link_lengths = zeros(2, 1);
    chest.link_lengths(1) = 0;
    chest.link_lengths(2) = 55;

    % Head
    head.link_lengths = zeros(2, 1);
    head.link_lengths(1) = 103;
    head.link_lengths(2) = 15;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Robot Joints position           %
    % w.r.t. the corresponding parent %
    % Default in mm                   %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % I changed this on 26/09/2011 for Simmechanics: CHECK FOR DYNAMICS IN MATLAB
    % The Origin is the Waist
    origin = [0; 0; sum(legs.link_lengths([2,3,5])) + waist.link_lengths(2)];

    % The x, y, z are referred to the parent joint
    ZERO = zeros(3, 1);
    ID = eye(3);   

    % Waist joint 1
    % Rototranslation w.r.t. the world
    waist.joint(1).origin = origin + [torso.link_lengths(1); 0; 0];
    waist.joint(1).orientation = [1, 0, 0; 0, 0, 1; 0, -1, 0];
    waist.joint(1).axis = [0, 1, 0];
    
    % Right Leg
    % Rototranslation w.r.t. the waist joint    
    legs.right.joint(1).origin  = waist.joint(1).origin + [-waist.link_lengths(1); -legs.link_lengths(1); -waist.link_lengths(2)];
    legs.right.joint(2).origin  = legs.right.joint(1).origin;
    legs.right.joint(3).origin  = legs.right.joint(2).origin;
    legs.right.joint(4).origin  = legs.right.joint(3).origin + [0; 0; -legs.link_lengths(2)];
    legs.right.joint(5).origin  = legs.right.joint(4).origin + [0; 0; -legs.link_lengths(3)];
    legs.right.joint(6).origin  = legs.right.joint(5).origin;
    
    legs.right.joint(1).orientation  = ID;
    legs.right.joint(2).orientation  = [0, 0, 1; 1, 0, 0; 0, 1, 0];
    legs.right.joint(3).orientation  = [0, -1, 0; 0, 0, 1; -1, 0, 0];
    legs.right.joint(4).orientation  = [0, -1, 0; 0, 0, 1; -1, 0, 0];
    legs.right.joint(5).orientation  = [0, -1, 0; 0, 0, 1; -1, 0, 0];
    legs.right.joint(6).orientation  = [0, 0, 1; 0, 1, 0; -1, 0, 0];
    
    legs.right.joint(1).axis = [0, 0, 1];
    legs.right.joint(2).axis = [1, 0, 0];
    legs.right.joint(3).axis = [0, 1, 0];
    legs.right.joint(4).axis = [0, 1, 0];
    legs.right.joint(5).axis = [0, 1, 0];
    legs.right.joint(6).axis = [1, 0, 0];
    
    % Right Foot
    % Rototranslation w.r.t. the robot ankle
    legs.right.foot.origin = legs.right.joint(6).origin + [0; 0; -legs.link_lengths(5)];
    legs.right.foot.orientation = ID;
    legs.right.foot.limits.x    = [-40, 68];
    legs.right.foot.limits.y    = [-41.5, 31.5];
    
    % Left Leg
    % Rototranslation w.r.t. the waist joint
    legs.left.joint(1).origin  = waist.joint(1).origin + [-waist.link_lengths(1); legs.link_lengths(1); -waist.link_lengths(2)];
    legs.left.joint(2).origin  = legs.left.joint(1).origin;
    legs.left.joint(3).origin  = legs.left.joint(2).origin;
    legs.left.joint(4).origin  = legs.left.joint(3).origin + [0; 0; -legs.link_lengths(2)];
    legs.left.joint(5).origin  = legs.left.joint(4).origin + [0; 0; -legs.link_lengths(3)];
    legs.left.joint(6).origin  = legs.left.joint(5).origin;
       
    for jj = 1:6
        legs.left.joint(jj).orientation = legs.right.joint(jj).orientation;
        legs.left.joint(jj).axis = legs.right.joint(jj).axis;
    end
    
    % Left Foot
    % Rototranslation w.r.t. the robot ankle
    legs.left.foot.origin  = legs.left.joint(6).origin + [0; 0; -legs.link_lengths(5)];
    legs.left.foot.orientation  = legs.right.foot.orientation;
    legs.left.foot.limits.x     = [-40, 68];
    legs.left.foot.limits.y     = [-31.5, 41.5];

    % IMU
    % Rototranslation w.r.t. the robot center
    waist.IMU.origin = origin + [0; 0; torso.link_lengths(2)];
    waist.IMU.orientation = ID;
    
    % Right Arm
    % Rototranslation w.r.t. the waist joint
    arms.right.joint(1).origin  = waist.joint(1).origin + [-torso.link_lengths(1) - chest.link_lengths(1); -arms.link_lengths(1); chest.link_lengths(2) + torso.link_lengths(2)];
    arms.right.joint(2).origin  = arms.right.joint(1).origin;
    arms.right.joint(3).origin  = arms.right.joint(2).origin;
    arms.right.joint(4).origin  = arms.right.joint(3).origin + [arms.link_lengths(2); 0; 0];
    arms.right.joint(5).origin  = arms.right.joint(4).origin + [arms.link_lengths(3); 0; 0];

    arms.right.joint(1).orientation  = [1, 0, 0; 0, 0, 1; 0, -1, 0];
    arms.right.joint(2).orientation  = [0, -1, 0; 1, 0, 0; 0, 0, 1];
    arms.right.joint(3).orientation  = [0, 0, 1; 0, -1, 0; 1, 0, 0];
    arms.right.joint(4).orientation  = [0, 1, 0; 0, 0, 1; 1, 0, 0];
    arms.right.joint(5).orientation  = [0, 0, 1; 0, -1, 0; 1, 0, 0];
    
    arms.right.joint(1).axis = [0, 1, 0];
    arms.right.joint(2).axis = [0, 0, 1];
    arms.right.joint(3).axis = [1, 0, 0];
    arms.right.joint(4).axis = [0, 1, 0];
    arms.right.joint(5).axis = [1, 0, 0];
    
    
    % Left Arm
    % Rototranslation w.r.t. the waist joint
    arms.left.joint(1).origin = waist.joint(1).origin + [-torso.link_lengths(1) - chest.link_lengths(1); arms.link_lengths(1); chest.link_lengths(2) + torso.link_lengths(2)];
    arms.left.joint(2).origin  = arms.left.joint(1).origin;
    arms.left.joint(3).origin  = arms.left.joint(2).origin;
    arms.left.joint(4).origin  = arms.left.joint(3).origin + [arms.link_lengths(2); 0; 0];
    arms.left.joint(5).origin  = arms.left.joint(4).origin + [arms.link_lengths(3); 0; 0];
    
    for jj = 1:5
        arms.left.joint(jj).orientation = arms.right.joint(jj).orientation;
        arms.left.joint(jj).axis = arms.right.joint(jj).axis;
    end 
    
    % Head
    % Rototranslation w.r.t. the waist joint
    head.joint(1).origin = waist.joint(1).origin + [-torso.link_lengths(1) - chest.link_lengths(1); 0; chest.link_lengths(2) + torso.link_lengths(2) + head.link_lengths(1)];
    head.joint(1).orientation  = ID;    
    head.joint(1).axis = [0, 0, 1];
    
    head.joint(2).origin = head.joint(1).origin + [head.link_lengths(2); 0; 0];
    head.joint(2).orientation  = [1, 0, 0; 0, 0, 1; 0, -1, 0];
    head.joint(2).axis = [0, 1, 0];
    

    %%%%%%%%%%%%%%%%%%%%%%%%%%
    % Motors Characteristics %
    % Default angle: Radians %
    % Default length: mm     %
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Right Leg
    legs.right.joint(1).angle_limits  = [-91, 31] * DEGREE2RAD;
    legs.right.joint(1).torque_limits = [1.5, 3] * M2MM^2;
    legs.right.joint(1).relation      = 209 / DEGREE2RAD;

    legs.right.joint(2).angle_limits  = [-31, 21] * DEGREE2RAD;
    legs.right.joint(2).torque_limits = [2, 4.5] * M2MM^2;
    legs.right.joint(2).relation      = 209 / DEGREE2RAD;

    legs.right.joint(3).angle_limits  = [-82, 71] * DEGREE2RAD;
    legs.right.joint(3).torque_limits = [1.5, 3] * M2MM^2;
    legs.right.joint(3).relation      = -209 / DEGREE2RAD;

    legs.right.joint(4).angle_limits  = [-1, 130] * DEGREE2RAD;
    legs.right.joint(4).torque_limits = [2, 4.5] * M2MM^2;
    legs.right.joint(4).relation      = 209 / DEGREE2RAD;

    legs.right.joint(5).angle_limits  = [-61, 61] * DEGREE2RAD;
    legs.right.joint(5).torque_limits = [2, 4.5] * M2MM^2;
    legs.right.joint(5).relation      = 209 / DEGREE2RAD;

    legs.right.joint(6).angle_limits  = [-25, 25] * DEGREE2RAD;
    legs.right.joint(6).torque_limits = [1.5, 3] * M2MM^2;
    legs.right.joint(6).relation      = -209 / DEGREE2RAD;

    % Left Leg
    legs.left.joint(1).angle_limits  = [-31, 91] * DEGREE2RAD;
    legs.left.joint(1).torque_limits = [1.5, 3] * M2MM^2;
    legs.left.joint(1).relation      = 209 / DEGREE2RAD;

    legs.left.joint(2).angle_limits  = [-21, 31] * DEGREE2RAD;
    legs.left.joint(2).torque_limits = [2, 4.5] * M2MM^2;
    legs.left.joint(2).relation      = 209 / DEGREE2RAD;

    legs.left.joint(3).angle_limits  = [-82, 71] * DEGREE2RAD;
    legs.left.joint(3).torque_limits = [1.5, 3] * M2MM^2;
    legs.left.joint(3).relation      = 209 / DEGREE2RAD;

    legs.left.joint(4).angle_limits  = [-1, 130] * DEGREE2RAD;
    legs.left.joint(4).torque_limits = [2, 4.5] * M2MM^2;
    legs.left.joint(4).relation      = -209 / DEGREE2RAD;

    legs.left.joint(5).angle_limits  = [-61, 61] * DEGREE2RAD;
    legs.left.joint(5).torque_limits = [2, 4.5] * M2MM^2;
    legs.left.joint(5).relation      = -209 / DEGREE2RAD;

    legs.left.joint(6).angle_limits  = [-25, 25] * DEGREE2RAD;
    legs.left.joint(6).torque_limits = [1.5, 3] * M2MM^2;
    legs.left.joint(6).relation      = -209 / DEGREE2RAD;

    % Right Arm
    arms.right.joint(1).angle_limits  = [-91, 151] * DEGREE2RAD;
    arms.right.joint(1).torque_limits = [1.5, 3] * M2MM^2;
    arms.right.joint(1).relation      = 209 / DEGREE2RAD;
    % In the robot the 0 in the shoulder is -90º w.r.t. to standard convention
    arms.right.joint(1).offset        = -pi/2;

    arms.right.joint(2).angle_limits  = [-96, 1] * DEGREE2RAD;
    arms.right.joint(2).torque_limits = [1.5, 3] * M2MM^2;
    arms.right.joint(2).relation      = 209 / DEGREE2RAD;

    arms.right.joint(3).angle_limits  = [-91, 91] * DEGREE2RAD;
    arms.right.joint(3).torque_limits = [1.5, 3] * M2MM^2;
    arms.right.joint(3).relation      = 209 / DEGREE2RAD;

    arms.right.joint(4).angle_limits  = [-115, 1] * DEGREE2RAD;
    arms.right.joint(4).torque_limits = [1.5, 3] * M2MM^2;
    arms.right.joint(4).relation      = -209 / DEGREE2RAD;

    arms.right.joint(5).angle_limits  = [-60, 60] * DEGREE2RAD;
    arms.right.joint(5).torque_limits = [0.28, 0.36] * M2MM^2;
    arms.right.joint(5).relation      = 0;

    % Left Arm
    arms.left.joint(1).angle_limits  = [-91, 151] * DEGREE2RAD;
    arms.left.joint(1).torque_limits = [1.5, 3] * M2MM^2;
    arms.left.joint(1).relation      = -209 / DEGREE2RAD;
    % In the robot the 0 in the shoulder is -90º w.r.t. to standard convention
    arms.left.joint(1).offset        = -pi/2;

    arms.left.joint(2).angle_limits  = [-1, 96] * DEGREE2RAD;
    arms.left.joint(2).torque_limits = [1.5, 3] * M2MM^2;
    arms.left.joint(2).relation      = 209 / DEGREE2RAD;

    arms.left.joint(3).angle_limits  = [-91, 91] * DEGREE2RAD;
    arms.left.joint(3).torque_limits = [1.5, 3] * M2MM^2;
    arms.left.joint(3).relation      = 209 / DEGREE2RAD;

    arms.left.joint(4).angle_limits  = [-115, 1] * DEGREE2RAD;
    arms.left.joint(4).torque_limits = [1.5, 3] * M2MM^2;
    arms.left.joint(4).relation      = 209 / DEGREE2RAD;

    arms.left.joint(5).angle_limits  = [-60, 60] * DEGREE2RAD;
    arms.left.joint(5).torque_limits = [0.28, 0.36] * M2MM^2;
    arms.left.joint(5).relation      = 0;

    % Waist
    waist.joint(1).angle_limits      = [-1, 90] * DEGREE2RAD;
    waist.joint(1).torque_limits     = [2, 4.5] * M2MM^2;
    waist.joint(1).relation          = 209 / DEGREE2RAD;    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Mass/Inertia Properties of Hoap-3 %
    % Default angle: Radians            %
    % Default length: mm                %   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % The link jj is the link corresponding to the jj joint
    % The positions and Inertias are referred to the corresponding joint
    % w.r.t. its coordinate frame
    
    % Right Leg
    legs.right.link(1).mass = 5.87e-02;
    R1 = [0, -1, 0; 1, 0, 0; 0, 0, 1];
    legs.right.link(1).CoM = R1 * [-3.01500; -2.2399; 3.35846e+01];
    legs.right.link(1).inertia = rotate_inertia_matrix([5.98251e+01, 4.07692e-01, 5.70458e+00; 4.07692e-01, 4.32046e+01,-2.93763e+00; 5.70458e+00,-2.93763e+00, 2.95761e+01], R1);
    
    legs.right.link(2).mass = 2.10e-01;
    R2 = [0, -1, 0; 1, 0, 0; 0, 0, 1];
    legs.right.link(2).CoM = R2 * [-2.72697; 1.13482; 2.37780];
    legs.right.link(2).inertia = rotate_inertia_matrix([6.94143e+01,-8.93597e-01, 4.50011e-01; -8.93597e-01, 5.87096e+01, 5.13879e-01; 4.50011e-01, 5.13879e-01, 5.47365e+01], R2);

    legs.right.link(3).mass = 5.19e-01;
    R3 = [-1, 0, 0; 0, -1, 0; 0, 0, 1];
    legs.right.link(3).CoM = R3 * [-6.98208e+01; 6.16508; -3.26392];
    legs.right.link(3).inertia = rotate_inertia_matrix([3.33437e+02,-1.04660e+01, 2.86176e+01; -1.04660e+01, 7.15435e+02,-1.85334e+01; 2.86176e+01,-1.85334e+01, 6.69659e+02], R3);

    legs.right.link(4).mass = 3.28e-01;
    R4 = R3;
    legs.right.link(4).CoM = R4 * [-5.08521e+01; 5.36785; -4.69750];
    legs.right.link(4).inertia = rotate_inertia_matrix([2.14004e+02,-2.63054e+01,-1.52711e+01; -2.63054e+01, 3.34738e+02,-9.48167; -1.52711e+01,-9.48167, 2.73483e+02], R4);

    legs.right.link(5).mass = 1.95e-01;
    R5 = R3;
    legs.right.link(5).CoM = R5 * [2.93669; 3.66853; -1.33746];
    legs.right.link(5).inertia = rotate_inertia_matrix([6.35459e+01, 2.96980e-01,-5.85740e-01; 2.96980e-01, 5.29750e+01,-9.54911e-01; -5.85740e-01,-9.54911e-01, 5.27906e+01], R5);

    legs.right.link(6).mass = 1.54e-01;
    R6 = R3;
    legs.right.link(6).CoM = R6 * [-2.92236e+01; 3.24167; 8.27642];
    legs.right.link(6).inertia = rotate_inertia_matrix([2.19262e+02, 3.30179, 1.63411e+01; 3.30179, 1.78310e+02,-3.47988; 1.63411e+01,-3.47988, 8.20349e+01], R6);

    % Left Leg
    legs.left.link(1).mass = 5.87e-02;
    R1 = [0, -1, 0; 1, 0, 0; 0, 0, 1];
    legs.left.link(1).CoM = R1 * [3.00449; -2.25088; 3.35846e+01];
    legs.left.link(1).inertia = rotate_inertia_matrix([5.98222e+01,-3.85905e-01,-5.71594; -3.85905e-01, 4.32083e+01,-2.91155; -5.71594,-2.91155, 2.95769e+01], R1);
    
    legs.left.link(2).mass = 2.10e-01;
    R2 = [0, -1, 0; 1, 0, 0; 0, 0, 1];
    legs.left.link(2).CoM = R2 * [-2.71179; -1.24723; 2.38665];
    legs.left.link(2).inertia = rotate_inertia_matrix([6.93238e+01, 5.11967e-01, 4.43105e-01; 5.11967e-01, 5.85697e+01,-6.18331e-01; 4.43105e-01,-6.18331e-01, 5.45564e+01], R2);

    legs.left.link(3).mass = 5.19e-01;
    R3 = [-1, 0, 0; 0, -1, 0; 0, 0, 1];
    legs.left.link(3).CoM = R3 * [-6.99224e+01; 6.27642; 3.31502];
    legs.left.link(3).inertia = rotate_inertia_matrix([3.33124e+02,-1.04299e+01,-2.93471e+01; -1.04299e+01, 7.14361e+02, 2.11288e+01; -2.93471e+01, 2.11288e+01, 6.69044e+02], R3);
    
    legs.left.link(4).mass = 3.28e-01;
    R4 = R3;
    legs.left.link(4).CoM = R4 * [-5.08386e+01; 5.37031; 4.87773];
    legs.left.link(4).inertia = rotate_inertia_matrix([2.10989e+02,-2.62826e+01, 1.58357e+01; -2.62826e+01, 3.31772e+02, 8.60718; 1.58357e+01, 8.60718, 2.73517e+02], R4);
    
    legs.left.link(5).mass = 1.95e-01;
    R5 = R3;
    legs.left.link(5).CoM = R5 * [2.92057; 3.69219; 1.19731];
    legs.left.link(5).inertia = rotate_inertia_matrix([6.35542e+01, 2.67585e-01, 8.67380e-01; 2.67585e-01, 5.31460e+01, 8.92463e-01; 8.67380e-01, 8.92463e-01, 5.26905e+01], R5);
    
    legs.left.link(6).mass = 1.54e-01;
    R6 = R3;
    legs.left.link(6).CoM = R6 * [-2.92236e+01; -3.34620; 8.27642];
    legs.left.link(6).inertia = rotate_inertia_matrix([2.19156e+02,-2.72205, 1.63411e+01; -2.72205, 1.78310e+02, 2.84694; 1.63411e+01, 2.84694, 8.19287e+01], R6);
    
    % Hoap arms are rotated by 90º in shoulder joint
    %(check arms.right.joint(1).offset and arms.left.joint(1).offset)
    
    % Right Arm
    arms.right.link(1).mass = 2.10e-01;
    % R1 = [0, -1, 0; 1, 0, 0; 0, 0, 1];
    R1 = eye(3);
    arms.right.link(1).CoM = R1 * [8.28367e-02; 6.66808; 6.34824];
    arms.right.link(1).inertia = rotate_inertia_matrix([8.66688e+01, 6.58521e-02,-5.21141e-01; 6.58521e-02, 6.69963e+01, 3.24962; -5.21141e-01, 3.24962, 7.29623e+01], R1);

    arms.right.link(2).mass = 2.34e-01;
    % R2 = [0, -1, 0; 1, 0, 0; 0, 0, 1];
    R2 = eye(3);
    arms.right.link(2).CoM = R2 * [-6.57560e-01; -4.63848e+01; -3.77395];
    arms.right.link(2).inertia = rotate_inertia_matrix([1.90208e+02, 4.38002,-1.91098e-01; 4.38002, 9.55647e+01, 1.09480e+01; -1.91098e-01, 1.09480e+01, 1.66913e+02], R2);

    arms.right.link(3).mass = 1.965e-01;
    %R3 = [0, 1, 0; 1, 0, 0; 0, 0, -1];
    % Hoap-3 Manual joint 3 origin is in the elbow, here it is translated
    % in the shoulder
    %arms.right.link(3).CoM = R3 * [-2.72189; -1.60245; -2.20452] + [0; 0; -arms.link_lengths(2)];
	R3 = eye(3);
    arms.right.link(3).CoM = R3 * [-2.72189; -1.60245; -2.20452];
    arms.right.link(3).inertia =  rotate_inertia_matrix([6.49972e+01,-1.21159e-01, 1.21288; -1.21159e-01, 5.29142e+01, 6.02001e-01; 1.21288, 6.02001e-01, 6.80659e+01], R3);

    arms.right.link(4).mass = 2.16e-01;
    arms.right.link(4).CoM = [1.00237; 6.64225e+01; -6.15512e-01];
    arms.right.link(4).inertia = [3.60567e+02,-4.53239e-01, 8.95113e-01; -4.53239e-01, 1.03381e+02, 2.20497e+01; 8.95113e-01, 2.20497e+01, 3.36596e+02];

    arms.right.link(5).mass = 0;
    arms.right.link(5).CoM = zeros(3,1);
    arms.right.link(5).inertia = eye(3);

    % Left Arm
    arms.left.link(1).mass = 2.10e-01;
    %R1 = [0, -1, 0; 1, 0, 0; 0, 0, 1];
    R1 = eye(3);
    arms.left.link(1).CoM = R1 * [-3.64197e-02; 6.64174; -6.34825];
    arms.left.link(1).inertia = rotate_inertia_matrix([8.66760e+01, 8.54331e-02, 2.13651e-01; 8.54331e-02, 6.69975e+01,-3.14870; 2.13651e-01,-3.14870, 7.29706e+01], R1);

    arms.left.link(2).mass = 2.34e-01;
%    R2 = [0, -1, 0; 1, 0, 0; 0, 0, 1];
    R2 = eye(3);
    arms.left.link(2).CoM = R2 * [7.47218e-01; -4.63619e+01; -3.77403];
    arms.left.link(2).inertia = rotate_inertia_matrix([1.90701e+02,-4.53865, 4.72738e-01; -4.53865, 9.55349e+01, 1.08196e+01; 4.72738e-01, 1.08196e+01, 1.67377e+02], R2);
    
    arms.left.link(3).mass = 1.965e-01;
    %R3 = [0, 1, 0; 1, 0, 0; 0, 0, -1];
    % Hoap-3 Manual joint 3 origin is in the elbow, here it is translated
    % in the shoulder
    %arms.left.link(3).CoM = R3 * [-2.70991; 1.60227; -2.09019] + [0; 0; -arms.link_lengths(2)];
    R3 = eye(3);
    arms.left.link(3).CoM = R3 * [-2.70991; 1.60227; -2.09019];    
    arms.left.link(3).inertia = rotate_inertia_matrix([6.51094e+01, 1.24947e-01, 1.66316; 1.24947e-01, 5.30426e+01,-5.66321e-01; 1.66316,-5.66321e-01, 6.81049e+01], R3);
    
    arms.left.link(4).mass = 2.16e-01;
    arms.left.link(4).CoM = [9.62091e-01; 6.64766e+01; 5.58629e-01];
    arms.left.link(4).inertia = [3.59070e+02,-7.34962e-01,-7.26630e-01; -7.34962e-01, 1.03303e+02,-2.22301e+01; -7.26630e-01,-2.22301e+01, 3.35228e+02];
    
    arms.left.link(5).mass = 0;
    arms.left.link(5).CoM = zeros(3,1);
    arms.left.link(5).inertia = eye(3);
    
    % Waist
    % This is the link of the Hoap-3 body
    waist.link(1).mass = 3.26;
    R1 = eye(3);
    waist.link(1).CoM = R1 * [-4.94890e+01; 8.92912e-01; 8.41741e+01];
    waist.link(1).inertia = rotate_inertia_matrix([1.21947e+04, 2.95479e+02, 6.84001e+02; 2.95479e+02, 1.75644e+04, 1.45795e+02; 6.84001e+02, 1.45795e+02, 1.56708e+04], R1);

    % This is a link between Waist and Right/Left Leg. Here it is
    % referenced w.r.t. the frame 5 of the standing legs with the origin in
    % the center of the two legs (w)   
    waist.link(2).mass = 4.17e-01;
%     R2 = [0, 0, -1; 1, 0, 0; 0, -1, 0];
%     waist.link(2).CoM = R2 * [-3.22707e+01; 1.11375e+01; -3.29172e-01] + [0; waist.link_lengths(1); -waist.link_lengths(2)];
%     waist.link(2).inertia = rotate_inertia_matrix([6.60777e+02, 6.74647, 4.93444; 6.74647, 6.42117e+02,-1.51777; 4.93444,-1.51777, 1.70178e+02], R2);
    R2 = [1, 0, 0; 0, -1, 0; 0, 0, -1];
    waist.link(2).CoM = R2 * [-3.22707e+01; 1.11375e+01; -3.29172e-01];
    waist.link(2).inertia = rotate_inertia_matrix([6.60777e+02, 6.74647, 4.93444; 6.74647, 6.42117e+02,-1.51777; 4.93444,-1.51777, 1.70178e+02], R2);

    % Head
    head.link(1).mass = 7.10e-02;
    head.link(1).CoM = [5.39731; -1.58067; 5.64034e+01];
    head.link(1).inertia = [6.90569e+01, 1.08546,-9.83691; 1.08546, 6.41244e+01, 5.08942; -9.83691, 5.08942, 3.33541e+01];
    
    head.link(2).mass = 4.12e-01;    
    head.link(2).CoM = [-1.03952; -5.17985; 1.82267e-02];
    head.link(2).inertia = [5.52356e+02, 7.77318, 3.17868; 7.77318, 7.32140e+02, 4.57784e-01; 3.17868, 4.57784e-01, 6.82132e+02];

    
    %%%%%%%%%%%%%%%%
    % Total Masses %
    %%%%%%%%%%%%%%%%
    legs.right.mass = sum([legs.right.link(:).mass]);
    legs.left.mass  = sum([legs.left.link.mass]);
    legs.mass       = legs.right.mass + legs.left.mass;
    arms.right.mass = sum([arms.right.link.mass]);
    arms.left.mass  = sum([arms.left.link.mass]);
    arms.mass       = arms.right.mass + arms.left.mass;
    waist.mass      = sum([waist.link.mass]);
    head.mass       = sum([head.link.mass]);
    upper_body_mass = arms.mass + waist.mass + head.mass;
    total_mass      = legs.mass + upper_body_mass;
    
    %%%%%%%%%%%%%%%%%%%%
    % Standard gravity %
    %%%%%%%%%%%%%%%%%%%%
    g0 = [0; 0; -9.80665]*M2MM;
    if strcmpi(angle_unit, 'degrees')
        disp('Angles measured in degrees')
        angle_transform = 180/pi;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Adapt Robot generalized variables angle unit (Now in Degrees) %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        legs.right.joint = multiply_vector_struct(legs.right.joint, {'q', 'dq', 'ddq', 'angle_limits', 'relation'}, angle_transform);
        legs.left.joint  = multiply_vector_struct(legs.left.joint,  {'q', 'dq', 'ddq', 'angle_limits', 'relation'}, angle_transform);
        arms.right.joint = multiply_vector_struct(arms.right.joint, {'q', 'dq', 'ddq', 'angle_limits', 'relation'}, angle_transform);
        arms.left.joint  = multiply_vector_struct(arms.left.joint,  {'q', 'dq', 'ddq', 'angle_limits', 'relation'}, angle_transform);
        waist.joint      = multiply_vector_struct(waist.joint,      {'q', 'dq', 'ddq', 'angle_limits', 'relation'}, angle_transform);        
    else
        disp('Angles measured in radians')
    end
    
    if strcmpi(length_unit, 'mm')
        disp('Length measured in millimeters')
    else
        disp('Length measured in meters')
        length_transform = 1e-3;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Adapt Robot Dimensions Length unit (Now in m) %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        origin  = origin * length_transform;
        legs.link_lengths  = legs.link_lengths * length_transform;
        arms.link_lengths  = arms.link_lengths * length_transform;
        waist.link_lengths = waist.link_lengths * length_transform;
        torso.link_lengths = torso.link_lengths * length_transform; 
        chest.link_lengths = chest.link_lengths * length_transform;
        head.link_lengths  = head.link_lengths * length_transform;

        legs.right.joint = multiply_vector_struct(legs.right.joint, {'origin'}, length_transform);
        legs.left.joint  = multiply_vector_struct(legs.left.joint,  {'origin'}, length_transform);
        arms.right.joint = multiply_vector_struct(arms.right.joint, {'origin'}, length_transform);
        arms.left.joint  = multiply_vector_struct(arms.left.joint,  {'origin'}, length_transform);
        legs.right.foot  = multiply_vector_struct(legs.right.foot,  {'origin'}, length_transform);
        legs.left.foot   = multiply_vector_struct(legs.left.foot,   {'origin'}, length_transform);
        legs.right.foot.limits = multiply_vector_struct(legs.right.foot.limits,  {'x', 'y'}, length_transform);
        legs.left.foot.limits  = multiply_vector_struct(legs.left.foot.limits,   {'x', 'y'}, length_transform);
        waist.joint      = multiply_vector_struct(waist.joint,      {'origin'}, length_transform);
        waist.IMU  = multiply_vector_struct(waist.IMU,  {'origin'}, length_transform);
        head.joint = multiply_vector_struct(head.joint, {'origin'}, length_transform);

        legs.right.joint = multiply_vector_struct(legs.right.joint, {'torque_limits'}, length_transform^2);
        legs.left.joint  = multiply_vector_struct(legs.left.joint,  {'torque_limits'}, length_transform^2);
        arms.right.joint = multiply_vector_struct(arms.right.joint, {'torque_limits'}, length_transform^2);
        arms.left.joint  = multiply_vector_struct(arms.left.joint,  {'torque_limits'}, length_transform^2);
        waist.joint      = multiply_vector_struct(waist.joint,      {'torque_limits'}, length_transform^2);
               
        legs.right.link = multiply_vector_struct(legs.right.link, {'CoM'}, length_transform);
        legs.left.link  = multiply_vector_struct(legs.left.link,  {'CoM'}, length_transform);
        arms.right.link = multiply_vector_struct(arms.right.link, {'CoM'}, length_transform);
        arms.left.link  = multiply_vector_struct(arms.left.link,  {'CoM'}, length_transform);
        waist.link      = multiply_vector_struct(waist.link,      {'CoM'}, length_transform);
        head.link       = multiply_vector_struct(head.link,      {'CoM'}, length_transform);
        
        legs.right.link = multiply_vector_struct(legs.right.link, {'inertia'}, length_transform^2);
        legs.left.link  = multiply_vector_struct(legs.left.link,  {'inertia'}, length_transform^2);
        arms.right.link = multiply_vector_struct(arms.right.link, {'inertia'}, length_transform^2);
        arms.left.link  = multiply_vector_struct(arms.left.link,  {'inertia'}, length_transform^2);
        waist.link      = multiply_vector_struct(waist.link,      {'inertia'}, length_transform^2);        
        head.link       = multiply_vector_struct(head.link,      {'inertia'}, length_transform^2);        

        g0 = g0 * length_transform;

        %%%%%%%%%%%%%%%%%%%%%
        % Round to mm and g %
        %%%%%%%%%%%%%%%%%%%%%        
%         legs.right.link = round_vector_struct(legs.right.link, {'CoM', 'mass', 'inertia'});
%         legs.left.link  = round_vector_struct(legs.left.link,  {'CoM', 'mass', 'inertia'});
%         arms.right.link = round_vector_struct(arms.right.link, {'CoM', 'mass', 'inertia'});
%         arms.left.link  = round_vector_struct(arms.left.link,  {'CoM', 'mass', 'inertia'});
%         waist.link      = round_vector_struct(waist.link,      {'CoM', 'mass', 'inertia'});
% 
%         g0 = round_to_mm(g0);
        
        legs.right.g0 = sym(g0);
        legs.left.g0  = sym(g0);
        waist.g0      = sym(g0);
        arms.right.g0 = sym(g0);
        arms.left.g0  = sym(g0);
        head.g0      = sym(g0);
        robot.g0      = g0;

    end
end

% Setting fields
robot.origin = origin;
robot.legs = legs;
robot.arms = arms;
robot.waist = waist;
robot.torso = torso;
robot.chest = chest;
robot.head = head;
end


function S_new = multiply_vector_struct(S, field, c)
% This function multiplies by a number c every element of the cell field
% indicating a field of vector of structs S
S_new = S;
n = length(S);
m = length(field);
for jj = 1:n
    for kk = 1:m
        S_new(jj).(field{kk}) = S(jj).(field{kk})*c;
    end
end
end

function S_new = round_vector_struct(S, field)
% This function multiplies by a number c every element of the cell field
% indicating a field of vector of structs S
S_new = S;
n = length(S);
m = length(field);
for jj = 1:n
    for kk = 1:m
        S_new(jj).(field{kk}) = round_to_mm (S(jj).(field{kk}));
    end
end
end

function Y = round_to_mm (X)
EPS = 1e-4;
Y = round(X / EPS) * EPS;
end