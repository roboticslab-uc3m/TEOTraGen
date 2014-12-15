function TEOStepGenConfig = getTEOStepGenConfig( input_args )
%GETTEOSTEPGENCONFIG Summary of this function goes here
%   Default parameters:
%   - q0: Teo Initial Configuration
%   - Ts: Sample Time
%   - alpha_ds: Percentage of Time for Double Support
%   - gamma_com: Step length due to CoM
%   - L_val: Step Length (m)
%   - H_val: Step Height (m)
%   - TStep: Step Time (s)



% #########################
% TEO INITIAL CONFIGURATION
% #########################

q0 =   [ ...
        % Right Leg
        0;                    ... % Right Hip Yaw
        0;                    ... % Right Hip Roll
        -0.2701;              ... % Right Hip Pitch
        0.5680;               ... % Right Knee Pitch
        -0.2979;              ... % Right Ankle Pitch
        0;                    ... % Right Ankle Roll
        % Left Leg 
        0;                    ... % Left Hip Yaw
        0;                    ... % Left Hip Roll 
        -0.2701;              ... % Left Hip Pitch
        0.5680;               ... % Left Knee Pitch
        -0.2979;              ... % Left Ankle Pitch
        0;                    ... % Left Ankle Roll
        % Torso
        0;                    ... % Torso Yaw
        0;                    ... % Torso Pitch
        % Right Arm
        0.420000000000000;    ... % Right Shoulder Pitch
        -0.167017153300893;   ... % Right Shoulder Roll
        0;                    ... % Right Shoulder Yaw
        -1.250000000000000;   ... % Right Elbow Pitch
        0;                    ... % Right Wrist Yaw
        0;                    ... % Right Wrist Pitch
        % Left Arm
        0.420000000000000;    ... % Left Shoulder Pitch
        0.167017153300893;    ... % Left Shoulder Roll
        0;                    ... % Left Shoulder Yaw
        -1.250000000000000;   ... % Left Elbow Pitch
        0;                    ... % Left Wrist Yaw
        0                     ... % Left Wrist Pitch
        ];


% ################
% SAMPLE TIME (s)
% ################

Ts = 0.01;


% ######################################
% PERCENTAGE OF TIME FOR DOUBLE SUPPORT
% ######################################
% (Between 0 and 1)
alpha_ds = 0.5;



% ##################################
% PERCENTAGE STEP LENGTH DUE TO CoM
% ##################################
% (Between 0 and 1)
gamma_com = 0.4;



% ################
% STEP LENGTH (m)
% ################
L_val = 0.1;



% ################
% STEP HEIGHT (m)
% ################
H_val = 0.01;

% ################
% STEP TIME (s)
% ################
TStep = 5;

% Output

TEOStepGenConfig.q0 = q0;
TEOStepGenConfig.alpha_ds = alpha_ds;
TEOStepGenConfig.gamma_com = gamma_com;
TEOStepGenConfig.L_val = L_val;
TEOStepGenConfig.H_val = H_val;
TEOStepGenConfig.TStep = TStep;
TEOStepGenConfig.Ts = Ts;


end

