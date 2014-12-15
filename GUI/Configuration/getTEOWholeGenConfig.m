function TEOWholeGenConfig = getTEOWholeGenConfig(  )
%GETTEOSTEPGENCONFIG Gets TEOWholeGen default parameters
%   Default parameters:
%   - q0: Teo Initial Configuration
%   - Ts: Sample Time
%   - kp: IK Position Gain
%   - kp: IK Orientation Gain


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


% ############
% SAMPLE TIME
% ############

Ts = 0.09;



% #################
% IK POSITION GAIN
% #################

kp = 0.01;


% ####################
% IK ORIENTATION GAIN
% ####################

ko = pi/8;







% Output

TEOWholeGenConfig.q0 = q0;
TEOWholeGenConfig.Ts = Ts;
TEOWholeGenConfig.kp = kp;
TEOWholeGenConfig.ko = ko;

end