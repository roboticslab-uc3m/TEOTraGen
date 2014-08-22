%POLY5_TRAJ Generate scalar Fifth-order polynomial trajectory
%
% [S,SD,SDD] = POLY5_TRAJ(S0, SF, M) is a scalar trajectory (Mx1) that varies 
% smoothly from S0 to SF in M steps using a 5th order polynomial.
% Velocity and acceleration can be optionally returned as SD (Mx1) and SDD (Mx1).
%
% [S,SD,SDD] = POLY5_TRAJ(S0, SF, T) as above but specifies the trajectory in 
% terms of the length of the time vector T (Mx1).
%
% Notes::
% - If no output arguments are specified S, SD, and SDD are plotted.
%
% [S,SD,SDD] = POLY5_TRAJ(S0, SF, N, SD0, SDF) as above but specifies initial 
% and final joint velocity for the trajectory.
% [S,SD,SDD] = POLY5_TRAJ(S0, SF, T, SD0, SDF) as above but specifies initial 
% and final joint velocity for the trajectory and time vector T.
%
% [S,SD,SDD] = POLY5_TRAJ(S0, SF, N, SD0, SDF, SDD0, SDDF) as above but specifies initial 
% and final joint velocity and joint acceleration for the trajectory.
% [S,SD,SDD] = POLY5_TRAJ(S0, SF, T, SD0, SDF, SDD0, SDDF) as above but specifies initial 
% and final joint velocity and joint acceleration for the trajectory and time vector T.
%
% Notes::
% - In all cases if no output arguments are specified S, SD, and SDD are plotted 
%   against time.
%
% See also LSPB, JTRAJ, POLY3_TRAJ, POLY7_TRAJ, POLY5_TRAJECTORY.
%
%   Author: Domingo Esteban
%   References: Peter I. Corke
%   $Revision: 1.0 $  $Date: 2013/07/23 $


function [s,sd,sdd,sddd] = poly5_traj(q0, qf, t, qd0, qdf, qdd0, qddf)

    t0 = t;
    if isscalar(t)
		t = (0:t-1)';
    else
        t = t(:);
    end
    
    if nargin < 4
        qd0 = 0;
    end
    if nargin < 5
        qdf = 0;
    end
    if nargin < 6
        qdd0 = 0;
    end
    if nargin < 7
        qddf = 0;
    end
    
    
    tf = max(t);
    
    % solve for the polynomial coefficients using least squares
    X = [
        0           0           0       0       0   1
        tf^5        tf^4        tf^3    tf^2	tf  1
        0           0           0       0       1   0
        5*tf^4      4*tf^3      3*tf^2  2*tf    1   0
        0        	0           0       2       0   0
        20*tf^3     12*tf^2     6*tf    2       0   0
    ];
    coeffs = (X \ [q0 qf qd0 qdf qdd0 qddf]')';

    % coefficients of derivatives 
    coeffs_d = coeffs(1:5) .* (5:-1:1);
    coeffs_dd = coeffs_d(1:4) .* (4:-1:1);
    coeffs_ddd = coeffs_dd(1:3) .* (3:-1:1);

    % evaluate the polynomials
    p = polyval(coeffs, t);
    pd = polyval(coeffs_d, t);
    pdd = polyval(coeffs_dd, t);
    pddd = polyval(coeffs_ddd, t);

    switch nargout
        case 0
            if isscalar(t0)
                % for scalar time steps, axis is labeled 1 .. M
                xt = t+1;
            else
                % for vector time steps, axis is labeled by vector M
                xt = t;
            end

            clf
            subplot(411)
            plot(xt, p); grid; ylabel('s');

            subplot(412)
            plot(xt, pd); grid; ylabel('sd');
            
            subplot(413)
            plot(xt, pdd); grid; ylabel('sdd');
            
            subplot(414)
            plot(xt, pddd); grid; ylabel('sddd');  
            if ~isscalar(t0)
                xlabel('time')
            else
                for c=get(gcf, 'Children');
                    set(c, 'XLim', [1 t0]);
                end
            end
            shg
        case 1
            s = p;
        case 2
            s = p;
            sd = pd;
        case 3
            s = p;
            sd = pd;
            sdd = pdd;
        case 4
            s = p;
            sd = pd;
            sdd = pdd;
            sddd = pddd;
    end

