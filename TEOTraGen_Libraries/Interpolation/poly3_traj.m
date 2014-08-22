%POLY3_TRAJ Generate scalar Third-order polynomial trajectory
%
% [S,SD,SDD] = POLY3_TRAJ(S0, SF, M) is a scalar trajectory (Mx1) that varies 
% smoothly from S0 to SF in M steps using a 3rd order polynomial.
% Velocity and acceleration can be optionally returned as SD (Mx1) and SDD (Mx1).
%
% [S,SD,SDD] = POLY3_TRAJ(S0, SF, T) as above but specifies the trajectory in 
% terms of the length of the time vector T (Mx1).
%
% Notes::
% - If no output arguments are specified S, SD, and SDD are plotted.

% [S,SD,SDD] = POLY3_TRAJ(S0, SF, N, SD0, SDF) as above but specifies initial 
% and final joint velocity for the trajectory.
%
% [S,SD,SDD] = POLY3_TRAJ(S0, SF, T, SD0, SDF) as above but specifies initial 
% and final joint velocity for the trajectory and time vector T.
%
% Notes::
% - In all cases if no output arguments are specified S, SD, and SDD are plotted 
%   against time.
%
% See also LSPB, JTRAJ, POLY5_TRAJ, POLY7_TRAJ, POLY3_TRAJECTORY.
%
%   Author: Domingo Esteban
%   $Revision: 1.0 $  $Date: 2013/07/23 $


function [s,sd,sdd] = poly3_traj(q0, qf, t, qd0, qdf)

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
    
    tf = max(t);
    
    % solve for the polynomial coefficients using least squares
    X = [
        0           0           0       1
        tf^3        tf^2        tf      1
        0           0           1       0
        3*tf^2      2*tf        1       0
    ];
    coeffs = (X \ [q0 qf qd0 qdf]')';

    % coefficients of derivatives 
    coeffs_d = coeffs(1:3) .* (3:-1:1);
    coeffs_dd = coeffs_d(1:2) .* (2:-1:1);

    % evaluate the polynomials
    p = polyval(coeffs, t);
    pd = polyval(coeffs_d, t);
    pdd = polyval(coeffs_dd, t);

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
            subplot(311)
            plot(xt, p); grid; ylabel('s');

            subplot(312)
            plot(xt, pd); grid; ylabel('sd');
            
            subplot(313)
            plot(xt, pdd); grid; ylabel('sdd');

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

    end

