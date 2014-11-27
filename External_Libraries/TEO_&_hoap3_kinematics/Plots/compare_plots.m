function h = compare_plots (S, varargin)
%COMPARE_PLOTS is a function to compare plots.
%   COMPARE_PLOTS(S, 'PropertyName',propertyvalue,...) compare the plots
%   defined by the vector of structs 'S'. The optional commands are:
%       'x_shift':  shifts the plot in x axis (must be a vector of the same
%       dimension of S)
%       'x_lim':    sets x axis limits (must be a vector of two elements)
%       'y_lim':    sets y axis limits (must be a vector of two elements or
%       the string 'automatic')
%       'title':    sets the title for the plot
%       'subplot':  create axes in tiled positions (the parameter should be
%       a vector of the dimensions m and n of the small axes)
%
%   See also CREATE_PLOT_STRUCTURE, COMPARE_PLOTS, PLOT, SUBPLOT.
%
%   Author: Paolo Pierro $Revision: 1.6 $  $Date: 2010/04/26 $
% I should revise the y limits!!!
optargin = size(varargin,2);
if mod(optargin, 2) == 1
    error('COMPARE_PLOTS:argChk', 'Wrong number of input arguments')
end
n = length(S);

Ts = S(1).Ts;
for ii=2:n
    if S(ii).Ts > Ts
        Ts = S(ii).Ts;
    end
end

% Initializing flags
X_SHIFT_FLAG = 0;
X_LIM_FLAG = 0;
Y_LIM_FLAG = 0;
TITLE_FLAG = 0;
SUBPLOT_FLAG = 0;

for jj=1:2:optargin-1
    if ~ischar(varargin{jj})
        error('COMPARE_PLOTS:argChk', 'Wrong type of input arguments')
    end
    switch lower(varargin{jj})
        case {'x_shift'}
            delta_t = varargin{jj+1};
            if length(delta_t) ~= n
                error ('COMPARE_PLOTS:argChk','Wrong type of input arguments: S and delta_t array must have the same number of elements')
            end
            X_SHIFT_FLAG = 1;
            
        case {'x_lim'}
            x_lim = varargin{jj+1};
            if ~isnumeric(x_lim) || length(x_lim)~=2
                error ('COMPARE_PLOTS:argChk','Wrong type of input arguments: x_lim should be a vector of two elements')
            end
            if x_lim(1) > x_lim (2)
                error ('COMPARE_PLOTS:argChk','Wrong type of input arguments: x_lim values must be increasing')
            end
            X_LIM_FLAG = 1;
            
        case {'y_lim'}
            y_lim_par = varargin{jj+1};
            if isnumeric(y_lim_par)
                if length(y_lim_par)~=2
                    error ('COMPARE_PLOTS:argChk','Wrong type of input arguments: y_lim should be a vector of two elements')
                end
                if y_lim_par(1) > y_lim_par(2)
                    error ('COMPARE_PLOTS:argChk','Wrong type of input arguments: y_lim values must be increasing')
                end
                y_lim = y_lim_par;
                Y_LIM_FLAG = 1;
            elseif strcmpi(y_lim_par,'automatic')
                max_value = max(abs(S(1).data));
                for kk=2:n
                    max_rel = max(abs(S(kk).data));
                    if max_rel > max_value
                        max_value = max_rel;
                    end
                end
                max_value_abs = ceil(max_value*100)/100;
                y_lim = [-max_value_abs,max_value_abs];
                Y_LIM_FLAG = 1;
            else
                error ('COMPARE_PLOTS:argChk','Wrong type of input arguments: y_lim should be a vector of numbers or the string "automatic"')
            end
            
        case {'title'}
            title_fig = varargin{jj+1};
            if ~ischar (title_fig)
                error ('COMPARE_PLOTS:argChk','Wrong type of input arguments: title values must be string')
            end
            TITLE_FLAG = 1;
        
        case {'subplot'}
            dim_subplots = varargin{jj+1};
            if n~=dim_subplots(1)*dim_subplots(2)
                error('COMPARE_PLOTS:argChk','Wrong type of input arguments: the dimensions of the subplot should be coherent with the data to plot')
            else
                SUBPLOT_FLAG = 1;
            end
    end
end

h = figure;
if X_SHIFT_FLAG == 1
    delta_k = round(delta_t/Ts);
else
    delta_k = zeros (1,3);
end
if X_LIM_FLAG == 1
    xlim(x_lim);
    L = (x_lim(2)-x_lim(1))/Ts+1;
else
    L = S(1).L;
end
if Y_LIM_FLAG == 1
    ylim(y_lim);
end
if TITLE_FLAG == 1
    title(title_fig);
end

hold all
for ii=1:n
    S_new = S(ii);
    S_new.Ts = Ts;
    S_new.L = L;
    data = downsample (S(ii).data, S(ii).Ts, Ts);
    L_d = length(data);
    if delta_k(ii) < 0
        init = min(-delta_k(ii), L_d);
        data(1:init) = [];
    else
        zeri = zeros (delta_k(ii), 1);
        data = [zeri;data];
    end
    L_d = length(data);
    if L_d<L
        % data = [data;zeros(L-L_d,1)];
        data = [data,zeros(1, L-L_d)];
    else
        data(L+1:L_d)=[];
    end
    S_new.data = data;
    if SUBPLOT_FLAG == 1
        subplot(dim_subplots(1),dim_subplots(2),ii)
        plot_structure(S_new, 'complete');
        xlim(x_lim);
    else
        plot_structure(S_new, 'labels');
    end
end
if SUBPLOT_FLAG == 0
    %legend(S.legend)
    legend(S.name)
end