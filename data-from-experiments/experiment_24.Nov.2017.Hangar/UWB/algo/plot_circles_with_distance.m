%%  nodes oder x2, x3, x1, x5, x6
    %     locations fixed: x1(0,0)  x3(+6,0) for v2
    %     locations fixed: x2(0,0)  x3(-6,0) for v1
    % unit mm
    
% clear;
experimentNumber = 3; % [1.1 1 2 3 4]
nodesNumber = 5;
    switch experimentNumber
        case 1
            distances2all_abs= importdata('..\data\data_t_dist_1circle_t.mat');
            MoCap_data = importdata('..\..\mocap\afterRTtoUWB\cortex_json2_circle_RT2UWB.mat');
            near_idex = nearestpoint(data(:,1)-1, MoCap_data(:,9));
            RRT_oo = [113.020896838943        -0.259574835362515        0.0678477520020548];
        case 2
            distances2all_abs= importdata('..\data\data_t_dist_2_sq_t.mat');
            MoCap_data = importdata('..\..\mocap\afterRTtoUWB\cortex_json6_sq_RT2UWB.mat');
            near_idex = nearestpoint(data(:,1)-1.2, MoCap_data(:,9));
            RRT_oo = [-0.0775624574281473        -0.254508229572838        0.0491643222229634];
        case 3
            load('..\output_algo\ekf\3_workspace_0.001_1.mat');
            distances2all_abs = measurements_data_noisy; % 5*86
            section = [222:1:288];
            distances2all_abs= distances2all_abs(:,section); % (48:133,:); %(134:218,:)  
            real_X = real_X(:,section);
            X = X(:,section);
            RESIDUAL = RESIDUAL(:,section);
            positionOfNodes = positionOfNodes;
            real_X = real_X; 
            
        otherwise
            warning('please specify the experiment number #')
    end
%distances2all_abs = distances2all_abs';
%% make cross points of circles with Radius distances2all
% make circles
string =sprintf('experiment %d', experimentNumber);
figure; title(string);hold on; % axis square; axis tight;
subplot(1,2,1);hold on;
plot(positionOfNodes(1,:), positionOfNodes(2,:), 'd');
plot(real_X(1,:), real_X(2,:), 'r-+');
plot(X(1,:), X(2,:), 'b-+');
subplot(1,2,2);hold on;
abs_RESIDUAL = abs(RESIDUAL);
plot(abs_RESIDUAL','c+');

for j = 1:size(distances2all_abs, 2)
    % h_1 = plot(traj(1,j), traj(2,j), 'ro'); %hold on;
    subplot(1,2,2);hold on;
    abs_RESIDUAL_J = abs_RESIDUAL(:,j);
    abs_RESIDUAL_J(isnan(abs_RESIDUAL_J)) = [];
    if size(abs_RESIDUAL_J,1) == 0
        continue
    end
    plot(j,abs_RESIDUAL_J,'go');
    plot(j,mean(abs_RESIDUAL_J),'b^');
    plot(j,std(abs_RESIDUAL_J),'rp');
    if nodesNumber == 5
        for i = 1 : size(positionOfNodes, 2)
            subplot(1,2,1);hold on;
            h(i) = plotCircle(positionOfNodes(1, i), positionOfNodes(2, i), ...
                distances2all_abs(i, j), 0*pi, 2*pi); % 0, 2*pi); %
        end
    end
    axis square; axis tight;
    xlim([-4 6]); ylim([-1 11]);  
    daspect([10,10,10]);
    string =sprintf('j %d, experiment %d', j, experimentNumber);
    title(string);
        subplot(1,2,1);hold on;
    h1 = plot(real_X(1,j), real_X(2,j), 'ro');
    h2 = plot(X(1,j), X(2,j), 'bo');
    pause(0.15);

    delete(h); 
    delete(h1);
    delete(h2);
end
% make cross points 'x'
 
function h_circle = plotCircle(x,y,r,ang_start,ang_end)
%x and y are the coordinates of the center of the circle
%r is the radius of the circle
%0.01 is the angle step, bigger values will draw the circle faster but
%you might notice imperfections (not very smooth)
ang=ang_start:0.01:ang_end; 
%ang=0:0.01:2*pi; 
xp=r*cos(ang);
yp=r*sin(ang);
h_circle = plot(x+xp,y+yp);
end

    %% function
    function [IND, D] = nearestpoint(x,y,m)
        % NEARESTPOINT - find the nearest value in another vector
        %
        %   IND = NEARESTPOINT(X,Y) finds the value in Y which is the closest to
        %   each value in X, so that abs(Xi-Yk) => abs(Xi-Yj) when k is not equal to j.
        %   IND contains the indices of each of these points.
        %   Example:
        %      NEARESTPOINT([1 4 12],[0 3]) % -> [1 2 2]
        %
        %   [IND,D] = ... also returns the absolute distances in D,
        %   that is D == abs(X - Y(IND))
        %
        %   NEARESTPOINT(X, Y, M) specifies the operation mode M:
        %   'nearest' : default, same as above
        %   'previous': find the points in Y that are closest, but preceeds a point in X
        %               NEARESTPOINT([0 4 3 12],[0 3],'previous') % -> [NaN 2 1 2]
        %   'next'    : find the points in Y that are closets, but follow a point in X
        %               NEARESTPOINT([1 4 3 12],[0 3],'next') % -> [2 NaN 2 NaN]
        %
        %   If there is no previous or next point in Y for a point X(i), IND(i)
        %   will be NaN (and D(i) as well).
        %
        %   X and Y may be unsorted.
        %
        %   This function is quite fast, and especially suited for large arrays with
        %   time data. For instance, X and Y may be the times of two separate events,
        %   like simple and complex spike data of a neurophysiological study.
        %
        %   Nearestpoint('test') will run a test to show it's effective ness for
        %   large data sets
        
        % version 4.1 (jan 2016)
        % (c) 2004 Jos van der Geest
        % Matlab File Exchange Author ID: 10584
        % email: samelinoa@gmail.com
        
        % History :
        %  aug 25, 2004 - corrected to work with unsorted input values
        %  nov 02, 2005 -
        %  apr 28, 2006 - fixed problem with previous points
        %  sep 14, 2012 - updated for more recent versions of ML
        %                 fixed two errors per suggestion of Drew Compston
        %  v4.1 (jan 2016) - fixed error when second output was requested without a
        %          next or previous nearestpoint (thanks to Julian)
        %          - fixed mlint suggestions
        
        if nargin==1 && strcmp(x,'test'),
            
            testnearestpoint ;
            return
        end
        
        narginchk(2,3) ;
        
        if nargin==2,
            m = 'nearest' ;
        else
            if ~ischar(m),
                error('Mode argument should be a string (either ''nearest'', ''previous'', or ''next'')') ;
            end
        end
        
        if ~isa(x,'double') || ~isa(y,'double'),
            error('X and Y should be double matrices') ;
        end
        
        if isempty(x) || isempty(y)
            IND = [] ;
            D = [] ;
            return ;
        end
        
        % sort the input vectors
        sz = size(x) ;
        [x, xi] = sort(x(:)) ;
        [~, xi] = sort(xi) ; % for rearranging the output back to X
        nx = numel(x) ;
        cx = zeros(nx,1) ;
        qx = isnan(x) ; % for replacing NaNs with NaNs later on
        
        [y,yi] = sort(y(:)) ;
        ny = length(y) ;
        cy = ones(ny,1) ;
        
        xy = [x ; y] ;
        
        [~, xyi] = sort(xy) ;
        cxy = [cx ; cy] ;
        cxy = cxy(xyi) ; % cxy(i) = 0 -> xy(i) belongs to X, = 1 -> xy(i) belongs to Y
        ii = cumsum(cxy) ;
        ii = ii(cxy==0).' ; % ii should be a row vector
        
        % reduce overhead
        clear cxy xy xyi ;
        
        switch lower(m),
            case {'nearest','near','absolute'}
                % the indices of the nearest point
                ii = [ii ; ii+1] ;
                ii(ii==0) = 1 ;
                ii(ii>ny) = ny ;
                yy = y(ii) ;
                dy = abs(repmat(x.',2,1) - yy) ;
                [~, ai] = min(dy) ;
                IND = ii(sub2ind(size(ii),ai,1:nx)) ;
            case {'previous','prev','before'}
                % the indices of the previous points
                ii(ii < 1) = NaN ;
                IND = ii ;
            case {'next','after'}
                % the indices of the next points
                ii = ii + 1 ;
                ii(ii>ny) = NaN ;
                IND = ii ;
            otherwise
                error('Unknown method "%s"',m) ;
        end
        
        IND(qx) = NaN ; % put NaNs back in
        % IND = IND(:) ; % solves a problem for x = 1-by-n and y = 1-by-1
        
        if nargout==2,
            % also return distance if requested;
            D = NaN(1,nx) ;
            q = ~isnan(IND) ;
            if any(q)
                D(q) = abs(x(q) - reshape(y(IND(q)),[],1)) ;
            end
            D = reshape(D(xi),sz) ;
            
        end
        
        % reshape and sort to match input X
        IND = reshape(IND(xi),sz) ;
        
        % because Y was sorted, we have to unsort the indices
        q = ~isnan(IND) ;
        IND(q) = yi(IND(q)) ;
    end

% END OF FUNCTION

    function testnearestpoint
        disp('TEST for nearestpoint, please wait ... ') ;
        M = 13 ;
        tim = NaN(M,3) ;
        tim(8:M,1) = 2.^(8:M).' ;
        figure('Name','NearestPointTest','doublebuffer','on') ;
        h = plot(tim(:,1),tim(:,2),'bo-',tim(:,1),tim(:,3),'rs-') ;
        xlabel('N') ;
        ylabel('Time (seconds)') ;
        title('Test for Nearestpoint function ... please wait ...') ;
        set(gca,'xlim',[0 max(tim(:,1))+10]) ;
        for j=8:M,
            N = 2.^j ;
            A = rand(N,1) ; B = rand(N,1) ;
            tic ;
            D1 = zeros(N,1) ;
            I1 = zeros(N,1) ;
            for i=1:N,
                [D1(i), I1(i)] = min(abs(A(i)-B)) ;
            end
            tim(j,2) = toc ;
            pause(0.1) ;
            tic ;
            [D1x, D2x] = nearestpoint(A,B) ; %#ok<NASGU,ASGLU>
            tim(j,3) = toc ;
            % isequal(I1,I2)
            set(h(1),'Ydata',tim(:,2)) ;
            set(h(2),'Ydata',tim(:,3)) ;
            drawnow ;
        end
        disp('Done.')
        title('Test for Nearestpoint function') ;
        legend('Traditional for-loop','Nearestpoint',2) ;
    end
%%
