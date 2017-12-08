%{
% unit: m
% output
%           X: matrix of state vector [4 * #semple]
%           P:  covariance matrix of state vector [4 * 4 * #semple]
%           z_all: the simulate measurements will NaN data, representing
the missed data
% input  
%           factor_Q: float
%           factor_R: float
%           measurements_missing: boolean
%           MaxNumMeasMissedWithinSet: int , rangs from 0 to numNodes
or minus integer, which means delete random numbers of elements in each set
or 100, for missing with a pattern, 1st, 2nd, 3rd, 4th, 1st, 2nd, 3rd...
%           traj_num: string : '01'
%}
%% nodes position 
    % [    6
    %  5     2
    %    1  3  ]
%%
function [X, P, z_all] = KF_using_HTerm_data(factor_Q, factor_R, experimentNumber) %, measurements_missing, MaxNumMeasMissedWithinSet, traj_name
    format longG
    %% flag of remove_outlier_measurements
    remove_outlier_meas = 0; 
    %% flag of using 25ms40Hz method
    ms25Hz40 = 0;     
    %% import and process original-data
    switch experimentNumber
        case 1
            data = importdata('..\data\data_t_dist_1circle_t.mat');
            MoCap_data = importdata('..\..\mocap\afterRTtoUWB\cortex_json2_circle_RT2UWB.mat');
        case 2
            data = importdata('..\data\data_t_dist_2_sq_t.mat');
            MoCap_data = importdata('..\..\mocap\afterRTtoUWB\cortex_json6_sq_RT2UWB.mat');
        case 3
            data = importdata('..\data\data_t_dist_3_sq_t.mat');
            MoCap_data = importdata('..\..\mocap\afterRTtoUWB\cortex_json7_sq_RT2UWB.mat');
        otherwise
            warning('please specify the experiment number #')
    end
    
    nodes_Nums = 5;
    
    near_idex = nearestpoint(data(:,1), MoCap_data(:,9));
    MoCap_data_shrinked =[];
    for ih = near_idex
        MoCap_data_shrinked = [MoCap_data_shrinked; MoCap_data(ih,:)];
    end
    real_X = MoCap_data_shrinked(:,[7,8])'; % unit mm to m 
    %% if 25ms40Hz mode are activated
    if ms25Hz40 == 1
        temp_data = nan(size(data,1)*nodes_Nums, size(data,2));
        for ji = 1:size(data,1)
            for nodeName = 1:nodes_Nums %1:5
                shift = nodeName-1; %0;4
                row_temp_data = (ji-1)*5+nodeName;
                
                temp_data(row_temp_data,nodeName+1) = data(ji,nodeName+1);
                temp_data(row_temp_data,1) = data(ji,1) - (4-shift)*1/40;
            end
        end
        temp_data(1,:) = temp_data(1,:) + 0.1;
        data = temp_data;
    else
        % do nothing
    end
    %% process data
    timeStamp = data(:,1); % unit second
    time_diff = diff(timeStamp);
    h0 = figure;
    plot(time_diff,'-+');
    str_title0 = sprintf('experiment%d  Time Difference', experimentNumber);
    title(str_title0);
    ylabel('time difference(s)'); xlabel('step');
    plot_str = ['time_diff_each_experi/time_diff_experi', num2str(experimentNumber), '.fig'];
    % ----------- savefig(h0, plot_str);
    measurements_data_noisy = data(:,2:end)';% unit mm
    figure;
    subplot(3,1,1)
    ttemp = sum(~isnan(measurements_data_noisy), 1);
    plot(ttemp')
    title('#valuable measurements before removed outlier meas')
    subplot(3,1,2)
    plot([0;time_diff])
    title('time diff between measurement-sets')
    
    if remove_outlier_meas == 1
        idx_outlier_meas = sub2ind(size(measurements_data_noisy), outlier_meas_index(:,2), outlier_meas_index(:,1));
        measurements_data_noisy(idx_outlier_meas) = NaN;
    else
    end

    
    subplot(3,1,3)
    ttemp = sum(~isnan(measurements_data_noisy), 1);
    plot(ttemp')
    title('#valuable measurements after removed outlier meas')
    
    measurements_data_noisy = measurements_data_noisy/1000; %unit from mm to m
    %{
    % substract the height, we get the horizontal distances
    measurements_data_noisy = sqrt(measurements_data_noisy.^2 - 1.20274960392966^2);
    %}
    
    positionOfNodes = importdata('..\output_algo\nodesPositionLaserOptimal\nodePo.mat');
    
    circle_center = [1.5;5.3]; %unit m % rough estimate
    circle_angle = [0:pi/50:2*pi];
    circle_x = circle_center(1)+2.5*cos(circle_angle); %unit m
    circle_y = circle_center(2)+2.5*sin(circle_angle); %unit m

    %% initiation
    x_0 = [3.45, 7.12, 0.1, 0.1]';
    P_0 = eye(4, 4); % TODO, choose to be all one, a litle too big, but it should converge at the end if the KF work 

    % control-input model matrix(control matrix) B, control-input(control vector) is zero
    B = ones(4, 4);
    u = zeros(4, 1);
    
    % measurement noise covariance R
    %R_all = factor_R * eye(nodes_Nums);
    R_all = factor_R * diag(([24.70555794, 29.76394171, 28.30651397, 27.9094253, 21.65470671]/1000).^2); % base on calibration analysis; TODO
    %                                              TODO, correct Q & R, they are square matrices

    % H matrix
    syms N_xy x_m Z_e
    numNodes = nodes_Nums; % <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    N_x_n = sym('N_x_n%d%d', [2 numNodes]); % posNodes %[n_x1 n_x2 n_x3; n_y1 n_y2 n_y3]. i.e. 'N_x_n21' means the y_posi of the 1st node 
    x_m = sym('x_m%d', [4 1]); % time_updated state vector
    Z_e_all = [     % Z_e: expected measurements (all nodes are presented)
                sqrt( (N_x_n(1,1) - x_m(1))^2 + (N_x_n(2,1) - x_m(2))^2 );
                sqrt( (N_x_n(1,2) - x_m(1))^2 + (N_x_n(2,2) - x_m(2))^2 );
                sqrt( (N_x_n(1,3) - x_m(1))^2 + (N_x_n(2,3) - x_m(2))^2 );
                sqrt( (N_x_n(1,4) - x_m(1))^2 + (N_x_n(2,4) - x_m(2))^2 );
                sqrt( (N_x_n(1,5) - x_m(1))^2 + (N_x_n(2,5) - x_m(2))^2 )
              ];
    Z_e_all = simplify(Z_e_all);
    H_all_symbolic = simplify( jacobian(Z_e_all, x_m) ); %H_symbolic = simplify(jacobian(Z_e_all, x_m));
    
    %% Kalman Filter initiation
     % notation symbols please check Page 30 in 'An Intro to the Kalman Filter, G. Welch G. Bishop' 
    X = zeros(4, size(measurements_data_noisy,2)); % state matrix
    P = zeros(4, 4, size(measurements_data_noisy,2)); % state covariance matrix
    % K = zeros(4, nodes_Nums, size(measurements_data_noisy,2)); % Kalman Gain matrix
    X(:, 1) = x_0;
    P(:, :, 1) = P_0;
    z_all = measurements_data_noisy;
    RESIDUAL = zeros(size(measurements_data_noisy,1),1);
    INDEX_IAN =[]; % for debugging
    STD_MINUS_MAD = [0];
    

%{
    if measurements_missing % here each colimn misses a certain numbers of data
        % delete(replaced with NaN) randomly some elements in each column to simulate missing data
        if MaxNumMeasMissedWithinSet < 0 %  delete random numbers of elements in each set
%{            
%             for kk = 1:size(z_all,2)  %<<<<<<<<<<<<<<<<<<<<NOT GOOD TODO: IMPROVE
%                 index_NaN_in_each_column = randi([1 size(z_all,1)] ,1, 4);
%                 %figure;histogram(index_NaN_in_each_column, 4);
%                 for ll = 1: size(index_NaN_in_each_column,2)
%                     z_all(index_NaN_in_each_column(ll),kk) = NaN;
%                 end
%             end
%}
            for kk = 1:size(z_all,2)  
                num_NaN_in_each_column = randi([0 size(z_all,1)], 1,1);
                index_NaN_in_each_column = randi([1 size(z_all,1)] ,1, num_NaN_in_each_column);
                %figure;histogram(index_NaN_in_each_column, 4);
                for ll = 1: size(index_NaN_in_each_column,2)
                    z_all(index_NaN_in_each_column(ll),kk) = NaN;
                end
            end   
        elseif MaxNumMeasMissedWithinSet == 100 % for missing with a pattern, 1st, 2nd, 3rd, 4th, 1st, 2nd, 3rd...
                index_not_NaN_in_each_column = repmat( linspace(1, nodes_Nums, nodes_Nums), [1 ceil(size(measurements_data_noisy,2)/nodes_Nums)]);
                index_not_NaN_in_each_column = index_not_NaN_in_each_column(1:size(measurements_data_noisy,2));
                temp_matrix = NaN(size(measurements_data_noisy));
                for iii = 1:size(measurements_data_noisy,2)
                    temp_matrix(index_not_NaN_in_each_column(iii),iii) = z_all(index_not_NaN_in_each_column(iii),iii);                   
                end
                z_all = temp_matrix;
        else %  delete the same max numbers of elements in each set
            for ii = 1:MaxNumMeasMissedWithinSet 
                index_NaN_in_each_column = randi([1 size(z_all,1)] ,1,size(z_all,2));
                figure;histogram(index_NaN_in_each_column,12);
                for jj = 1:size(z_all,2)
                    z_all(index_NaN_in_each_column(jj),jj) = NaN;
                end
            end
        end

    end
%}    
    
    %% EKF loop
    x_estimated = X(:, 1);
%     time_last = timeStamp(1);
    meas_trust_factor = 1;
    for i = 2:size(measurements_data_noisy,2)
        %% time update
        % sampling time interval        
        dt = timeStamp(i) - timeStamp(i-1);
%         time_now = timeStamp(i);
%         dt = time_now - time_last;
        % adjust Q R by dt (0.91s is a threshold)
        if dt > 0.91
            Q_affected_by_dt = 1 + 15*(dt - 0.91);
            R_affected_by_dt = 1 - 15*(dt -0.91);
        else
            Q_affected_by_dt = 1;
            R_affected_by_dt = 1;
        end
        % state transition model matrix A
        A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
        % process noise covariance Q
        G = [dt^2/2*eye(2);dt*eye(2)];
        sigma = 0.5;
        Q_of_process_noise = diag([1,1]);
        %since displacement cause by acceleration is G * sigma * randn(2,1) {acceleration is  sigma * randn(2,1)}
        Q = factor_Q * G * sigma^2 * G'; % I think the factor_Q should be deleted here.TODO: factor_Q^2 ???  factor_Q ???
        Q = Q *Q_affected_by_dt; % adjust Q by dt
        
        % x_minus= A * X(:, i - 1); %  + B * u; % for motion update, when no measurements are coming at all 
        x_minus= A * x_estimated; 
        P_minus = vpa(A * P(:, :, i-1) * A' + Q);
        
        %% measurement update
        R = R_all;

        Z_e = Z_e_all;
        H_symbolic = H_all_symbolic;
        
        % remove the measurement data with NaN
        z = z_all(:, i); % measurement data
        k = length(z);
        index_ian = find(~isnan(z)); % is a number
        while k >= 1
            if isnan(z(k)) % if no measurements are coming
                z(k) =[];
                Z_e(k) = [];
                H_symbolic(k,:) = [];
                R(k,:) = []; R(:,k) = [];
            end
            k = k-1;
        end
        
        % calculate the std(z)-MAD(z) to see the outlier in the measurement
        std_minus_mad = std(z) -mad(z);
        STD_MINUS_MAD = [STD_MINUS_MAD; std_minus_mad];
        
        residual_with_nan = nan(5,1);
        RESIDUAL = [RESIDUAL, residual_with_nan];
        %         if isempty(z) % if no measurements are coming, skip the measurement update step
        %             X(:, i) = x_minus;
        %             P(:, :, i) = P_minus;
        %             residual_with_nan = nan(5,1);
        % if isempty(z) || length(z) == 1 || length(z) == 2 % if only 0/1/2 node measurement are coming, skip the whole update( motion and measurement update)
        %% if 25ms40Hz mode are activated
        if ms25Hz40 == 1
            conditon_skip_meas_updata = isempty(z);
            X(:, i) = x_minus;
            P(:, :, i) = P_minus;
        else
            conditon_skip_meas_updata =  isempty(z) || length(z)==1;% if only 0/1 node measurement are coming, skip the whole update( motion and measurement update)
            % meas_trust_factor = meas_trust_factor / 1000;
        end
        
        if conditon_skip_meas_updata 
            continue
        else
            meas_trust_factor = 1;
%             time_last = time_now;
            H = vpa(eval(subs( subs(H_symbolic, x_m, x_minus), N_x_n, positionOfNodes))); %<<<change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                %{
                % Never use the inverse of a matrix to solve a linear system Ax=b with x=inv(A)*b, because it is slow and inaccurate.
                % Replace inv(A)*b with A\b, Replace b*inv(A) with b/A, replace A*inv(B)*C with A*(B\C).
                % HERE replace K = P_minus * H' * invs(H * P_minus * H' +R) with P_minus * H' / (H * P_minus * H' +R) 
            %}
            
            R = R * R_affected_by_dt; % adjust R by dt
            R = R * meas_trust_factor; % adjust R by steps of ignored update due to missing measurements
            
            K_k = P_minus * H' / (H * P_minus * H' + R); %<<< R >change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            %<<< add Z_e >cha nge!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            residual = z - eval(subs( subs(Z_e, x_m, x_minus), N_x_n, positionOfNodes)); % <<<<<<wikipedia<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
            x_estimated = x_minus + K_k * (residual); % <<<<<<wikipedia<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
            X(:, i) = x_estimated;
            
            % refill the array with NaN in the index marked position,
            % exactly refill the NaN matrix with data
           for ijj = 1:length(index_ian) % is a number
                RESIDUAL(index_ian(ijj), end) = residual(ijj);
           end
            P(:, :, i) = vpa((eye(length(x_0)) - K_k * H) * P_minus);
            %K(:, :, i) = K_k;
        end
             
        INDEX_IAN = [INDEX_IAN; index_ian];
        
    end
    
    leng_RESIDUAL = length(RESIDUAL);
    std_RESIDUAL = zeros(leng_RESIDUAL,1);
    mad_RESIDUAL = zeros(leng_RESIDUAL,1);
    for i=1:length(RESIDUAL)
        tmp = RESIDUAL(:,i);
        tmp(isnan(tmp))=[];
        std_RESIDUAL(i) = std(tmp);
        mad_RESIDUAL(i) = mad(tmp);
    end
    figure;
    subplot(2,1,1)
    plot(std_RESIDUAL, '-g');
    hold on;
    plot(mad_RESIDUAL, '-b');
    title('std(RESIDUAL) v.s. mad(RESIDUAL)');
    legend('std', 'mad');
    std_minus_mad = std_RESIDUAL - mad_RESIDUAL;
    subplot(2,1,2);
    plot(std_minus_mad, '-r');
    legend('std - mad');
    title('std(RESIDUAL) - mad(RESIDUAL)');
%% Fill missing values, replace with previous value
X(X==0)=NaN;
X = fillmissing(X,'previous',2);
% %% plot RESIDUAL
% figure;
% plot(RESIDUAL','-+');
% 
% %% plot STD_MINUS_MAD
% figure;
% plot(STD_MINUS_MAD','-+');
%% 
    h = figure;    hold on;
    switch experimentNumber
        case {0.1 0.11}
            plot(circle_center(1), circle_center(2), '*');
        case 1
%             plot(circle_center(1), circle_center(2), '*');
%             plot(circle_x, circle_y,'-');            
        case 2
%             plot(circle_center(1), circle_center(2), '*');
%             plot(circle_x, circle_y,'-');
        case 3
%             plot(circle_center(1), circle_center(2), '*');
%             plot(circle_x, circle_y,'-');
        case 4
            plot(circle_center(1), circle_center(2), '*');
            plot(circle_x, circle_y,'-');
        case 9
            plot(circle_center(1), circle_center(2), '*');
            plot(circle_x, circle_y,'-');            
        otherwise
            warning('please specify the experiment number #')
    end
    plot(positionOfNodes(1,:), positionOfNodes(2,:), 'rd');
    str_title = sprintf('experiment%d; factorQ: %d; factorR: %d', experimentNumber, factor_Q, factor_R);
    title(str_title);
    daspect([10,10,10]);
%% plot position on map 
        % h = figure;
        plot(X(1,:), X(2,:), 'o-b');
        plot(real_X(1,:), real_X(2,:),'Xr');
        % load real positions
        %{
        % real_X = importdata('..\..\trajectory\goodTraj01\position01.mat'); 
        hold on;plot(real_X(1,:), real_X(2,:), '-+r');
        % calculate the mis_match of the estimation of EKF
        % TODO: how to determine the performance of the EKF, should P also be taken into account?
        mis_pos = X(1:2,:) - real_X(1:2,:);
        area_of_map = (positionOfNodes(1,2) - positionOfNodes(1,1))^2 + (positionOfNodes(2,3) - positionOfNodes(2,1))^2;
        mis_match = sum(mis_pos(1,:).^2 + mis_pos(2,:).^2) / size(X,2) / area_of_map;
        str = sprintf('%0.20f misMatch   R %0.3f   Q %0.3f    sigma %0.3f    trajName %s    numNodes %d    MaxMeasMissedWithinSet%d', mis_match, factor_R, factor_Q, sigma, traj_name, nodes_Nums, MaxNumMeasMissedWithinSet);
        title(str);
        str = [str, '   .fig'];
        savefig(h,str);
        %}

    mat_str = ['estimated_posi_with_timeSt_EKF_experi',  num2str(experimentNumber), '.mat'];
    estimated_posi_with_timeSt = [X; timeStamp'];
    % ----------- save(mat_str, 'estimated_posi_with_timeSt');
    
%     pause_time = 0.2*[time_diff; 2];
%     for j = 5:size(X,2) %1:size(X,2)-9 
%         h2 = plot(X(1,j-4:j), X(2,j-4:j), '-+r'); %h2 = plot(X(1,j:j+9), X(2,j:j+9), '-+r'); 
%         str_title = sprintf('experiment%d; factorQ: %d; factorR: %d; j: %d', experimentNumber, factor_Q, factor_R, j);
%         title(str_title);
%         % Fram(j-4) = getframe(gcf);
%         pause(pause_time(j));
%         delete(h2);
%     end
    % ----------- video_str = ['outliar_removed_video_ekf_experiment', num2str(experimentNumber), '.avi'];
    % ----------- video = VideoWriter(video_str);
    % ----------- open(video)
    % ----------- writeVideo(video, Fram)
    % ----------- close(video)
    
    fig_str = ['traj_recovered_ekf_experiment',  num2str(experimentNumber), '.fig'];
    % ----------- savefig(h, fig_str);
    
    
    



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
