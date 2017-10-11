%{
% unit: mm
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
function [X, P, z_all] = KF_using_HTerm_data(factor_Q, factor_R, data) %, measurements_missing, MaxNumMeasMissedWithinSet, traj_name
    format longG
%{    
%     %% import measuremnets data from recording
%     %measurements_data_noisy = importdata('..\..\trajectory\goodTraj01\noisy_measuremnts_data2.mat'); % load noisy_measurements
%     distances2all_abs = importdata('..\..\trajectory\goodTraj01\distances2all_abs_without_noise.mat'); % load measurements without noise
%     rng(1,'twister');s = rng;rng(s); % Save and Restore the Generator Settings, this make the measurements_data_noisy will always be the same
%     measurements_data_noisy = distances2all_abs + factor_R * randn(size(distances2all_abs));
%     nodes_Nums = 4;
%     positionOfNodes = [-5 -5; 5 -5; 5 5; -5 5]'; % <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
%}
    %% import trajectory positions & generate noisy measuremnets data
    % file_name = ['..\..\trajectory\goodTraj' traj_name '\position' traj_name '.mat']; 
%     file_name = ['position' traj_name '.mat']; 
%     real_X = importdata(file_name); 
    %data = importdata('data_t_dist_HT_temp_filled.mat');
    timeStamp = data(:,1);
    measurements_data_noisy = data(:,2:end)';
    %measurements_data_noisy = measurements_data_noisy/100;
    %{
    % substract the height, we get the horizontal distances
    measurements_data_noisy = sqrt(measurements_data_noisy.^2 - 1.20274960392966^2); 
    %}
    nodes_Nums = 5;    
    positionOfNodes = [2938.41844377029,-3013.26989169788; [184.822603869210,-143.127276884650];...
        [4161.77182689655,2235.61214448276]; [-1396.30540772784,-2433.57009459426]; [-1741.84732630000,2032.12649985000]]'/1000;
    %{
    distances2all_abs = zeros(size(positionOfNodes, 2), size(real_X, 2));
    for i = 1 : size(positionOfNodes, 2)
        distances2each_xy = [real_X(1:2, :) - repmat(positionOfNodes(:,i), 1, size(real_X, 2))];
        distances2each_abs = sqrt(distances2each_xy(1,:).^2 + distances2each_xy(2,:).^2);
        distances2all_abs(i, :) = distances2each_abs;
    end
    rng(1,'twister');s = rng;rng(s); % Save and Restore the Generator Settings, this make the measurements_data_noisy will always be the same
    measurements_data_noisy = distances2all_abs + factor_R * randn(size(distances2all_abs));    
    
   %% for small test, use a series measurements data generate when Tag stays at origin all the time
    % measurements_data_noisy = repmat([sqrt(50^2 + 50^2); sqrt(100^2 + 50^2); sqrt(100^2 + 100^2); sqrt(50^2 + 100^2)], 1, size(measurements_data_noisy, 2));  
%}
    %% initiation
    x_0 = [[4161.77182689655,2235.61214448276]/1000 1.0 1.0]';
    P_0 = eye(4, 4); % TODO, choose to be all one, a litle too big, but it should converge at the end if the KF work 

    % control-input model matrix(control matrix) B, control-input(control vector) is zero
    B = ones(4, 4);
    u = zeros(4, 1);
    

 %{   
    switch traj_name %sigma should be the same as the title of the traj_plot
        case '1' 
            sigma = 0.165;
        case '2'
            sigma = 0.17;
        case '3'
            sigma = 0.18;
        case '4'
            sigma = 0.185;
        case '5'
            sigma = 0.19;
        case '6'
            sigma = 0.195;
        case '7'
            sigma = 0.2; %TODO: not sure, need to be checked
        case '8'
            sigma = 0.205;
        case '9'
            sigma = 0.235;
        case '10'
            sigma = 0.215;
        case '11'
            sigma = 0.26;
        case '12'
            sigma = 0.27;%TODO: not sure, need to be checked
        case '13'
            sigma = 0.285;
        case '14'
            sigma = 0.3;
        case '15'
            sigma = 0.31;
        case '16'
            sigma = 0.335;
        case '17'
            sigma = 0.36;
        case '18'
            sigma = 0.375;
        case '19'
            sigma = 0.39;
        case '20'
            sigma = 0.415;
        case '21'
            sigma = 0.43;
        case '22'
            sigma = 0.475;
        case '23'
            sigma = 0.485;
        case '24'
            sigma = 0.625;
        case '25'
            sigma = 0.635;
        case '26'
            sigma = 0.675;
        case '27'
            sigma = 0.715;
        case '28'
            sigma = 0.855;%TODO: not sure, need to be checked
        case '29'
            sigma = 0.975;
        case '30'
            sigma = 0.985; %TODO: not sure, need to be checked                  
        otherwise
            error('>>>by programmer<<< !!! sigma unspecified, please check the simga value in traj_plot; <<<<<!!!!!!!<<<<< ')
    end
 %}   
    % measurement noise covariance R
    %R_all = factor_R * eye(nodes_Nums);
    R_all = factor_R * diag(([3.900095115, 3.83763106, 4.0818845734, 2.7939164184, 2.9198114402]/100).^2); % base on calibration analysis
    % TODO, correct Q & R, they are square matrices

    % H matrix
    syms N_xy x_m Z_e
    numNodes = nodes_Nums; % <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    N_x_n = sym('N_x_n%d%d', [2 numNodes]); % posNodes %[n_x1 n_x2 n_x3; n_y1 n_y2 n_y3]. i.e. 'N_x_n21' means the y_posi of the ist node 
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
    
    %% Kalman Filter
     % notation symbols please check Page 30 in 'An Intro to the Kalman Filter, G. Welch G. Bishop' 
    X = zeros(4, size(measurements_data_noisy,2)); % state matrix
    P = zeros(4, 4, size(measurements_data_noisy,2)); % state covariance matrix
    % K = zeros(4, nodes_Nums, size(measurements_data_noisy,2)); % Kalman Gain matrix
    X(:, 1) = x_0;
    P(:, :, 1) = P_0;

    z_all = measurements_data_noisy;
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
    % EKF loop
    for i = 2:size(measurements_data_noisy,2)
        % time update
        % sampling time interval
        dt = timeStamp(i) - timeStamp(i-1);
        % state transition model matrix A
        A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
        % process noise covariance Q
        G = [dt^2/2*eye(2);dt*eye(2)];
        sigma = 0.5;
        %since displacement cause by acceleration is G * sigma * randn(2,1) {acceleration is  sigma * randn(2,1)}
        Q = factor_Q * G * sigma^2 * G'; % I think the factor_Q should be deleted here.TODO: factor_Q^2 ???  factor_Q ???
        
        x_minus= A * X(:, i - 1); %  + B * u; % TODO, think about if the accelerations should be included into state vectors
        P_minus = vpa(A * P(:, :, i-1) * A' + Q);
        
        % measurement update
        R = R_all;
        Z_e = Z_e_all;
        H_symbolic = H_all_symbolic;
        z = z_all(:, i);
        k = length(z);
        while k >= 1
            if isnan(z(k)) % if no measurements are coming
                z(k) =[];
                Z_e(k) = [];
                H_symbolic(k,:) = [];
                R(k,:) = []; R(:,k) = [];
            end
            k = k-1;
        end
        if length(z) == 0 % if no measurements are coming, skip the measurement update step
            X(:, i) = x_minus;
            P(:, :, i) = P_minus;
        else
            H = vpa(eval(subs( subs(H_symbolic, x_m, x_minus), N_x_n, positionOfNodes))); %<<<change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                %{
                % Never use the inverse of a matrix to solve a linear system Ax=b with x=inv(A)*b, because it is slow and inaccurate.
                % Replace inv(A)*b with A\b, Replace b*inv(A) with b/A, replace A*inv(B)*C with A*(B\C).
                % HERE replace K = P_minus * H' * invs(H * P_minus * H' +R) with P_minus * H' / (H * P_minus * H' +R) 
                %}
            K_k = P_minus * H' / (H * P_minus * H' + R); %<<< R >change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            %<<< add Z_e >change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            X(:, i) = x_minus + K_k * (z - eval(subs( subs(Z_e, x_m, x_minus), N_x_n, positionOfNodes))); % <<<<<<wikipedia<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
            P(:, :, i) = vpa((eye(length(x_0)) - K_k * H) * P_minus);
            %K(:, :, i) = K_k; 
        end
        

    end

%% plot position on map 
% for loop here can be removed, only used to collapse the code
    for i = 1:1
        h = figure;
        plot(X(1,:), X(2,:), '-ob');
        % load real positions
%         real_X = importdata('..\..\trajectory\goodTraj01\position01.mat'); 
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
    end


end