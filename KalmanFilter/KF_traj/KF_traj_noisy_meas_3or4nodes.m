function KF_traj_noisy_meas_3or4nodes(factor_Q, factor_R, nodes_Nums, measurements_missing, meas_missing_rate)

    %% nodes position 
    % [4 3
    %  1 2]
    %%
    %clear all;
    %close all;
    format longG
    %% import measuremnets data from recording
    measurements_data_noisy = importdata('..\..\trajectory\goodTraj01\noisy_measuremnts_data2.mat'); % load noisy_measurements
    %measurements_data_noisy = importdata('..\..\trajectory\goodTraj01\distances2all_abs_without_noise.mat'); % load measurements without noise
    positionOfNodes = [-50 -50; 100 -50; 100 100; -50 100]'; % <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    %% for small test, use a series measurements data generate when Tag stays at origin all the time
    % measurements_data_noisy = repmat([sqrt(50^2 + 50^2); sqrt(100^2 + 50^2); sqrt(100^2 + 100^2); sqrt(50^2 + 100^2)], 1, size(measurements_data_noisy, 2));  

    %% initiation
    x_0 = [0 0 1.0 1.0]';
    P_0 = eye(4, 4); % TODO, choose to be all one, a litle too big, but it should converge at the end if the KF work 
    % sampling time interval
    dt = 2/3;
    % state transition model matrix A
    A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
    % control-input model matrix B, control-input is zero
    B = ones(4, 4);
    u = zeros(4, 1);

    % H matrix
    syms N_xy x_m Z_e
    numNodes = nodes_Nums; % <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    N_x_n = sym('N_x_n%d%d', [2 numNodes]); % posNodes %[n_x1 n_x2 n_x3; n_y1 n_y2 n_y3]. i.e. 'N_x_n21' means the y_posi of the ist node 
    x_m = sym('x_m%d', [4 1]); % time_updated state vector
    if  nodes_Nums == 4
            Z_e = [     % Z_e: expected measurements [numNodes 1]<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
           sqrt( (N_x_n(1,1) - x_m(1))^2 + (N_x_n(2,1) - x_m(2))^2 );
           sqrt( (N_x_n(1,2) - x_m(1))^2 + (N_x_n(2,2) - x_m(2))^2 );
           sqrt( (N_x_n(1,3) - x_m(1))^2 + (N_x_n(2,3) - x_m(2))^2 );
           sqrt( (N_x_n(1,4) - x_m(1))^2 + (N_x_n(2,4) - x_m(2))^2 )
           ];
    end
    if  nodes_Nums == 3
            Z_e = [     % Z_e: expected measurements [numNodes 1]<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
            sqrt( (N_x_n(1,1) - x_m(1))^2 + (N_x_n(2,1) - x_m(2))^2 );
            sqrt( (N_x_n(1,2) - x_m(1))^2 + (N_x_n(2,2) - x_m(2))^2 );
            sqrt( (N_x_n(1,3) - x_m(1))^2 + (N_x_n(2,3) - x_m(2))^2 )
            ];
    end
    H_symbolic = jacobian(Z_e, x_m);

    % process noise covariance Q 
    G = [dt^2/2*eye(2);dt*eye(2)];
    sigma = 0.1;
    %RandnForGenTracj = randn(2, 1); 
    %Q = G * (sigma * diag(RandnForGenTracj))^2 * G'; %since displacement cause by acceleration is G * sigma * randn(2,1) {acceleration is  sigma * randn(2,1)}
    %Q = G * sigma^2 * G';
    Q = factor_Q * G * sigma^2 * G';
    % measurement noise covariance R
    %R = 0.1 * eye(3); % shoule be relative to the value of 'noiseLevelForMeasurements.mat' in folder 'goodTraj01'
    R = factor_R * eye(nodes_Nums);
    % TODO, correct Q & R, they are square matrices
    
    %% Kalman Filter
     % notation symbols please check Page 30 in 'An Intro to the Kalman Filter, G. Welch G. Bishop' 
    X = zeros(4, size(measurements_data_noisy,2)); % state matrix
    P = zeros(4, 4, size(measurements_data_noisy,2)); % state covariance matrix
    K = zeros(4, nodes_Nums, size(measurements_data_noisy,2)); % Kalman Gain matrix
    X(:, 1) = x_0;
    P(:, :, 1) = P_0;
    % take 4 out of 4 sets measurements, for KF here.
    z = measurements_data_noisy(1:nodes_Nums,:);
    if measurements_missing
        % delete randomly some colums to simulate missing data
        index_NaN = randi([1 size(z,2)] ,1,ceil(size(z,2)*meas_missing_rate));
        figure;histogram(index_NaN,size(z,2));
        z(:,index_NaN) = NaN;
    end

    
    % EKF loop
    for i = 2:size(measurements_data_noisy,2)
        % time update
        x_minus= A * X(:, i - 1); %  + B * u; % TODO, think about if the accelerations should be included into state vectors
        P_minus = vpa(A * P(:, :, i-1) * A' + Q); 
        
        % measurement update
        if isnan(z(1, i)) % if no measurements are coming, skip the measurement update step
            X(:, i) = x_minus;
            P(:, :, i) = P_minus;
        else
            H = vpa(eval(subs( subs(H_symbolic, x_m, x_minus), N_x_n, positionOfNodes(:,1:nodes_Nums))));
                % Never use the inverse of a matrix to solve a linear system Ax=b with x=inv(A)*b, because it is slow and inaccurate.
                % Replace inv(A)*b with A\b, Replace b*inv(A) with b/A, replace A*inv(B)*C with A*(B\C).
                % HERE replace K = P_minus * H' * invs(H * P_minus * H' +R) with P_minus * H' / (H * P_minus * H' +R) 
            K_k = P_minus * H' / (H * P_minus * H' + R);
            %X(:, i) = x_minus + K_k * (z(:, i) - H * x_minus); % <<<<<<<<<book<<<<<<<<<<<<<<<<<<<<<<<<<
            X(:, i) = x_minus + K_k * (z(:, i) - eval(subs( subs(Z_e, x_m, x_minus), N_x_n, positionOfNodes(:,1:nodes_Nums)))); % <<<<<<wikipedia<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
            P(:, :, i) = vpa((eye(length(x_0)) - K_k * H) * P_minus);
            K(:, :, i) = K_k; 
        end
        

    end

%% plot position on map
    for i = 1:1
        h = figure;
        plot(X(1,:), X(2,:), '-ob');
        % load real positions
        real_X = importdata('..\..\trajectory\goodTraj01\position01.mat'); 
        hold on;plot(real_X(1,:), real_X(2,:), '-*r');
        % calculate the mis_match of the estimation of EKF
        % TODO: how to determine the performance of the EKF, should P also be taken into account?
        mis_pos = X(1:2,:) - real_X(1:2,:);
        area_of_map = (positionOfNodes(1,2) - positionOfNodes(1,1))^2 + (positionOfNodes(2,3) - positionOfNodes(2,1))^2;
        mis_match = sum(mis_pos(1,:).^2 + mis_pos(2,:).^2) / size(X,2) / area_of_map;
        str = sprintf('%0.20f mismatch   R %0.6f   Q %0.6f    numNodes %d    meas"missing"rate %0.5f', mis_match, factor_R, factor_Q, nodes_Nums, meas_missing_rate);
        title(str);
        str = [str, '   .fig'];
        savefig(h,str);
    end


end