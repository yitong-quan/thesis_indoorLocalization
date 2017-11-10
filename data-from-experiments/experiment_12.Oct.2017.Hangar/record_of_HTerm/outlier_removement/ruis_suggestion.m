experNum= 4;

switch experNum
    case 4
        % X_ekf_output = importdata('exper4/estimated_posi_with_timeSt_EKF_experi4_0.8_1.mat');
        %  X_ekf_output = importdata('exper4/estimated_posi_with_timeSt_EKF_experi4_2_1.mat');
        X_ekf_output = importdata('exper4/estimated_posi_with_timeSt_EKF_experi4_5_1.mat');
        % RESIDUAL = importdata('exper4/exper4_RESIDUIAL_0.8_1.mat');
        % RESIDUAL = importdata('exper4/exper4_RESIDUIAL_2_1.mat');
        RESIDUAL = importdata('exper4/exper4_RESIDUIAL_5_1.mat');
        measurements = importdata('../data_t_dist_p4_circle_t.mat');
    case 6
        RESIDUAL = importdata('exper4/exper&_RESIDUIAL.mat');
    otherwise
        error('---------------!!!!!-------experNum not correct. not such file')
end

num_meas_each_time = sum(~isnan(RESIDUAL'), 2);
index_0_meas = find(num_meas_each_time==0);
index_1_meas = find(num_meas_each_time==1);
index_2_meas = find(num_meas_each_time==2);
index_3_meas = find(num_meas_each_time==3);
index_4_meas = find(num_meas_each_time==4);
index_5_meas = find(num_meas_each_time==5);

figure;
subplot(2,1,1)

plot(X_ekf_output(1,:), 'c');
hold on;
plot(index_0_meas, X_ekf_output(1, index_0_meas),'sr');
plot(index_1_meas, X_ekf_output(1, index_1_meas),'sk');
plot(index_2_meas, X_ekf_output(1, index_2_meas),'dr');
plot(index_3_meas, X_ekf_output(1, index_3_meas),'vm');
plot(index_4_meas, X_ekf_output(1, index_4_meas),'ob');
plot(index_5_meas, X_ekf_output(1, index_5_meas),'+g');
legend('x posi', '0 measurement', '1 measurement', '2 measurements', '3 measurements', '4 measurements', '5 measurements');
title('y posi(UWB) v.s. #measurements');
subplot(2,1,2)
hold on;
plot(X_ekf_output(2,:), 'c');
plot(index_0_meas, X_ekf_output(2, index_0_meas),'sr');
plot(index_1_meas, X_ekf_output(2, index_1_meas),'sk');
plot(index_2_meas, X_ekf_output(2, index_2_meas),'dr');
plot(index_3_meas, X_ekf_output(2, index_3_meas),'vm');
plot(index_4_meas, X_ekf_output(2, index_4_meas),'ob');
plot(index_5_meas, X_ekf_output(2, index_5_meas),'+g');
legend('y posi', '0 measurement', '1 measurement', '2 measurements', '3 measurements', '4 measurements', '5 measurements');
title('y posi(UWB) v.s. #measurements');

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

subplot(2,1,2);
plot(std_RESIDUAL - mad_RESIDUAL, '-r');
legend('std - mad');
title('std(RESIDUAL) - mad(RESIDUAL)');

figure;
plot(measurements(:,2:end));
legend('node2', 'node3', 'node1', 'node5', 'node6');

figure;
subplot(2,2,1)
plot(X_ekf_output(1,:), X_ekf_output(2,:), '-+c');
hold on;
daspect([10,10,10]);

subplot(2,2,3);
plot(std_RESIDUAL - mad_RESIDUAL, '-r');
hold on;
legend('std - mad');
title('std(RESIDUAL) - mad(RESIDUAL)');

subplot(2,2,2)
plot(X_ekf_output(1,:), 'c');
hold on;
plot(index_0_meas, X_ekf_output(1, index_0_meas),'sr');
plot(index_1_meas, X_ekf_output(1, index_1_meas),'sk');
plot(index_2_meas, X_ekf_output(1, index_2_meas),'dr');
plot(index_3_meas, X_ekf_output(1, index_3_meas),'vm');
plot(index_4_meas, X_ekf_output(1, index_4_meas),'ob');
plot(index_5_meas, X_ekf_output(1, index_5_meas),'+g');
legend('x posi', '0 meas', '1 meas', '2 meas', '3 meas', '4 meas', '5 meas');
title('y posi(UWB) v.s. #measurements');

subplot(2,2,4)
hold on;
plot(X_ekf_output(2,:), 'c');
plot(index_0_meas, X_ekf_output(2, index_0_meas),'sr');
plot(index_1_meas, X_ekf_output(2, index_1_meas),'sk');
plot(index_2_meas, X_ekf_output(2, index_2_meas),'dr');
plot(index_3_meas, X_ekf_output(2, index_3_meas),'vm');
plot(index_4_meas, X_ekf_output(2, index_4_meas),'ob');
plot(index_5_meas, X_ekf_output(2, index_5_meas),'+g');
legend('y posi', '0 meas', '1 meas', '2 meas', '3 meas', '4 meas', '5 meas');
title('y posi(UWB) v.s. #measurements');

    for j = 2:size(X_ekf_output,2) %1:size(X,2)-9 
        subplot(2,2,1)
        h2 = plot(X_ekf_output(1,j-1:j), X_ekf_output(2,j-1:j), '-ob'); %h2 = plot(X(1,j:j+9), X(2,j:j+9), '-+r');
            str_title0 = sprintf('step j = %d', j);
    title(str_title0);
        subplot(2,2,2)
        h3 = plot([j-1:j], X_ekf_output(1,j-1:j), '*k');
        subplot(2,2,3)
        h4 = plot([j-1:j], std_RESIDUAL(j-1:j) - mad_RESIDUAL(j-1:j), '*k');
        subplot(2,2,4)
        h5 = plot([j-1:j], X_ekf_output(2,j-1:j), '*k');

        pause(0.1);
        delete(h2); delete(h3); delete(h4); delete(h5);   
    end


%{
experNum= 6;

switch experNum
    case 4
        X_ekf_output = importdata('../estimated_posi_with_timeSt_EKF_experi4.mat');
        measurements_t = importdata('../data_t_dist_p4_circle_t.mat');
    case 6
        X_ekf_output = importdata('../estimated_posi_with_timeSt_EKF_experi6.mat');
        measurements_t = importdata('../data_t_dist_p6_acht_slow_t.mat');
    otherwise
        error('---------------!!!!!-------experNum not correct. not such file')
end
measurements = measurements_t(:,2:end);
num_meas_each_time = sum(~isnan(measurements), 2);
index_0_meas = find(num_meas_each_time==0);
index_1_meas = find(num_meas_each_time==1);
index_2_meas = find(num_meas_each_time==2);
index_3_meas = find(num_meas_each_time==3);
index_4_meas = find(num_meas_each_time==4);
index_5_meas = find(num_meas_each_time==5);

figure;
subplot(2,1,1)

plot(X_ekf_output(1,:), 'c');
hold on;
plot(index_0_meas, X_ekf_output(1, index_0_meas),'sr');
plot(index_1_meas, X_ekf_output(1, index_1_meas),'sk');
plot(index_2_meas, X_ekf_output(1, index_2_meas),'dr');
plot(index_3_meas, X_ekf_output(1, index_3_meas),'vm');
plot(index_4_meas, X_ekf_output(1, index_4_meas),'ob');
plot(index_5_meas, X_ekf_output(1, index_5_meas),'+g');
legend('x posi', '0 measurement', '1 measurement', '2 measurements', '3 measurements', '4 measurements', '5 measurements');
title('y posi(UWB) v.s. #measurements');
subplot(2,1,2)
hold on;
plot(X_ekf_output(2,:), 'c');
plot(index_0_meas, X_ekf_output(2, index_0_meas),'sr');
plot(index_1_meas, X_ekf_output(2, index_1_meas),'sk');
plot(index_2_meas, X_ekf_output(2, index_2_meas),'dr');
plot(index_3_meas, X_ekf_output(2, index_3_meas),'vm');
plot(index_4_meas, X_ekf_output(2, index_4_meas),'ob');
plot(index_5_meas, X_ekf_output(2, index_5_meas),'+g');
legend('y posi', '0 measurement', '1 measurement', '2 measurements', '3 measurements', '4 measurements', '5 measurements');
title('y posi(UWB) v.s. #measurements');

leng_meas = length(measurements);
std_measure = zeros(leng_meas,1);
mad_measuer = zeros(leng_meas,1);
for i=1:leng_meas
    tmp = measurements(i,:);
    tmp(isnan(tmp))=[];
    std_measure(i) = std(tmp,[],2);
    mad_measuer(i) = mad(tmp,[],2);
end
figure;
subplot(2,1,1)
plot(std_measure, '-g');
hold on;
plot(mad_measuer, '-b');
title('std(measurements) v.s. mad(measurements)');
legend('std', 'mad');
subplot(2,1,2);
plot(std_measure - mad_measuer, '-r');
legend('std - mad');
title('std(measurements) - mad(measurements)');

figure;
plot(measurements);
legend('node2', 'node3', 'node1', 'node5', 'node6');

% n = length(measurements);
% figure;hold on;
% adj = 5;
% sds = 2.3;
% [y,i,xmedian,xsigma] = hampel(measurements(:,1), adj, sds); %x_p(2,2), (y_p,3,0.9)
% plot([1;1]*xmedian'+sds*[-1;1]*xsigma');
% plot(find(i),dist(i),'sk');
%}