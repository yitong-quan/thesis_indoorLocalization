experNum= 4;

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
index_1_meas = find(num_meas_each_time==1);
index_2_meas = find(num_meas_each_time==2);
index_3_meas = find(num_meas_each_time==3);
index_4_meas = find(num_meas_each_time==4);
index_5_meas = find(num_meas_each_time==5);

figure;
subplot(2,1,1)

plot(X_ekf_output(1,:), 'c');
hold on;
plot(index_1_meas, X_ekf_output(1, index_1_meas),'sk');
plot(index_2_meas, X_ekf_output(1, index_2_meas),'dr');
plot(index_3_meas, X_ekf_output(1, index_3_meas),'vm');
plot(index_4_meas, X_ekf_output(1, index_4_meas),'ob');
plot(index_5_meas, X_ekf_output(1, index_5_meas),'+g');
legend('x posi', '1 measurement', '2 measurements', '3 measurements', '4 measurements', '5 measurements');
title('y posi(UWB) v.s. #measurements');
subplot(2,1,2)
hold on;
plot(X_ekf_output(2,:), 'c');
plot(index_1_meas, X_ekf_output(2, index_1_meas),'sk');
plot(index_2_meas, X_ekf_output(2, index_2_meas),'dr');
plot(index_3_meas, X_ekf_output(2, index_3_meas),'vm');
plot(index_4_meas, X_ekf_output(2, index_4_meas),'ob');
plot(index_5_meas, X_ekf_output(2, index_5_meas),'+g');
legend('y posi', '1 measurement', '2 measurements', '3 measurements', '4 measurements', '5 measurements');
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
plot(std_measure, '-+g');
hold on;
plot(mad_measuer, '-+b');
title('std(measurements) v.s. mad(measurements)');
subplot(2,1,2);
plot(std_measure - mad_measuer, '-+r');
title('std(measurements) - mad(measurements)');
