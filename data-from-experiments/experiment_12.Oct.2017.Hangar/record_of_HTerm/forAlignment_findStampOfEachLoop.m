data_ekf = importdata('estimated_posi_with_timeSt_EKF_experi6.mat');
data_US = importdata('../record_of_ultra_sound_system/mat_positions6.mat');
data_US = data_US';
% nearestpoint(shortArray,longArray)
timeStampNeed_data_US = nearestpoint(data_ekf(5,:), data_US(4,:));
%{
% check for timeStampNeed_data_US
diff=zeros(1,size(data_ekf,2));
for i = 1:size(data_ekf,2)
    diff(i) = data_ekf(5,i) - data_US(4, timeStampNeed_data_US(i));
end
%}
% data with the row_index of timeStampNeed_data_US
data_US_trimed = data_US(:,timeStampNeed_data_US);
