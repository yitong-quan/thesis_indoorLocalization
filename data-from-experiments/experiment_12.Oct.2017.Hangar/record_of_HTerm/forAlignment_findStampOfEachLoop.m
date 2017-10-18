expNum = 1;
str_ekf = ['estimated_posi_with_timeSt_EKF_experi', num2str(expNum), '.mat'];
str_US = ['../record_of_ultra_sound_system/mat_positions', num2str(expNum), '.mat'];
data_ekf = importdata(str_ekf);
data_US = importdata(str_US);
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

%R0 = zeros(2);
% theta = 0;
% t0 = zeros(2,1);
% M0 = [ [cos(theta), -sin(theta); sin(theta), cos(theta)], t0];
x0 = zeros(1,3); % M = [theta, t1, t2]
options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','Display','iter','MaxIterations',2000);
[x,resnorm] = lsqnonlin(@myfun,x0,[],[],options);
M = [ [cos(x(1)), -sin(x(1)); sin(x(1)), cos(x(1))], x(2:3)'];
afterRT_data_US_trimed = M(:,[1,2]) * data_US_trimed(2:3,:) + M(:,end);
figure; plot(data_ekf(1,:), data_ekf(2,:),'r-o'); hold on;
plot(afterRT_data_US_trimed(1,:), afterRT_data_US_trimed(2,:), 'b-+');
plot(data_US_trimed(2,:), data_US_trimed(3,:), 'g-+');
title_stri = ['exp', num2str(expNum), '  theta=', num2str(x(1)), '  t1=', num2str(x(2)), '  t2=', num2str(x(3))];
title(title_stri);
   daspect([10,10,10]);

% afterRT = R0 * nodes_optimal + t0;
function F = myfun(x)

expNum = 1;
str_ekf = ['estimated_posi_with_timeSt_EKF_experi', num2str(expNum), '.mat'];
str_US = ['../record_of_ultra_sound_system/mat_positions', num2str(expNum), '.mat'];
data_ekf = importdata(str_ekf);
data_US = importdata(str_US);
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
M = [ [cos(x(1)), -sin(x(1)); sin(x(1)), cos(x(1))], x(2:3)'];
afterRT = M(:,[1,2]) * data_US_trimed(2:3,:) + M(:,end);
dif = afterRT - data_ekf(1:2,:);
mis_dist = zeros(length(dif));
for i = 1:length(mis_dist)
    mis_dist(i) = norm(dif(:,i));
end
F = mis_dist;
end

