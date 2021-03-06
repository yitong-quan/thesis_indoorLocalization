clear
%% calculate the translateion matrix R and T
expNum = 4;

str_ekf = ['../traj_recovered_ekf_experiment_old_Q1_R100/estimated_posi_with_timeSt_EKF_experi', num2str(expNum), '.mat'];
data_ekf = importdata(str_ekf);
str_US = ['../../record_of_ultra_sound_system/positionBeforeMakedBetterFromJoan/mat_positions', num2str(expNum), '.mat'];
% str_US = ['../../record_of_ultra_sound_system/positionsBetterFromJoan/mat_positions', num2str(expNum), '.mat'];
data_US = importdata(str_US);
data_US = data_US';
% replace outliar with the closest 'correct' position
data_US(2:3,1:3) = repmat(data_US(2:3,4),1,3);
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

% M0 = [ [cos(theta), -sin(theta); sin(theta), cos(theta)], t0];
x0 = 100*rand(1,3); % M = [theta, t1, t2]
options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','Display','iter','MaxIterations',2000);
[x,resnorm] = lsqnonlin(@myfun,x0,[],[],options);
M = [ [cos(x(1)), -sin(x(1)); sin(x(1)), cos(x(1))], x(2:3)'];
%% plot the location of EKF and the location of the USS
afterRT_data_US_trimed = M(:,[1,2]) * data_US_trimed(2:3,:) + M(:,end);
h1 = figure; plot(data_ekf(1,:), data_ekf(2,:),'r-o'); hold on;
plot(afterRT_data_US_trimed(1,:), afterRT_data_US_trimed(2,:), 'b-+');
% plot(data_US_trimed(2,:), data_US_trimed(3,:), 'g-+'); no need to plot the original position of Assist-data
legend('UWB', 'Assist');
title_stri = ['exp', num2str(expNum), '  theta=', num2str(x(1)), '  t1=', num2str(x(2)), '  t2=', num2str(x(3))];
title(title_stri);
xlabel({'x / m'});
ylabel({'y / m'});
daspect([10,10,10]);
if expNum == 4
    savefig(h1, 'beforeBetterFromJoan\outlier_removed_forAlignment_RT_ASSIS_2_UWB_exp4.fig');
else
    str_fig = ['afterBetterFromJoan\outlier_removed_forAlignment_RT_ASSIS_2_UWB_exp', num2str(expNum), '.fig'];
    savefig(h1, str_fig);
end    

%% plot the traj of EKF(moving) and the location of the USS(after RT)
figure; hold on;
title_stri = ['exp', num2str(expNum), ' comparision of UWB and ASSIST'];
title(title_stri);
plot(afterRT_data_US_trimed(1,:), afterRT_data_US_trimed(2,:), 'c');
plot(data_ekf(1,:), data_ekf(2,:), 'g');
xlabel({'x / m'});
ylabel({'y / m'});
% if expNum == 5 || expNum == 6
% else
% % plot the circle 
% circle_center = importdata('../circleCenterPos_by_determineCircleCenterPositionBaseOnDistToEachOthers.mat');
% circle_angle = [0:pi/50:2*pi];
% circle_x = circle_center(1)+2.5*cos(circle_angle); %unit m
% circle_y = circle_center(2)+2.5*sin(circle_angle); %unit m
% plot(circle_x,circle_y, 'g');
% end
        daspect([10,10,10]);

time_diff = diff(data_ekf(5,:))'; % unit second
pause(3);
	pause_time = 0.01*[time_diff; 2];
    for j = 2:size(data_ekf,2) %1:size(X,2)-9 
        
        h3 = plot(afterRT_data_US_trimed(1,j-1:j), afterRT_data_US_trimed(2,j-1:j), '-ob');
        hold on;
        h2 = plot(data_ekf(1,j-1:j), data_ekf(2,j-1:j), '-or'); %h2 = plot(X(1,j:j+9), X(2,j:j+9), '-+r');
        legend('ASSIST traj', 'UWB traj', 'ASSIST', 'UWB');
        
        Fram(j-1) = getframe(gcf);
        
        % pause(pause_time(j));
        delete(h2);
        delete(h3);
    end
    video_str = ['outlier_removed_video_comparision', num2str(expNum), '.avi'];
    video = VideoWriter(video_str);
    open(video)
    writeVideo(video, Fram)
    close(video)


%%
% afterRT = R0 * nodes_optimal + t0;
function F = myfun(x)

expNum = 4;
str_ekf = ['../traj_recovered_ekf_experiment_old_Q1_R100/estimated_posi_with_timeSt_EKF_experi', num2str(expNum), '.mat'];
str_US = ['../../record_of_ultra_sound_system/positionBeforeMakedBetterFromJoan/mat_positions', num2str(expNum), '.mat'];
% str_US = ['../../record_of_ultra_sound_system/positionsBetterFromJoan/mat_positions', num2str(expNum), '.mat'];

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

