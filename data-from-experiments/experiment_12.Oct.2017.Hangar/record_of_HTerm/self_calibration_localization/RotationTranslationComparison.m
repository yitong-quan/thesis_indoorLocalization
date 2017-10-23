%% using only the   ---nodes positions---   for self-calibration
% nodes position estimated by the file 'cali_free_lsqnonlin_importDistData_12octHangar.m'
% compare with the hand measured value
% reference coordinate hand measured value

% file locates in 'D:\Yitong\GitHub\thesis_indoorLocalization\data-from-experiments\experiment_12.Oct.2017.Hangar\record_of_HTerm\self_calibration_localization\...'
clear
%% process data
expNum = 5;

str_self_calib_node = ['results/remove dataSet with NaN element/exper', num2str(expNum), '_opt_node.mat'];
data_self_calib_node = importdata(str_self_calib_node);
node_po_hand_meas = importdata('../nodePos_by_determineNodesPositionBaseOnDistToEachOthers.mat');

str_US = ['../../record_of_ultra_sound_system/positionBeforeMakedBetterFromJoan/mat_positions', num2str(expNum), '.mat'];
% str_US = ['../../record_of_ultra_sound_system/positionsBetterFromJoan/mat_positions', num2str(expNum), '.mat'];
data_US = importdata(str_US);
data_US = data_US';
data_US_p = data_US(2:3,:);
str_RT2hand_measure_coordinate = ['../forAlignment\beforeBetterFromJoan/forAlignment_RT_ASSIS_2_UWB_exp', num2str(expNum), '.mat'];
M_RT2h_m = importdata(str_RT2hand_measure_coordinate);
data_US_p_RT2h_m = M_RT2h_m(:,[1,2]) *data_US_p + M_RT2h_m(:,end);

%{
% replace outliar with the closest 'correct' position
% data_US(2:3,1:3) = repmat(data_US(2:3,4),1,3);
% % nearestpoint(shortArray,longArray)
% timeStampNeed_data_US = nearestpoint(data_self_calib(5,:), data_US(4,:));
%{
% check for timeStampNeed_data_US
diff=zeros(1,size(data_self_calib,2));
for i = 1:size(data_self_calib,2)
    diff(i) = data_self_calib(5,i) - data_US(4, timeStampNeed_data_US(i));
end
%}

% data with the row_index of timeStampNeed_data_US
data_US_trimed = data_US(:,timeStampNeed_data_US);
%}

%% (M)calculate the translateion matrix R and T
% M0 = [ [cos(theta), -sin(theta); sin(theta), cos(theta)], [t1;t2]];
x0 = 10*rand(1,3); % M = [theta, t1, t2]
options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','Display','iter','MaxIterations',2000);
resnorm_last = inf;
for ii = 1:6
    [x,resnorm] = lsqnonlin(@myfun,x0,[],[],options);
    if resnorm < resnorm_last
        M = [ [cos(x(1)), -sin(x(1)); sin(x(1)), cos(x(1))], x(2:3)'];
        x_opt = x;
        resnorm_opt = resnorm;
        resnorm_last = resnorm_opt;        
    end

%% set next starting point   
    x0 = x + 10*rand(size(x));
end



%% plot nodes positions of self _calib and hand meas
data_self_calib_node_after_RT = M(:,1:2) * data_self_calib_node + M(:,end);
figure; plot(node_po_hand_meas(1,:), node_po_hand_meas(2,:), 'r-d');
hold on;
plot(data_self_calib_node(1,:), data_self_calib_node(2,:),'g-d'); 
plot(data_self_calib_node_after_RT(1,:), data_self_calib_node_after_RT(2,:), 'b-d');
legend('node hand meas', 'node self calib', 'node self calib after RT');
    daspect([10,10,10]);


%% plot the location of self-calibration-results and the location of the USS
afterRT_data_US_trimed = M(:,[1,2]) * data_US_trimed(2:3,:) + M(:,end);
figure; plot(data_self_calib_node(1,:), data_self_calib_node(2,:),'r-o'); hold on;
plot(afterRT_data_US_trimed(1,:), afterRT_data_US_trimed(2,:), 'b-+');
plot(data_US_trimed(2,:), data_US_trimed(3,:), 'g-+');
 legend('EKF', 'USS trimed', 'USS trimed RT');
title_stri = ['exp', num2str(expNum), '  theta=', num2str(x(1)), '  t1=', num2str(x(2)), '  t2=', num2str(x(3))];
title(title_stri);
daspect([10,10,10]);

%% plot the traj of EKF(moving) and the location of the USS(after RT)
figure; hold on;
title_stri = ['exp', num2str(expNum), ' comparision of EKF and USS'];
title(title_stri);
plot(afterRT_data_US_trimed(1,:), afterRT_data_US_trimed(2,:), 'b+');
% plot the circle 
circle_center = importdata('../circleCenterPos_by_determineCircleCenterPositionBaseOnDistToEachOthers.mat');
circle_angle = [0:pi/50:2*pi];
circle_x = circle_center(1)+2.5*cos(circle_angle); %unit m
circle_y = circle_center(2)+2.5*sin(circle_angle); %unit m
plot(circle_x,circle_y, 'g');
        daspect([10,10,10]);
time_diff = diff(data_self_calib_node(5,:))'; % unit second
pause(3);
	pause_time = 0.002*[time_diff; 2];
    for j = 5:size(data_self_calib_node,2) %1:size(X,2)-9 
        
        h3 = plot(afterRT_data_US_trimed(1,j-4:j), afterRT_data_US_trimed(2,j-4:j), '-ob');
        hold on;
        h2 = plot(data_self_calib_node(1,j-4:j), data_self_calib_node(2,j-4:j), '-or'); %h2 = plot(X(1,j:j+9), X(2,j:j+9), '-+r');
        legend('USS loc', 'circle marker', 'USS', 'EKF');

        Fram(j-4) = getframe(gcf);
        
        % pause(pause_time(j));
        delete(h2);
        delete(h3);
    end
    video_str = ['video_comparision', num2str(expNum), '.avi'];
    video = VideoWriter(video_str);
    open(video)
    writeVideo(video, Fram)
    close(video)


%%
% afterRT = R0 * nodes_optimal + t0;
function F = myfun(x)

expNum = 5;

str_self_calib_node = ['results/remove dataSet with NaN element/exper', num2str(expNum), '_opt_node.mat'];
data_self_calib_node = importdata(str_self_calib_node);
node_po_hand_meas = importdata('../nodePos_by_determineNodesPositionBaseOnDistToEachOthers.mat');
%{
% check for timeStampNeed_data_US
diff=zeros(1,size(data_self_calib,2));
for i = 1:size(data_self_calib,2)
    diff(i) = data_self_calib(5,i) - data_US(4, timeStampNeed_data_US(i));
end
%}

M = [ [cos(x(1)), -sin(x(1)); sin(x(1)), cos(x(1))], x(2:3)'];
afterRT = M(:,[1,2]) * data_self_calib_node + M(:,end);
dif = afterRT - node_po_hand_meas;
mis_dist = zeros(length(dif));
for i = 1:length(mis_dist)
    mis_dist(i) = norm(dif(:,i));
end
F = mis_dist;
end

